# 导览机器人语音交互（learn2_Z）

基于 ROS 2 Jazzy 的展厅导览机器人语音交互工作空间。机器人听懂访客说话，带访客前往展厅点位，到达后询问是否还有事，访客说“没有了”就回到起点待命。

```
访客说话 → 平台 ASR 转文字 → 本地规则拆解 + 大模型兜底理解
        → 下发导航任务给底盘 → 到点后语音播报并询问 → 访客说没有则回待命点
```

核心原则：

- 硬决策在本地，软决策给模型：导航判断用代码规则，模型只处理模糊表达
- 模型输出必须校验：地点名必须真实存在于当前地图
- 模型永远不直接指挥底盘
- 所有请求走独立工作线程串行处理，避免阻塞 ROS 回调

## 运行环境

| 项 | 值 |
|---|---|
| 设备 | Jetson AGX Thor Developer Kit |
| 系统 | JetPack 7 / aarch64 |
| ROS | ROS 2 Jazzy（rclpy） |
| 语音 | 平台讯飞 AIUI（4 麦阵列）+ 平台 TTS |
| 大模型 | 远程 vLLM（OpenAI 兼容接口） |
| 底盘 | `robot_controller` SDK，REST 默认 `192.168.11.10:9090` |

## 目录结构

```
learn2_Z/
├── README.md
├── docx/
│   ├── PROJECT_GUIDE.md      # 实现说明
│   └── TROUBLESHOOTING.md    # 问题与解决记录
└── src/nav_pkg/
    ├── config/places.yaml    # 地点别名、待命点、主线配置
    ├── nav_pkg/
    │   ├── tour_chat.py      # 主节点：语音 → 理解 → 导航 → 播报
    │   ├── qwen.py           # 大模型客户端（唯一的模型 HTTP 出口）
    │   ├── places.py         # 中文别名 → 地图点位
    │   ├── nav_route.py      # 底盘导航封装 + 独立导航入口
    │   └── chat_say.py       # TTS 单句测试工具
    ├── setup.py              # 三个入口：tour_chat / nav_route / chat_say
    └── package.xml
```

## 快速开始

```bash
cd ~/guide_robot/learn/learn2_Z
source /opt/ros/jazzy/setup.bash
source ~/xos/setup.bash          # 平台消息、TTS、语音节点
colcon build --packages-select nav_pkg
source install/setup.bash
```

启动主节点（默认带导航）：

```bash
ros2 run nav_pkg tour_chat
```

只测对话、不动底盘：

```bash
ros2 run nav_pkg tour_chat --ros-args -p enable_nav:=false
```

没有麦克风时用文本模拟输入：

```bash
ros2 topic pub --once /tour/chat std_msgs/msg/String "{data: '带我去创想谷'}"
ros2 topic pub --once /tour/chat std_msgs/msg/String "{data: '没有了'}"
```

单独测试：

```bash
# 只测导航，不经过语音和大模型
ros2 run nav_pkg nav_route

# 只测 TTS
ros2 run nav_pkg chat_say --ros-args -p text:="你好"
```

常用参数覆盖：

```bash
ros2 run nav_pkg tour_chat --ros-args \
  -p map:=消防 \
  -p track:=track_line_1003 \
  -p qwen_base_url:=http://127.0.0.1:8000/v1
```

## 语音识别（ASR）

当前使用**平台讯飞 AIUI 的 ASR**，不是本地 whisper。

| 项 | 说明 |
|---|---|
| 识别结果 | 订阅 `/audio_asr/iat`（`lyre_msgs/AsrIat`） |
| 使能 | 每 0.5 秒向 `/intelligent_interaction/lyre/enable` 发 `true` |
| 唤醒 | 人脸唤醒，`/audio_asr/keyword` 仅记录日志 |
| 拾音范围 | 4 麦线性阵列，正前方约 ±45°，侧后方无效 |

关键点：平台 ASR **默认关闭**，业务节点必须持续发使能心跳，停发即被关回，表现为“说话毫无反应”。退出时节点会发 `false` 还原现场。

> 历史提交中曾尝试本地 faster-whisper 识别，该实现已移除，相关参数（`enable_mic` 等）不再存在。

## 交互流程

| 场景 | 触发 | 行为 |
|---|---|---|
| 明确指令 | “带我去创想谷” | 直接导航 |
| 模糊表达 | “创想谷” | 反问是否需要带路 |
| 确认/否认 | “好” / “不用” | 出发 / 放弃 |
| 间接意图 | “我想练跆拳道” | 大模型推断地点后反问 |
| 到达 | 导航完成 | 播报并询问“现在还有什么事吗” |
| 无事 | “没有了 / 没有 / 没事了” | 返回待命点 `standby` |
| 其它 | 任意闲聊 | 大模型自由对话 |

导航期间会忽略新的语音输入，避免中途改目标；播报结束后保留短暂静默，防止喇叭尾音被识别成指令。

## 主要参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `asr_topic` | `/audio_asr/iat` | 平台识别结果 |
| `enable_asr` | `True` | 是否发 ASR 使能心跳 |
| `chat_topic` | `/tour/chat` | 文本直连 |
| `tts_service` | `/intelligent_interaction/tts/play` | 播报服务 |
| `enable_voice` | `True` | 是否启用 TTS |
| `enable_greet` | `True` | 启动问候 |
| `min_chars` | `2` | 最短有效字数 |
| `dedupe_window` | `2.0` | 去重窗口（秒） |
| `qwen_base_url` | `http://120.48.75.178:7006/v1` | 模型地址 |
| `qwen_model` | `Qwen3.5-122B-A10B` | 模型名 |
| `qwen_timeout` | `30.0` | 请求超时 |
| `history_turns` | `6` | 闲聊历史轮数 |
| `enable_nav` | `True` | 是否启用导航 |
| `host` / `rest_port` | `192.168.11.10` / `9090` | 底盘 REST |
| `map` | `消防` | 地图名 |
| `reloc_poi` | `乐知教育` | 重定位参考点 |
| `track` | `track_line_1003` | 虚拟轨道 |
| `nav_timeout` | `180.0` | 单次导航超时 |
| `places_file` | `""` | 空则使用包内 `config/places.yaml` |
| `log_file` | `""` | 空则写入 `~/guide_logs/` |

## 对外接口

订阅：

| 话题 | 类型 | 用途 |
|---|---|---|
| `/audio_asr/iat` | `lyre_msgs/AsrIat` | 语音识别文字 |
| `/audio_asr/keyword` | `lyre_msgs/AsrKeyword` | 唤醒事件 |
| `/tour/chat` | `std_msgs/String` | 文本输入 |

发布：

| 话题 | 类型 | 用途 |
|---|---|---|
| `/intelligent_interaction/lyre/enable` | `std_msgs/Bool` | ASR 使能心跳 |

服务：

| 服务 | 类型 | 用途 |
|---|---|---|
| `/intelligent_interaction/tts/play` | `interaction_msgs/TtsService` | append / query |

外部 HTTP：

| 目标 | 端点 |
|---|---|
| 大模型 | `POST {qwen_base_url}/chat/completions` |
| 底盘导航 | `PUT /api/core/navigation/v1/actions/moveto_poi` |
| 底盘状态 | `GET /api/core/motion/v1/actions/{action_id}` |

## 地点配置

`config/places.yaml` 维护点位别名与路线：

```yaml
standby: 乐知教育

aliases:
  乐知教育: 乐知教育
  乐知院: 乐知教育
  核心跑酷: 核心力量跑酷
  3号点: 创想谷
  卫生间: 厕所
```

- `standby`：访客说完事后返回的待命点
- `aliases`：中文别名 → 地图点位，支持一个点位多个别名
- `transfer_points`：只作中转，不出现在可去地点清单
- `main_line` / `branches`：主线与支线顺序（当前仅解析，尚未用于逐点巡线）

## 日志

每句话都会落盘：

```
~/guide_logs/tour_chat_YYYYMMDD.log
```

## 已知限制

| # | 限制 | 影响 |
|---|---|---|
| 1 | 模型请求无重试、无备用端点 | 网络抖动即失联 |
| 2 | 模型服务为明文 HTTP 且无鉴权 | 存在被滥用风险 |
| 3 | 模型超时 30 秒偏长 | 排队阻塞体验差 |
| 4 | 对话历史不重置 | 跨访客上下文可能串味 |
| 5 | `main_line` / `branches` 未接入导航 | 无法逐点巡线讲解 |
| 6 | 平台 ASR 仅前向拾音 | 侧后方说话无效 |
| 7 | 导航期间忽略新指令 | 需到点后再下新目标 |

## 相关文档

- [PROJECT_GUIDE.md](docx/PROJECT_GUIDE.md)：实现说明与设计决策
- [TROUBLESHOOTING.md](docx/TROUBLESHOOTING.md)：实际遇到的问题与排查方法

## 常见问题

| 现象 | 排查 |
|---|---|
| 说话完全没反应 | 确认 `/intelligent_interaction/lyre/enable` 每 0.5s 有 `true`，并已唤醒 |
| 有识别但不回应 | 可能处于导航/播报中，被“忙”过滤 |
| 大模型无回应 | 检查模型服务可达性，查看日志错误 |
| 不动 | 底盘健康检查未通过或定位未就绪 |
| 走一半停下 | 前方障碍，听“请让一让” |
| 到点报失败 | 已按 1 米容差兜底，可适当放宽 |
