# 导览机器人语音交互系统 — 实现说明

> 适用代码：`/home/nvidia/guide_robot/learn/learn2_Z/src/nav_pkg`
> 主程序：`nav_pkg/tour_chat.py`　配套：`qwen.py` / `places.py` / `nav_route.py`
> 目标平台：Jetson AGX Thor + JetPack 7 + ROS 2 Jazzy

---

## 1. 项目概览

本系统让一台导览机器人能够**听懂访客说话、理解意图、带路到展厅点位、并语音回应**。

一句话流程：

```
访客说话 → 平台 ASR 转文字 → 本地规则拆解 + 大模型兜底理解
        → 下发导航任务给底盘 → 到点后用大模型生成播报 → TTS 说出
```

核心设计原则（贯穿全文）：

| 原则 | 含义 |
|---|---|
| **硬决策在本地，软决策给模型** | 导航判断用代码规则，模型只处理模糊表达 |
| **模型输出必须被校验** | 模型给的地点名要在地图上真实存在才生效 |
| **导航不依赖模型** | 模型挂了链路失效，但导航仍可独立工作 |
| **单线程串行** | 所有请求排队处理，避免历史错乱和状态竞争 |

---

## 2. 目录与文件职责

```
learn2_Z/src/nav_pkg/
├── config/places.yaml        # 地点别名、待命点、主线、支线（数据配置）
├── nav_pkg/
│   ├── tour_chat.py          # ★ 主节点：语音→理解→导航→播报
│   ├── qwen.py               # 大模型客户端（唯一的 HTTP 出口）
│   ├── places.py             # 地点别名解析（PlaceBook）
│   ├── nav_route.py          # 底盘导航封装（TourNavigator）+ 独立导航入口
│   └── chat_say.py           # TTS 单句测试工具
├── setup.py                  # 三个入口：tour_chat / nav_route / chat_say
└── package.xml
```

| 文件 | 是否自研 | 作用 |
|---|---|---|
| `tour_chat.py` | 自研 | 业务总调度，唯一指挥者 |
| `qwen.py` | 自研 | 封装 OpenAI 兼容协议 |
| `places.py` | 自研 | 中文别名 → 地图点位 |
| `nav_route.py` | 自研封装 | 调用厂家 SDK，做状态机与容错 |
| `robot_controller` SDK | **厂家自带** | 底盘 REST 客户端 |
| 底盘导航模块 | **厂家自带（闭源）** | SLAM / 规划 / 避障 / 控制 |

---

## 3. 运行方式

本项目**没有 launch 文件**，直接 `ros2 run` + 参数覆盖：

```bash
# 编译
cd ~/guide_robot/learn/learn2_Z
colcon build --packages-select nav_pkg
source install/setup.bash

# 启动主节点（全部用默认参数）
ros2 run nav_pkg tour_chat

# 文本直连测试（无麦克风也能验证）
ros2 topic pub --once /tour/chat std_msgs/msg/String "{data: '带我去创想谷'}"

# 改参数启动
ros2 run nav_pkg tour_chat --ros-args \
  -p map:=消防 \
  -p track:=track_line_1003 \
  -p qwen_base_url:=http://127.0.0.1:8000/v1

# 只测导航（不经过语音和大模型）
ros2 run nav_pkg nav_route

# 只测 TTS
ros2 run nav_pkg chat_say --ros-args -p text:="你好"
```

日志落在 `~/guide_logs/tour_chat_YYYYMMDD.log`，每句话都记录（终端滚走也能查）。

---

## 4. 总体架构

```
┌──────────────────────────────────────────────────────────┐
│ 平台语音层（厂家）                                        │
│  AIUI：4 麦阵列 → 波束成形 → 唤醒 → ASR → /audio_asr/iat  │
└───────────────────────┬──────────────────────────────────┘
                        │ ROS 话题
┌───────────────────────▼──────────────────────────────────┐
│ 业务层  tour_chat.py                                     │
│  订阅 → 过滤 → 队列 → 工作线程 → 五级决策                 │
└──────┬──────────────────────┬────────────────────────────┘
       │                      │
┌──────▼─────────┐   ┌────────▼───────────────────────────┐
│ 大模型层        │   │ 导航层 nav_route.py                │
│ qwen.py         │   │  TourNavigator                     │
│ HTTP POST       │   │  切图→自检→AEB→下发→轮询→容差      │
└──────┬─────────┘   └────────┬───────────────────────────┘
       │                      │ HTTP REST
┌──────▼─────────┐   ┌────────▼───────────────────────────┐
│ 远程 vLLM       │   │ 底盘 192.168.11.10:9090            │
│ Qwen3.5-122B    │   │  SLAM + 规划 + 避障 + 控制          │
└────────────────┘   └────────────────────────────────────┘
```

---

## 5. 语音获取

### 5.1 输入通道（三条并存）

| 通道 | 话题 | 用途 |
|---|---|---|
| 平台 ASR 结果 | `/audio_asr/iat` | 主通道，需先唤醒 |
| 唤醒词事件 | `/audio_asr/keyword` | 仅记日志（含声源角度） |
| 文本直连 | `/tour/chat` | 调试/无麦克风场景 |

订阅注册位置：`tour_chat.py:226-233`。

### 5.2 关键：使能心跳（最容易踩的坑）

平台 ASR **默认关闭**。必须每 0.5 秒发布一次 `true` 到 `/intelligent_interaction/lyre/enable`，**停发就会被自动关回**，表现为“说话毫无反应”。

```python
# tour_chat.py:237-241
self._enable_pub = self.create_publisher(
    Bool, "/intelligent_interaction/lyre/enable", 10)
self.create_timer(0.5, self._publish_enable)
```

退出时发 `false` 还原现场。

> 平台配置 `aiui_params.yaml` 中 `nlp.enabled: false`、`mode: asr`、`wakeup_mode: face`。
> 唤醒方式是人脸唤醒（头部云台前向约 ±45°），因此**侧后方说话无法唤醒**。

### 5.3 输入过滤（四道闸，`_enqueue` L305-327）

| 顺序 | 过滤 | 作用 |
|---|---|---|
| 1 | `_meaningful(text, 2)` | 去标点后少于 2 字丢弃（“嗯”“啊”） |
| 2 | 导航中 / 播报中 / 播完 2 秒内 | 忙时忽略，避免自问自答 |
| 3 | `_echo_of(text, _last_spoken)` | 喇叭回声过滤（≥8 字且相似度 ≥0.5） |
| 4 | 2 秒去重窗口 | ASR 重复输出只处理一次 |

### 5.4 线程模型

```
ROS 回调（单线程 executor）→ _enqueue() → queue.put()  立即返回
                                    ↓
工作线程 _worker_loop()（daemon）
  queue.get(0.2) → _handle() → 大模型 / 导航 / 播报（阻塞数秒）
```

**为什么必须这样**：大模型一次要 1~3 秒，若在回调里直接调用，整个节点（含 TTS、定时器）会卡死。串行处理同时保证对话历史不被并发写乱。

---

## 6. TTS 播报

`Speech` 类封装 `/intelligent_interaction/tts/play`（`interaction_msgs/TtsService`）：

1. `append` 下发文本（`audio_id="audio_guide"`）
2. 每 0.2 秒 `query` 查询状态
3. 见到 `playing` 后，连续 0.4 秒非 `playing` 判定播完
4. 超时上限 `min(预计时长×2+5, 40)` 秒

播完后 `_speak()` 再置 2 秒静默期（`_speech_until`），防止尾音被麦克风收回当成指令。

---

## 7. 大模型接入

### 7.1 服务信息

| 项 | 值 |
|---|---|
| 地址 | `http://120.48.75.178:7006/v1` |
| 模型 | `Qwen3.5-122B-A10B`（Qwen3 系列 MoE，122B 总参 / 10B 激活） |
| 鉴权 | `Authorization: Bearer EMPTY`（服务端未开启校验） |
| 超时 | 30 秒 |
| 协议 | OpenAI 兼容 `/chat/completions` |

> **注意**：这是**远程自建 vLLM 服务**，不是机器人本地推理，也不是阿里云官方 API。
> 机器人本机没有模型权重，只作为 HTTP 客户端。

### 7.2 `QwenClient` 结构（qwen.py）

| 成员 | 作用 |
|---|---|
| `__init__` | 拼 URL、存 system prompt、初始化历史 `_turns` |
| `chat(text)` | **带历史**闲聊，写回历史 |
| `one_shot(prompt, system)` | **不带历史**，用于意图解析/文案生成 |
| `_complete(messages)` | 真正发 POST，解析 JSON |
| `_remember(user, reply)` | 历史裁剪到 `history_turns*2` 条 |
| `_clean(text)` | 去 markdown、压空白、去自称前缀、截断 200 字 |

### 7.3 请求体

```json
{
  "model": "Qwen3.5-122B-A10B",
  "messages": [
    {"role": "system", "content": "你的名字固定叫「小田」…"},
    {"role": "user", "content": "当前这句话"}
  ],
  "temperature": 0.6,
  "max_tokens": 300,
  "chat_template_kwargs": {"enable_thinking": false}
}
```

**`enable_thinking: false` 是必须的**：Qwen3 默认输出思考过程，不关闭会导致 `content` 为空（token 被思考内容吃光）。

### 7.4 两种用法对比

| | `chat()` | `one_shot()` |
|---|---|---|
| 历史 | 带最近 6 轮 | 无 |
| system | 固定“小田”人设 | 每次可覆盖（如“只输出 JSON”） |
| 调用点 | 闲聊（L387） | 意图解析（L409）、到达播报（L461） |
| 失败处理 | 播报“暂时联系不上大模型” | 返回 None，退化处理 |

### 7.5 失败与降级

| 失败点 | 处理 |
|---|---|
| 连接失败/超时 | `chat` 抛异常 → 播报道歉话术 |
| 返回空内容 | 抛 `RuntimeError("模型返回空内容")` |
| JSON 解析失败 | `_extract_json` 返回 None → 放弃 |
| 地点不在图上 | `resolve()` 返回 None → 放弃 |
| 到达文案失败 | 用固定话术 |

**没有重试，没有多端点兜底**（这是当前实现的一个弱点）。

---

## 8. 意图理解

### 8.1 五级决策（`_handle` L342-392）

| 级别 | 判断者 | 触发 | 动作 |
|---|---|---|---|
| ① | **代码** | 命中地点 + 有“去/到/带我去” | 直接导航 |
| ② | **代码** | 命中地点但表达模糊 | 反问“需要带你去吗” |
| ③ | **代码** | 上一句在征求同意 | “好”出发 / “不用”放弃 |
| ④ | **代码** | 说“没事了” | 回待命点 |
| ⑤ | **大模型** | 有暗示词但拆不出地点 | 推断地点→再反问确认 |
| ⑥ | **大模型** | 以上都不匹配 | 闲聊 |

### 8.2 关键词表（L522-530）

```python
_PLACE_HINTS = ("想","要","去","打","玩","练","参观","逛","看","找")
_GO_WORDS    = ("去","到","前往","带我去","领我去","回","走")
_YES_WORDS   = ("需要","好啊","好的","可以","行","带我去","麻烦","要")
_NO_WORDS    = ("不用","不需要","不了","算了","暂时不","先不","别")
_DONE_WORDS  = ("没事","没有了","不用了","没有别的","就这样","结束","回去")
```

匹配前统一 `_plain()` 去标点。

### 8.3 地点解析（`PlaceBook`）

- 点位名直接是中文（消防地图：乐知教育、核心力量跑酷、创想谷、厕所、跆拳道馆）
- 别名来自 `config/places.yaml`，支持“3号点”“卫生间”“核心跑酷”等口语
- `resolve()` 采用**最长别名匹配**，避免歧义
- `describe()` 生成给大模型看的地点清单
- `standby` 为待命点（乐知教育）

### 8.4 大模型兜底拆解（`_suggest_place` L395-427）

```python
prompt = (
    f"可去的地点：{self._places.describe()}。\n"
    f"访客说：{text}\n"
    "判断访客是不是想去某个地点，可能用活动间接表达（想练跆拳道 = 去跆拳道馆）。"
    "只能从上面地点里选，不要编造；没把握就留空。"
    '只输出 JSON：{"selected": "地点名或空字符串", "reply": "一句口语回复…"}'
)
```

输出示例：

```json
{"selected": "跆拳道馆", "reply": "可以带你去跆拳道馆，需要我带你过去吗？"}
```

**三重校验**：`_extract_json` 抠 JSON → `selected` 非空 → `resolve()` 必须命中真实点位。
任何一步失败都放弃，退回闲聊。**模型永远不能直接指挥底盘。**

---

## 9. 底盘运动

### 9.1 四层职责

| 层 | 代码 | 职责 |
|---|---|---|
| 业务 | `tour_chat._navigate` | 决定去哪、何时去 |
| 编排 | `nav_route.TourNavigator` | 切图、自检、AEB、下发、轮询、超时 |
| SDK | `robot_controller.KnewbotChassisNav` | Python 方法 → REST 请求 |
| 底盘 | 厂家闭源模块 | **SLAM + 规划 + 避障 + 控制** |

> 本项目**没有任何导航算法**。只能“下任务”，不能“控速度”。

### 9.2 导航前准备（`goto` L389-399）

```python
self.connect()                       # 懒加载连接底盘
self.ensure_map()                    # 切地图 + 等定位就绪（失败自动重定位，最多 3 次）
track_id = self._track_id(track)     # 取虚拟轨道 ID
self._chassis.set_aeb_control("On")  # 强制打开自动紧急制动
```

`ensure_map()` 最多等 60 秒定位收敛，**定位未就绪直接拒绝移动**。

### 9.3 两种导航模式

| 模式 | SDK 方法 | REST 端点 | 场景 |
|---|---|---|---|
| 点对点 | `move_to_poi(poi_name)` | `PUT /api/core/navigation/v1/actions/moveto_poi` | 底盘自规划最短路径 |
| 沿虚拟轨道 | `move_to_locations_poi(track_id, poi_name, uptrack_enabled)` | `PUT .../moveto_locations_poi` | 走录好的导览路线 |

配置里 `track=track_line_1003`，即**展厅固定路线**，防止机器人抄近路。

### 9.4 下发与轮询

`move_to_poi` 是**异步**的，只返回 `action_id`：

```python
while time.monotonic() < deadline:
    self._notifier.check(self._chassis)                  # 顺带查障碍
    status = self._chassis.get_action_status(action_id)  # GET /api/core/motion/v1/actions/{id}
    result = verdict(status)
    if result == "ok":   return
    if result == "fail": ...                             # 见 9.5 容差
    time.sleep(0.5)                                      # POLL_S
self._chassis.cancel_action([action_id])                 # 超时主动取消
```

### 9.5 状态判读与到点容差

`verdict()` 兼容三种字段名（`status` / `state` / `action_state`），返回 `ok / fail / running`。

**容差兜底**：底盘报失败时，若当前位姿距目标 ≤ `ARRIVE_TOL_M = 1.0` 米，按“已到达”处理。这是应对“最后 10 厘米反复调整报错”的实际问题。

### 9.6 三道安全机制

| 机制 | 实现 | 作用 |
|---|---|---|
| 健康自检 | `check_ready()` | 有 FATAL/ERROR/A类/B类故障、维护模式、读不到健康 → **拒绝移动** |
| AEB | `set_aeb_control("On")` | 物理层紧急制动兜底 |
| 障碍播报 | `ObstacleNotifier` | 识别 DTC `0x08020005` → 播报“请让一让”（15 秒冷却） |

---

## 10. 功能清单

| # | 功能 | 触发条件 | 实现位置 |
|---|---|---|---|
| 1 | 语音唤醒+识别 | 人脸唤醒后说话 | 平台 AIUI |
| 2 | 使能心跳保活 | 节点启动 | `_publish_enable` |
| 3 | 回声/重复/噪音过滤 | 每句输入 | `_enqueue` |
| 4 | 明确指令导航 | “带我去创想谷” | `_handle` ① |
| 5 | 模糊表达反问 | “创想谷” | `_handle` ② |
| 6 | 肯定/否定确认 | “好”/“不用” | `_handle` ③ |
| 7 | 间接意图推断 | “我想练跆拳道” | `_suggest_place` |
| 8 | 回待命点 | “没事了” | `_handle` ④ |
| 9 | 自由闲聊 | 其它任何话 | `_qwen.chat` |
| 10 | 开机自我介绍 | 节点启动 | `_greet` |
| 11 | 到达播报 | 导航成功 | `_arrival_line` |
| 12 | 导航前健康自检 | 每次导航 | `check_ready` |
| 13 | 自动重定位 | 定位丢失 | `ensure_map` |
| 14 | 障碍语音提示 | DTC 0x08020005 | `ObstacleNotifier` |
| 15 | 超时取消 | 180 秒未到 | `_wait_action` |
| 16 | 文本直连调试 | `/tour/chat` | `_on_chat` |
| 17 | 逐句落盘日志 | 每句输入输出 | `_file_log` |

---

## 11. 主要参数（默认值）

| 参数 | 默认 | 说明 |
|---|---|---|
| `asr_topic` | `/audio_asr/iat` | 平台识别结果 |
| `enable_asr` | `True` | 是否发使能心跳 |
| `chat_topic` | `/tour/chat` | 文本直连 |
| `tts_service` | `/intelligent_interaction/tts/play` | 播报服务 |
| `audio_id` | `audio_guide` | 音色/通道 |
| `min_chars` | `2` | 最短有效字数 |
| `dedupe_window` | `2.0` | 去重窗口（秒） |
| `qwen_base_url` | `http://120.48.75.178:7006/v1` | 模型地址 |
| `qwen_model` | `Qwen3.5-122B-A10B` | 模型名 |
| `qwen_api_key` | `EMPTY` | 鉴权 |
| `qwen_timeout` | `30.0` | 请求超时 |
| `history_turns` | `6` | 闲聊历史轮数 |
| `enable_nav` | `True` | 是否启用导航 |
| `host` / `rest_port` | `192.168.11.10` / `9090` | 底盘 REST |
| `map` | `消防` | 地图名 |
| `reloc_poi` | `乐知教育` | 重定位参考点 |
| `track` | `track_line_1003` | 虚拟轨道 |
| `nav_timeout` | `180.0` | 单次导航超时 |
| `obstacle_text` | `请让一让` | 障碍播报语 |
| `places_file` | `""` | 空则用包内 `config/places.yaml` |
| `log_file` | `""` | 空则用 `~/guide_logs/` |

---

## 12. 对外接口

### 订阅

| 话题 | 类型 | 用途 |
|---|---|---|
| `/audio_asr/iat` | `lyre_msgs/AsrIat` | 语音识别文字 |
| `/audio_asr/keyword` | `lyre_msgs/AsrKeyword` | 唤醒事件（含角度） |
| `/tour/chat` | `std_msgs/String` | 文本输入 |

### 发布

| 话题 | 类型 | 用途 |
|---|---|---|
| `/intelligent_interaction/lyre/enable` | `std_msgs/Bool` | 0.5s 使能心跳 |

### 服务调用

| 服务 | 类型 | 用途 |
|---|---|---|
| `/intelligent_interaction/tts/play` | `interaction_msgs/TtsService` | append / query |

### 外部 HTTP

| 目标 | 端点 |
|---|---|
| 大模型 | `POST http://120.48.75.178:7006/v1/chat/completions` |
| 底盘导航 | `PUT /api/core/navigation/v1/actions/moveto_poi` |
| 底盘状态 | `GET /api/core/motion/v1/actions/{action_id}` |

---

## 13. 关键设计决策

1. **为什么规则优先**：别名/关键词匹配是毫秒级、确定、可测试；模型是秒级、不确定。规则能覆盖 90% 的直白说法。
2. **为什么模型输出要二次校验**：防止模型编造不存在的地点，导致导航失败或乱跑。
3. **为什么推断出的地点要反问**：模型有误判概率，多一次确认成本极低，误导航成本很高。
4. **为什么导航期间不听指令**：边走边改目标会导致底盘任务冲突。
5. **为什么播报后留 2 秒静默**：喇叭尾音被麦克风收回会被识别成新指令。
6. **为什么所有请求走工作线程**：单线程 executor 不能阻塞。

---

## 14. 已知限制（重要）

| # | 限制 | 影响 | 建议 |
|---|---|---|---|
| 1 | 无重试、无兜底端点 | 网络抖动即失联 | 加一次重试 + 备用端点 |
| 2 | 历史从不重置 | 跨访客上下文串味 | 回待命点时 `_qwen.reset()` |
| 3 | 超时 30 秒过长 | 排队阻塞体验差 | 降到 8~10 秒 |
| 4 | `main_line`/`branches` 已解析但**未使用** | 无法“逐点巡线” | 需要接入 `_navigate` |
| 5 | 平台 ASR 仅前向拾音 | 侧后方说话无效 | 外接全向麦 + AudioMode 切换 |
| 6 | 明文 HTTP + 无鉴权 | 模型服务可被任何人调用 | 加 api-key + HTTPS |
| 7 | 单点依赖远程模型 | 服务器挂了全线失效 | 本地小模型兜底 |
| 8 | 小模型 JSON 遵循弱 | 换本地模型可能解析失败 | 加 few-shot / 强化容错 |
| 9 | `_speech_until` 赋值两次 | 冗余但无害 | 可清理 |

---

## 15. 故障排查

| 现象 | 可能原因 | 排查 |
|---|---|---|
| 说话完全没反应 | 使能心跳没发 / 未唤醒 | `ros2 topic echo /intelligent_interaction/lyre/enable` |
| 唤醒后无识别结果 | 未唤醒、ASR 关闭、离得太远 | 站正面 1~2 米、确认 `wakeup_mode` |
| 有识别但不回应 | 忙（导航/播报）中被过滤 | 看日志“忙中忽略” |
| 大模型无回应 | 远程服务不可达 | `curl -m 5 http://120.48.75.178:7006/v1/models` |
| 模型返回空 | 未关闭 thinking | 检查 `chat_template_kwargs` |
| 不动 | 健康故障 / 定位未就绪 | `get_robot_health`、`slam_status` |
| 报“定位未就绪” | 地图切换后未收敛 | 把车放到 `reloc_poi` 附近重试 |
| 走一半停下 | 前方障碍 | 听“请让一让”，移开障碍 |
| 到点报失败 | 底盘末段反复调整 | 已按 1 米容差兜底，可适当放宽 |
| 自己的话被当成指令 | 回声过滤失效 | 检查 `_echo_of` 阈值 |

---

## 16. 扩展方向

1. **本地模型兜底**：主用远程、失败切本地（`Qwen3-30B-A3B` + llama.cpp，接口同协议）
2. **接回主线巡线**：把 `main_line`/`branches` 接入 `_navigate`，实现逐点讲解
3. **多轮上下文重置**：回到待命点时清空历史
4. **流式输出**：降低首字延迟（需改 `_complete` 为 SSE 解析）
5. **全向拾音**：外接 USB 全向麦 + `AudioMode` 切蓝牙输入
6. **鉴权与加密**：模型服务加 `--api-key`，前置 Nginx + HTTPS
7. **可视化管理**：把 `places.yaml` 做成 Web 配置界面

---

*文档基于当前代码（`tour_chat.py` / `qwen.py` / `places.py` / `nav_route.py`）整理，参数与行号以实际代码为准。*
