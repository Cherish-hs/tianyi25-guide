# 小田 · 导览工作台（learn2_Z / web）

- 目录：`/home/nvidia/guide_robot/learn/learn2_Z/web`
- 监测进程与语音导览、导航、相机、底盘进程完全独立，不修改它们的代码、配置或启动项
- 只读取 ROS 图、已有话题和底盘只读 REST 接口；控制类操作仅限 4 个白名单动作，且请求必须带确认标记

---

## 1. 启动与访问

```bash
cd /home/nvidia/guide_robot/learn/learn2_Z/web
bash start_monitor.sh --port 8766
```

- 默认端口 **8766**（避开 `guide_robot/web_monitor` 的 8765）
- 默认监听 `0.0.0.0`，局域网内可直接访问 `http://<机器人IP>:8766/`
- 启动脚本会依次 source：`/opt/ros/jazzy/setup.bash`、`/home/nvidia/xos/setup.bash`、`learn2_Z/install/setup.bash`
- 可选参数：`--port 9000`、`--bind 127.0.0.1`

停止监测服务：终端 `Ctrl-C`，或向监测进程发 `SIGTERM`。监测服务退出不会自动停止已启动的 `tour_chat` 子进程；需要停止节点时使用停止接口或在终端单独处理。

当前启动脚本、地点配置路径和底盘 REST 地址按原机器人部署写定；在其他用户名或目录运行前，需要按实际环境调整 `start_monitor.sh`，以及 `monitor.py` 中的 `PLACES_FILE`、`PLACES_FALLBACK` 和底盘 URL。也可在加载正确的 ROS、xos、工作空间环境后，用 `/usr/bin/python3 monitor.py --port 8766` 启动后端。

### 运行依赖

| 依赖                         | 用途                           |
| ---------------------------- | ------------------------------ |
| Python 3.12                  | 后端 `monitor.py`              |
| ROS 2 Jazzy（rclpy、rosidl） | ROS 图、话题订阅、指令发布     |
| `/home/nvidia/xos/setup.bash`           | 底盘/麦克风/TTS 等平台消息类型 |
| psutil                       | CPU / 内存 / 温度 / 开机时长   |
| PyYAML                       | 读取 `places.yaml`             |
| OpenCV + NumPy               | 相机图像解码、深度图伪彩       |
| zoneinfo                     | 时间按 `Asia/Shanghai` 显示    |

> 没有 Node.js 或前端构建步骤，`index.html` + `live.js` 是静态文件，但实时监控和控制必须由 `monitor.py` 提供服务。

### 语音模式与启动前提

工作台本身不做语音识别。点击“启动节点”时，后端执行 `ros2 run nav_pkg tour_chat`，不附加语音模式参数，因此沿用源码默认值：**本地 faster-whisper 识别**（`enable_mic=true`、`enable_asr=false`），音频采集和 TTS 仍由平台提供，大模型默认使用远程接口。

需要先完成主 [README](../README.md) 的依赖准备和构建。若使用平台 AIUI，应在已加载环境的终端启动：

```bash
ros2 run nav_pkg tour_chat --ros-args \
  -p enable_mic:=false -p enable_asr:=true
```

页面会识别已运行的 `tour_chat`；当前页面没有切换识别模式的选项。“麦克风有音频数据”仅表示收到音频帧，不能证明 Whisper 已加载成功或平台 ASR 已唤醒。

---

## 2. 页面说明

左侧导航共 4 页，右上角始终显示连接状态（`实时监测 · 已连接` / `监测连接中断 · 正在重连`），侧栏固定显示电量、CPU、温度、内存、运行时长。

### 2.1 总览

| 区块         | 内容                                                                                  |
| ------------ | ------------------------------------------------------------------------------------- |
| 摄像头画面   | 头部相机彩色/深度实时画面，可下拉切换；左上角显示分辨率（如 `彩色画面 · 1280 × 720`） |
| 当前任务     | 待命时显示 3 步引导：启动节点 → 选择目的地 → 下发导航；导航中显示目标、路线步骤与进度 |
| 地图         | 主线示意图缩略图 + 机器人位置，点击查看全图                                           |
| 启动节点     | `启动节点` 按钮；语音链路 / 语音播报 / 麦克风三项状态                                 |
| 快捷指令下发 | 「导航 →」从 `places.yaml` 站点下拉选择后下发；「指令 →」输入 ≤200 字任意文本         |
| 事件流       | 实时事件列表（INFO/WARN/ERROR），支持全部/告警/错误筛选、暂停、导出                   |

### 2.2 设备状态

- **主机与底盘卡片**：CPU 负载、CPU 温度、主机内存、电池电量、开机时长，带趋势曲线；电池卡片同时显示「充电中 / 未充电 / 未知」
- **ROS 与硬件卡片**：ROS 图（节点数/话题数）、头部彩色相机（帧间隔）、深度图与前方距离、麦克风（最近 4 秒是否有音频帧）、移动底盘（`knewbots_chassis_control`）、语音播报（`/intelligent_interaction/tts/play`）、导览语音节点（`tour_chat` 是否运行）
- **趋势图**：CPU 负载、资源负载、节点状态
- **服务状态**：语音导览节点 / 语音播报服务 / 麦克风采集 / 移动底盘 四个状态胶囊

### 2.3 节点状态

- 列出 ROS 图中的全部节点：状态（运行中/已退出）、节点名、命名空间、自本监测服务首次发现以来的时长、最近在线时间
- 顶部统计：在线数 / 最近退出数 / 命名空间数，并标注刷新与保留策略（约 5 秒刷新，超过 15 分钟未见的节点会移除）
- 支持按节点名或命名空间即时过滤

### 2.4 地图配置

- 上半部分：运行示意图 + 实时位姿读数
- 下半部分：导览目的地清单，展示 `places.yaml` 中的中文名、地图点位名、主线顺序
- 该页只读，不能改点位

---

## 3. 数据来源与刷新频率

| 数据                         | 来源                                                                                         | 刷新                                 |
| ---------------------------- | -------------------------------------------------------------------------------------------- | ------------------------------------ |
| CPU / 内存 / 温度 / 开机时长 | 主机 `/proc`、`/sys/class/thermal`（psutil）                                                 | 2 秒                                 |
| ROS 节点 / 话题 / 服务       | ROS 图 `get_node_names_and_namespaces` 等                                                    | 5 秒                                 |
| 彩色画面                     | `/ob_camera_head/color/image_raw/compressed`，不存在时退回 `/ob_camera_head/color/image_raw` | 约 3 帧/秒，最宽 768px，JPEG 质量 65 |
| 深度画面                     | `/ob_camera_head/depth/image_raw`                                                            | 约 2 帧/秒，转 TURBO 伪彩            |
| 前方距离                     | 深度图中心区域（高 25%~60%、宽 35%~65%）                                                     | 随深度帧，取 5% 分位值               |
| 相机设备状态                 | `/ob_camera_head/device_status`                                                              | 随消息                               |
| 电池（底盘 REST）            | `GET http://192.168.11.10:9090/api/core/system/v1/battery/pack`                              | 5 秒                                 |
| 电池（话题）                 | `/power/battery/status`、`/power/board/status`                                               | 随消息                               |
| 实时位姿                     | `GET http://192.168.11.10:9090/api/core/navigation/v1/pose`                                  | 1 秒                                 |
| 点位坐标                     | `GET http://192.168.11.10:9090/api/core/navigation/v1/poi`                                   | 10 秒（有变化）/ 30 秒               |
| 麦克风活跃度                 | `/lyre/audio_stream` 消息到达时间（不保存音频）                                              | 随消息，判定窗口 4 秒                |
| 当前任务                     | `/rosout` 中 `tour_chat` / `nav_route` 的日志                                                | 随消息                               |
| 标定元数据（非当前 SVG 布局来源） | 本地 `map_calib.json` + `map.png`                                                            | 3 秒                                 |

前端每轮请求结束后约 **2 秒** 再轮询 `/api/status` 与 `/api/logs`；相机帧每次请求结束后约 350ms 再拉取，超过 3 秒的旧帧不再返回。

### 3.1 当前任务的解析规则

只解析来源节点名包含 `tour_chat` 或 `nav_route` 的日志：

| 日志                                                                   | 行为                                          |
| ---------------------------------------------------------------------- | --------------------------------------------- |
| `开始导航：<点位>（...）`                                              | 任务进入 running，目标为该点位                |
| `计划：A -> B -> C`                                                    | 任务进入 running，路线为 A/B/C，目标为 C      |
| `到达 <点位>，当前位置 ...`                                            | 点位等于当前目标时任务 done，否则保持 running |
| `导航失败：...`、`被障碍物挡住`、`放弃绕行`、`没有可用的头部深度图` 等 | 任务 failed                                   |

任务状态只存在内存中，不额外发布状态话题。

---

## 4. 控制能力与边界

下列四个 POST 接口的请求体必须带 `{"confirm": true}`，否则返回 400。页面的节点启停、导航及“发送”按钮会弹出二次确认；指令输入框按 Enter 则直接下发，不弹确认框；这个布尔标记不是身份认证，后端当前没有登录或令牌校验。

| 接口                   | 行为                                                                                         |
| ---------------------- | -------------------------------------------------------------------------------------------- |
| `POST /api/node/start` | 在后台执行 `ros2 run nav_pkg tour_chat`，输出写入 `/tmp/learn2z-tour-chat.log`；默认启动时问候并等待输入；后续识别或文本指令可触发移动 |
| `POST /api/node/stop`  | 只停止由本页启动的进程（先 SIGINT，约 8 秒未退出再 SIGTERM）；终端里手动启动的节点不会被动     |
| `POST /api/navigate`   | 校验地点在 `places.yaml` 白名单内，向 `/tour/chat` 发布 `去<地点>`；节点未运行时返回 409     |
| `POST /api/command`    | 把 ≤200 字的任意文本原样发布到 `/tour/chat`；节点未运行时返回 409                            |

其他约束：

- `PUT` / `PATCH` / `DELETE` 一律 405；未知路径 404
- 不提供任意文件读取、目录浏览或用户自定义 shell 命令；启动接口只执行固定节点命令
- 页面不会直接调用底盘 SDK，导航统一走 `tour_chat` 的既有链路

---

## 5. HTTP 接口一览

| 方法 | 路径                                                                              | 说明                                                                                   |
| ---- | --------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------- |
| GET  | `/api/status`                                                                     | 全量快照：主机、ROS 图、传感器样本、相机状态、任务、地点、点位、节点运行状态、地图标定 |
| GET  | `/api/logs`                                                                       | 最近 200 条事件（`{logs: [...]}`）                                                     |
| GET  | `/api/logs/export`                                                                | 导出纯文本事件流，带下载文件名 `learn2z-monitor-YYYYmmdd-HHMMSS.log`                   |
| GET  | `/api/frame?kind=color\|depth`                                                    | 最新 JPEG 帧；超过 3 秒返回 503                                                        |
| GET  | `/`、`/index.html`、`/live.js`、`/map.png`、`/schematic_base.png`、`/favicon.ico` | 静态资源                                                                               |
| POST | `/api/node/start`、`/api/node/stop`、`/api/navigate`、`/api/command`              | 见上一节                                                                               |

响应头统一带 `Cache-Control: no-store`、`X-Content-Type-Options: nosniff` 和 CSP（仅允许同源脚本/样式，`connect-src 'self'`）。

`/api/status` 中：

- `samples` 里每项带 `at` 与 `age_s`，前端据此判断新鲜度
- `cameras` 只含元数据（时间、尺寸），不包含图像字节
- `read_only` 当前固定返回 `true`，仅是现有字段；服务仍提供上述四个 POST 控制接口，不能据此判定服务完全只读
- `node.running` = ROS 图中发现 `/tour_chat` 或本页启动的进程仍在运行；`node.managed` = 是否由本页启动

---

## 6. 配置文件

### 6.1 地点清单 `places.yaml`

后端启动时按以下顺序读取（当前代码使用固定绝对路径）：

1. `/home/nvidia/guide_robot/learn/learn2_Z/install/nav_pkg/share/nav_pkg/config/places.yaml`
2. `/home/nvidia/guide_robot/learn/learn2_Z/src/nav_pkg/config/places.yaml`

```yaml
standby: 乐知教育          # 待命点（页面只展示）
aliases:                   # 别名 -> 地图点位名，同一站点的首个中文名用于展示
  乐知教育: 乐知教育
  乐知院: 乐知教育
  核心力量跑酷: 核心力量跑酷
transfer_points:           # 仅作中转，不出现在可去清单
  - start_point
main_line:                 # 主线顺序，决定示意图站点次序
  - 乐知教育
  - 核心力量跑酷
  - 创想谷
  - 厕所
  - 跆拳道馆
```

地点清单只在后端启动时读取，修改后需要重启监测服务，刷新页面不会重新读取 YAML。安装副本优先于源码配置：修改源码配置后需重新构建，或同步修改实际读取的安装副本。`tour_chat` 自己也在初始化时读取配置，运行中的节点需重启才会采用新配置。

`main_line` 在工作台中用于排列展示顺序，不会让 `tour_chat` 自动逐点巡线。当前页面部分提示写有“逐点经过”，实际导航行为仍以节点参数和源码为准。工作台不会跟随 `tour_chat` 的自定义 `places_file` 参数自动切换配置。

### 6.2 地图标定 `map_calib.json`

```json
{"width": 948, "height": 650, "scale": 15.1958, "origin_x": 124.92, "origin_y": 362.57}
```

后端按约 3 秒间隔读取标定，并用同目录的 `map.png` 是否存在设置 `map.available`。该标定约定的换算是 `px = origin_x + x * scale`、`py = origin_y - y * scale`。

当前页面的主地图使用 `live.js` 动态生成的 SVG 示意图，并未按上述标定渲染 `map.png`；更改 `map_calib.json` 不会改变当前 SVG 的几何布局。

### 6.3 示意图素材

- `schematic.json`：地图房间/障碍块几何
- `schematic_cells.json`：占据栅格采样
- `schematic_base.png`：示意图底图

这三份素材保留在目录中，当前 `live.js` 并未读取它们来生成地图。实际示意图由 `PLAN`、`PLAN_BRANCHES`、`CORRIDOR` 等常量及底盘 POI 坐标生成；机器人位置按走廊 X 方向投影，未读到 POI 坐标时使用展示用的回退位置，不能当作精确导航地图。

换场地时需调整 `live.js` 的示意图布局、底盘接口地址及地点配置，不能仅替换这些素材或 `map_calib.json`。

---

## 7. 事件流与日志

- 后端事件流保留最近 **200 条**，页面按过滤条件最多展示其中最近 60 条，来源包括 `monitor.host`、`monitor.ros`、`monitor.pose`、`monitor.battery`、`monitor.camera`、`monitor.map`、`monitor.control`
- 日志中匹配 `api_key`、`access_token`、`access_key`、`secret`、`password`、`Authorization` 赋值形式及 `Bearer` 凭据的内容会被遮盖为 `[redacted]`（不保证识别所有敏感信息），单条截断到 600 字
- `导出日志` 生成带时间戳的一行一条纯文本，可直接存档
- 仅由工作台启动的 `tour_chat` 标准输出/错误追加到 `/tmp/learn2z-tour-chat.log`；手动启动节点的输出仍在对应终端。节点自身默认日志为 `~/guide_logs/tour_chat_YYYYMMDD.log`，可用 `log_file` 覆盖
- 导航 INFO 日志用于推断任务状态；ROS 事件流主要收集指定节点的 WARN/ERROR 并限流，并不展示所有对话与导航 INFO 日志

---

## 8. 常见问题

| 现象                                                                   | 原因与处理                                                                                                                 |
| ---------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------- |
| 页面一直「正在连接监测服务」                                           | 服务未启动或端口不对；确认 `ss -ltn \| grep 8766`                                                                          |
| ROS 显示「未连接」                                                     | 启动脚本未 source 到 ROS/xos；先手动 `source /opt/ros/jazzy/setup.bash` 试 `ros2 node list`                                |
| 启动后报 `ImportError: librcl_action.so` 或 `GLIBCXX_3.4.29 not found` | 当前 shell 的 `LD_LIBRARY_PATH` 被其他工具污染；用干净环境启动：`env -u LD_LIBRARY_PATH bash start_monitor.sh --port 8766` |
| 相机一直「等待相机画面」                                               | 头部相机节点未启动，或话题名不是 `/ob_camera_head/...`；可在节点状态页搜索 `ob_camera`                                     |
| 语音链路显示「未运行」但机器人能对话                                   | 节点不是 `tour_chat` 名字，或不是本页启动；页面只认 `/tour_chat`                                                           |
| 导航下发返回 409                                                       | `tour_chat` 未运行，先在「启动节点」里启动，或在终端 `ros2 run nav_pkg tour_chat`                                          |
| 快捷导航列表为空                                                       | 没读到 `places.yaml`；检查第 6.1 节两个路径                                                                                |
| 端口冲突                                                               | 与 `guide_robot/web_monitor`（8765）同时运行时，本服务用 8766                                                              |
| 地图页没有机器人位置                                                   | 底盘 REST 不可达，或 `map_calib.json` / `map.png` 缺失                                                                     |

---

## 9. 校验

```bash
cd /home/nvidia/guide_robot/learn/learn2_Z/web
python3 -m unittest test_monitor -v
```

覆盖内容：确认标记校验、地点白名单、节点未运行时拒绝导航、未知 POST 路径 404、写方法 405、私有文件拒绝、旧图像 503、快照不含图像字节、NaN/Inf 处理、凭据遮盖、导航日志任务跟踪、后端不调用底盘 SDK。

测试不会启动 ROS 或机器人服务。
