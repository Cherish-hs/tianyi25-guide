#!/usr/bin/env python3
"""语音导览节点：语音/文本 -> 大模型理解 -> 定点导航 + TTS 播报。

订阅：
  /audio_asr/iat   lyre_msgs/AsrIat   平台语音识别结果（需先唤醒）
  /audio_asr/keyword lyre_msgs/AsrKeyword 唤醒词
  /tour/chat       std_msgs/String    文本直连，没有麦克风时测试用

注意：平台语音识别默认关闭，节点会以 0.5s 周期发布
  /intelligent_interaction/lyre/enable = true
停发就会被关回去；退出时发 false 还原。唤醒方式见
~/xos/lyre/share/lyre/config/aiui_params.yaml（当前是 face 人脸唤醒）。

播报：
  /intelligent_interaction/tts/play   interaction_msgs/TtsService（append）

说话方式：
  「带我去乐知教育」「去厕所」   -> 直接带路
  「我想练跆拳道」等间接表达     -> 大模型推断地点后询问是否带路
  「没事了」                     -> 回待命点（config/places.yaml 的 standby）
  其它任何话                     -> 大模型闲聊

用法：
  ros2 run nav_pkg tour_chat
  ros2 topic pub --once /tour/chat std_msgs/msg/String "{data: '带我去创想谷'}"
"""

from __future__ import annotations

import json
import os
import queue
import re
import threading
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from interaction_msgs.srv import TtsService
from lyre_msgs.msg import AsrIat, AsrKeyword
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String

from nav_pkg.nav_route import DEFAULT_AUDIO_ID, TourNavigator
from nav_pkg.places import PlaceBook
from nav_pkg.qwen import QwenClient

DEFAULT_SYSTEM_PROMPT = (
    "你的名字固定叫「小田」，不管谁问都不能改成别的名字。"
    "你是展厅里的导览机器人，正在接待访客。"
    "你可以像正常人一样自由聊天：闲聊、讲笑话、回答问题都可以，不要只聊展厅相关的话题。"
    "用简短、自然的中文口语回答，一般一到三句话，不要使用列表、编号或markdown。"
    "不知道的事情就老实说不知道，不要编造。"
)
DEFAULT_GREET = (
    "你好，我是导览机器人小田。"
    "直接说带我去哪里，我就能带你过去。"
)


class Speech:
    """平台 TTS 播报：append 后轮询到播完，让下一条话不会互相打断。

    节点由 executor 自旋，所以这里用异步调用 + 事件等待，
    say() 可以安全地在工作线程里阻塞调用。
    """

    def __init__(self, node, service, audio_id=DEFAULT_AUDIO_ID,
                 enabled=True):
        self._node = node
        self._log = node.get_logger()
        self._audio_id = audio_id
        self._enabled = enabled
        self._client = (node.create_client(TtsService, service)
                        if enabled else None)
        self.busy = False

    def say(self, text):
        text = (text or "").strip()
        if not text or not self._enabled:
            return False
        if not self._client.wait_for_service(timeout_sec=2.0):
            self._log.warn("TTS 服务未就绪，跳过播报")
            return False
        self._log.info(f"播报：{text}")
        self.busy = True
        try:
            response = self._call("append", text=text)
            if response is None or not response.success:
                self._log.warn(f"播报下发失败：{response}")
                return False
            self._wait_playback(text)
            return True
        finally:
            self.busy = False

    def _call(self, cmd, text=""):
        request = TtsService.Request()
        request.cmd = cmd
        request.type = "text" if text else ""
        request.text = text
        request.audio_id = self._audio_id
        request.extra = ""

        done = threading.Event()
        future = self._client.call_async(request)
        future.add_done_callback(lambda _: done.set())
        if not done.wait(5.0):
            self._log.warn(f"TTS {cmd} 调用超时")
            return None
        try:
            return future.result()
        except Exception as exc:  # noqa: BLE001 - 播报失败不打断对话
            self._log.warn(f"TTS {cmd} 调用异常：{exc}")
            return None

    def _wait_playback(self, text):
        """轮询 TTS 状态，等这句话播完再处理下一条输入。"""
        started = time.monotonic()
        est = max(1.5, len(text) * 0.22)
        limit = min(est * 2.0 + 5.0, 40.0)
        seen_playing = False
        idle_since = None
        while time.monotonic() - started < limit:
            response = self._call("query")
            playing = bool(
                response is not None
                and response.success
                and response.status == "playing"
            )
            now = time.monotonic()
            if playing:
                seen_playing = True
                idle_since = None
            elif idle_since is None:
                idle_since = now
            elif seen_playing and now - idle_since >= 0.4:
                return
            elif not seen_playing and now - started >= est + 3.0:
                return
            time.sleep(0.2)


class TourChat(Node):
    def __init__(self):
        super().__init__("tour_chat")

        self.declare_parameter("asr_topic", "/audio_asr/iat")
        self.declare_parameter("keyword_topic", "/audio_asr/keyword")
        self.declare_parameter("enable_asr", True)
        self.declare_parameter("enable_topic",
                               "/intelligent_interaction/lyre/enable")
        self.declare_parameter("chat_topic", "/tour/chat")
        self.declare_parameter("tts_service", "/intelligent_interaction/tts/play")
        self.declare_parameter("audio_id", DEFAULT_AUDIO_ID)
        self.declare_parameter("enable_voice", True)
        self.declare_parameter("enable_greet", True)
        self.declare_parameter("greet_text", DEFAULT_GREET)
        self.declare_parameter("min_chars", 2)
        self.declare_parameter("dedupe_window", 2.0)

        self.declare_parameter("qwen_base_url", "http://120.48.75.178:7006/v1")
        self.declare_parameter("qwen_model", "Qwen3.5-122B-A10B")
        self.declare_parameter("qwen_api_key", "EMPTY")
        self.declare_parameter("qwen_timeout", 30.0)
        self.declare_parameter("history_turns", 6)
        self.declare_parameter("system_prompt", DEFAULT_SYSTEM_PROMPT)

        self.declare_parameter("enable_nav", True)
        self.declare_parameter("host", "192.168.11.10")
        self.declare_parameter("rest_port", 9090)
        self.declare_parameter("map", "消防")
        self.declare_parameter("reloc_poi", "乐知教育")
        self.declare_parameter("track", "track_line_1003")
        self.declare_parameter("nav_timeout", 180.0)
        self.declare_parameter("obstacle_text", "请让一让")
        self.declare_parameter("places_file", "")
        self.declare_parameter("log_file", "")

        self._min_chars = int(self.get_parameter("min_chars").value)
        self._dedupe_window = float(self.get_parameter("dedupe_window").value)
        path = str(self.get_parameter("log_file").value or "").strip()
        if not path:
            path = os.path.join(os.path.expanduser("~"), "guide_logs",
                                time.strftime("tour_chat_%Y%m%d.log"))
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
        except OSError:
            pass
        self._log_file = path
        self._queue: queue.Queue[str | None] = queue.Queue()
        self._closing = False
        self._last_key = ""
        self._last_at = 0.0
        self._navigating = False
        self._pending = None
        self._awaiting = False
        self._last_spoken = ""
        self._speech_until = 0.0

        self._qwen = QwenClient(
            base_url=str(self.get_parameter("qwen_base_url").value),
            model=str(self.get_parameter("qwen_model").value),
            api_key=str(self.get_parameter("qwen_api_key").value),
            timeout=float(self.get_parameter("qwen_timeout").value),
            system_prompt=str(self.get_parameter("system_prompt").value),
            history_turns=int(self.get_parameter("history_turns").value),
        )
        self._speech = Speech(
            self,
            str(self.get_parameter("tts_service").value),
            audio_id=str(self.get_parameter("audio_id").value),
            enabled=bool(self.get_parameter("enable_voice").value),
        )

        self._nav = None
        self._places = None
        if bool(self.get_parameter("enable_nav").value):
            self._setup_navigation()

        self.create_subscription(
            AsrIat, str(self.get_parameter("asr_topic").value),
            self._on_asr, 10)
        self.create_subscription(
            AsrKeyword, str(self.get_parameter("keyword_topic").value),
            self._on_keyword, 10)
        self.create_subscription(
            String, str(self.get_parameter("chat_topic").value),
            self._on_chat, 10)

        # 平台 ASR 默认是关的（asr open status:0），必须持续使能，
        # 一旦停发就会被关回去，表现为「说话没有任何反应」。
        self._enable_pub = None
        if bool(self.get_parameter("enable_asr").value):
            self._enable_pub = self.create_publisher(
                Bool, str(self.get_parameter("enable_topic").value), 10)
            self.create_timer(0.5, self._publish_enable)

        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

        greet = ""
        if bool(self.get_parameter("enable_greet").value):
            try:
                greet = str(self.get_parameter("greet_text").value or "").strip()
            except Exception:  # noqa: BLE001 - 空字符串覆盖时参数未初始化
                greet = ""
        if greet:
            threading.Thread(target=self._greet, args=(greet,),
                             daemon=True).start()

        self.get_logger().info(
            f"导览对话已就绪：导航={'开' if self._nav else '关'}，"
            f"模型={self.get_parameter('qwen_model').value}")

    # ------------------------------------------------------------- 初始化
    def _setup_navigation(self):
        try:
            path = str(self.get_parameter("places_file").value).strip()
            if not path:
                path = os.path.join(
                    get_package_share_directory("nav_pkg"),
                    "config", "places.yaml")
            self._nav = TourNavigator(
                self,
                host=str(self.get_parameter("host").value),
                rest_port=int(self.get_parameter("rest_port").value),
                map_name=str(self.get_parameter("map").value),
                reloc_poi=str(self.get_parameter("reloc_poi").value),
                track=str(self.get_parameter("track").value),
                timeout=float(self.get_parameter("nav_timeout").value),
                speaker=self._speech,
                obstacle_text=str(self.get_parameter("obstacle_text").value),
            )
            self._nav.connect()
            self._places = PlaceBook.load(
                path, self._nav.poi_names(), self.get_logger())
            self.get_logger().info(f"地点配置：{path}")
            self.get_logger().info(f"可去的地点：{self._places.describe()}")
        except Exception as exc:  # noqa: BLE001 - 导航不可用时只保留聊天
            self._nav = None
            self._places = None
            self.get_logger().error(f"导航初始化失败（只保留聊天）：{exc}")

    # ------------------------------------------------------------- 订阅回调
    def _publish_enable(self) -> None:
        """持续打开平台语音识别，停发即自动关闭。"""
        msg = Bool()
        msg.data = True
        self._enable_pub.publish(msg)

    def _on_keyword(self, msg: AsrKeyword) -> None:
        text = str(msg.keyword or "").strip()
        if text:
            self.get_logger().info(
                f"唤醒词：{text}（声源角度 {msg.angle}°），请继续说")

    def _on_asr(self, msg: AsrIat) -> None:
        self._enqueue(msg.text or "", "听到")

    def _on_chat(self, msg: String) -> None:
        self._enqueue(msg.data or "", "文本输入")

    def _enqueue(self, text: str, source: str) -> None:
        text = (text or "").strip()
        if not _meaningful(text, self._min_chars):
            return
        # 刚问完「现在还有什么事吗」时访客会马上回答：
        # 只在播报结束后留 0.5 秒防尾音，不能像平时那样静默 2 秒，
        # 否则「没有」这句话会被直接丢掉。
        quiet_until = (self._speech_until - 1.5 if self._awaiting
                       else self._speech_until)
        quiet = time.monotonic() < quiet_until
        if self._navigating or self._speech.busy or quiet:
            self.get_logger().info(f"忙（导航/播报）中忽略：{text}")
            self._file_log("忽略(忙)", text)
            return
        # 喇叭声音被自己麦克风收回来时，不能当成用户指令
        if _echo_of(text, self._last_spoken):
            self.get_logger().info(f"忽略自己的播报回声：{text}")
            self._file_log("忽略(回声)", text)
            return
        key = re.sub(r"\s+", "", text)
        now = time.monotonic()
        if key == self._last_key and now - self._last_at < self._dedupe_window:
            return
        self._last_key, self._last_at = key, now
        self.get_logger().info(f"{source}：{text}")
        self._file_log(source, text)
        self._queue.put(text)

    # --------------------------------------------------------------- 工作线程
    def _worker_loop(self) -> None:
        while not self._closing:
            try:
                text = self._queue.get(timeout=0.2)
            except queue.Empty:
                continue
            if text is None:
                return
            try:
                self._handle(text)
            except Exception as exc:  # noqa: BLE001 - 单次失败不影响后续对话
                self.get_logger().error(f"处理失败：{exc}")

    def _handle(self, text: str) -> None:
        if self._nav is not None and not self._navigating:
            place = self._places.resolve(text)

            # 1) 直接说了地点：带「去/到」就出发，否则先问一句
            if place is not None:
                # 只认短指令，长串闲聊/回声不触发导航
                if _explicit_go(text) and len(_plain(text)) <= 20:
                    self._pending = None
                    self._navigate(place)
                    return
                self._pending = place
                self._speak(f"可以去{place.spoken}，需要我带你去吗？")
                return

            # 2) 上一句提议了地点，这一句是同意还是拒绝
            if self._pending is not None:
                pending, self._pending = self._pending, None
                if _means_no(text):
                    self._speak("好的，想去的时候随时叫我")
                    return
                if _means_yes(text):
                    self._navigate(pending)
                    return

            # 3) 到地方后问「还有什么事」，说没事就回待命点
            if self._awaiting and _wants_done(text):
                standby = self._places.standby
                if standby is None:
                    self._awaiting = False
                    self._speak("好的，那我就在这儿等你")
                    return
                self._navigate(standby, standby=True)
                return

            # 4) 没直接说地点（如「我想练跆拳道」），让大模型推断
            suggestion = self._suggest_place(text)
            if suggestion is not None:
                place, reply = suggestion
                self._pending = place
                self._speak(reply)
                return

        # 5) 都命不中就是闲聊
        try:
            reply = self._qwen.chat(text)
        except Exception as exc:  # noqa: BLE001 - 大模型不可用时给一句兜底
            self.get_logger().error(f"大模型请求失败：{exc}")
            self._speak("我这边暂时联系不上大模型，请稍后再试")
            return
        self.get_logger().info(f"小郑：{reply}")
        self._speak(reply)

    def _suggest_place(self, text: str):
        """用大模型从「我想练跆拳道」这类话里推断地点，返回 (Place, 回复)。"""
        if not any(word in text for word in _PLACE_HINTS):
            return None
        prompt = (
            f"可去的地点：{self._places.describe()}。\n"
            f"访客说：{text}\n"
            "判断访客是不是想去某个地点，可能用活动间接表达（想练跆拳道 = 去跆拳道馆）。"
            "只能从上面地点里选，不要编造；没把握就留空。"
            '只输出 JSON，不要其他文字：'
            '{"selected": "地点名或空字符串", "reply": "一句口语回复，'
            '选到地点时告诉对方可以去哪里并问要不要带路"}'
        )
        try:
            raw = self._qwen.one_shot(
                prompt, system="你是意图解析器，只输出 JSON，不解释。")
        except Exception as exc:  # noqa: BLE001 - 解析失败就当普通聊天
            self.get_logger().warn(f"地点意图识别失败：{exc}")
            return None

        data = _extract_json(raw)
        if not isinstance(data, dict):
            return None
        selected = str(data.get("selected") or "").strip()
        if not selected:
            return None
        place = self._places.resolve(selected)
        if place is None:
            return None
        reply = str(data.get("reply") or "").strip()
        self.get_logger().info(f"推断地点：{selected} -> {place.poi}")
        return place, (reply or f"可以去{place.spoken}，需要我带你去吗？")

    # ------------------------------------------------------------------ 导航
    def _navigate(self, place, standby: bool = False) -> None:
        self._navigating = True
        self._drain_queue()
        try:
            ok, message = self._nav.check_ready()
            if not ok:
                self._speak(message)
                return
            target = place.spoken
            if not standby:
                self._speak(f"好的，我这就带你去{target}")
            self.get_logger().info(
                f"开始导航：{place.poi}（{target}，"
                f"{'回待命点' if standby else '送访客'}）")
            self._file_log("导航", f"{place.poi} {'回待命点' if standby else '送访客'}")
            self._nav.goto(place.poi)
        except Exception as exc:  # noqa: BLE001 - 导航失败只提示，不退出节点
            self.get_logger().error(f"导航失败：{exc}")
            self._speak("抱歉，这次没能带你去，底盘导航没有完成")
            return
        finally:
            self._navigating = False

        if standby:
            self._awaiting = False
            self._speak(f"好的，我已经回到{place.spoken}待命，有事随时叫我")
        else:
            self._awaiting = True
            self._speak(self._arrival_line(place.spoken))

    def _arrival_line(self, target: str) -> str:
        try:
            reply = self._qwen.one_shot(
                f"你刚把访客带到了{target}。"
                "用一句简短自然的口语告诉对方已经到了，并问现在还有什么事。不要列表。")
        except Exception as exc:  # noqa: BLE001 - 大模型不可用就用固定话术
            self.get_logger().warn(f"生成到达播报失败：{exc}")
            return f"我们到{target}了，现在还有什么事吗？"
        return reply or f"我们到{target}了，现在还有什么事吗？"

    def _drain_queue(self) -> None:
        while True:
            try:
                self._queue.get_nowait()
            except queue.Empty:
                return

    def _speak(self, text: str) -> None:
        text = (text or "").strip()
        if not text:
            return
        self._last_spoken = text
        self._file_log("播报", text)
        self._speech_until = time.monotonic() + 2.0
        self._speech.say(text)
        # 播完再留 2 秒静默，等喇叭尾音和识别结果过去
        self._speech_until = time.monotonic() + 2.0

    def _file_log(self, kind: str, text: str) -> None:
        """把每句话落到日志文件，方便事后排查（终端滚走了也能查）。"""
        try:
            with open(self._log_file, "a", encoding="utf-8") as handle:
                handle.write(f"{time.strftime('%H:%M:%S')} {kind} {text}\n")
        except OSError:
            pass

    def _greet(self, text: str) -> None:
        """开机自我介绍：直接播报，绝不走 _handle（自己的话不能当指令）。"""
        time.sleep(2.0)
        for _ in range(3):
            if self._speech.say(text):
                self._last_spoken = text
                self._file_log("播报", text)
                self._speech_until = time.monotonic() + 2.0
                return
            time.sleep(2.0)

    def shutdown(self) -> None:
        self._closing = True
        self._queue.put(None)
        self._worker.join(timeout=2.0)
        # 还原本地现场：关闭语音识别
        if self._enable_pub is not None and rclpy.ok():
            msg = Bool()
            msg.data = False
            self._enable_pub.publish(msg)


def _meaningful(text: str, min_chars: int) -> bool:
    """过滤空串、纯标点、过短的误识别。"""
    return len(re.sub(r"\W", "", text or "")) >= max(1, min_chars)


_PLACE_HINTS = ("想", "要", "去", "打", "玩", "练", "参观", "逛", "看", "找")
_GO_WORDS = ("去", "到", "前往", "带我去", "领我去", "回", "走")
_YES_WORDS = ("需要", "好啊", "好的", "好呀", "可以", "行", "带我去", "去吧",
              "麻烦", "要")
_NO_WORDS = ("不用", "不需要", "不了", "算了", "暂时不", "先不", "别")
_DONE_WORDS = ("没事", "没有", "没什么", "没啥", "没别的事", "不用了",
               "就这样", "结束", "回去")


def _plain(text: str) -> str:
    return re.sub(r"[\s，。！？、,.!?~～]", "", text or "")


def _means_yes(text: str) -> bool:
    plain = _plain(text)
    return any(word in plain for word in _YES_WORDS)


def _explicit_go(text: str) -> bool:
    plain = _plain(text)
    return any(word in plain for word in _GO_WORDS)


def _means_no(text: str) -> bool:
    plain = _plain(text)
    return any(word in plain for word in _NO_WORDS)


def _wants_done(text: str) -> bool:
    plain = _plain(text)
    return any(word in plain for word in _DONE_WORDS)


def _echo_of(text: str, spoken: str) -> bool:
    """识别文本是不是自己刚播报内容的回声（长句才判，避免误伤短指令）。"""
    said = _plain(text)
    last = _plain(spoken)
    if len(said) < 8 or len(last) < 8:
        return False
    shorter, longer = (said, last) if len(said) <= len(last) else (last, said)
    return shorter in longer and len(shorter) / len(longer) >= 0.5


def _extract_json(text: str):
    match = re.search(r"\{.*\}", text or "", re.S)
    if not match:
        return None
    try:
        return json.loads(match.group(0))
    except json.JSONDecodeError:
        return None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TourChat()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.shutdown()
            executor.shutdown()
            node.destroy_node()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
