"""RemoteCAN — Tier 2: CAN 隧道客户端（直接使用厂商 SDK）。

将控制器的 can0（灵巧手部分）映射到本地的 vcan0，
让用户可以直接使用厂商 SDK 操作灵巧手。

安全设计：
- 只转发 server 白名单内的 CAN ID（由 server 端强制过滤）
- 客户端也做 ID 过滤（双重保险）
- 无法通过隧道控制机械臂（ID 被 server 屏蔽）

架构：
    客户端 vcan0 (虚拟)
        └── 灵巧手 SDK 直接使用
                ↕ zenoh 隧道
    控制器 can0 (物理)
        └── 灵巧手（白名单 ID）

Usage::

    from litearm.can_bridge import RemoteCAN

    can = RemoteCAN(endpoint="tcp/192.168.1.100:7447")
    can.start()  # 创建 vcan0，启动隧道

    # 现在可以用厂商 SDK
    from lingxin_hand import Hand
    hand = Hand(can_interface="vcan0")
    hand.open()
    hand.set_gesture("pinch")

    # 清理
    can.stop()

注意：
- 需要 sudo 权限创建 vcan 接口
- 厂商 SDK 需要用户自行安装
"""
from __future__ import annotations

import logging
import socket
import struct
import subprocess
import threading
from typing import Optional, Set

log = logging.getLogger("litearm.can_bridge")

# CAN 帧常量
CAN_FRAME_FMT = "=IB3x8s"
CAN_FRAME_SIZE = 16
CAN_RAW = 1


def _extract_can_id(frame: bytes) -> int:
    """从 CAN 帧提取 CAN ID。"""
    raw_id = struct.unpack("=I", frame[:4])[0]
    return raw_id & 0x1FFFFFFF


def _format_can_id(can_id: int) -> str:
    return f"0x{can_id:03X}"


class RemoteCAN:
    """CAN 隧道客户端（Tier 2: 直接使用厂商 SDK）。

    创建虚拟 CAN 接口（vcan0），通过 zenoh 桥接到控制器的 can0。
    灵巧手 SDK 可以直接使用 vcan0，就像在控制器本地运行一样。

    Usage::

        can = RemoteCAN(endpoint="tcp/192.168.1.100:7447")
        can.start()

        from lingxin_hand import Hand
        hand = Hand(can_interface="vcan0")
        hand.open()

        can.stop()
    """

    # Zenoh topic 命名（与 server 端对应）
    TOPIC_TO_CLIENT = "litearm/v4/can/to_client"
    TOPIC_FROM_CLIENT = "litearm/v4/can/from_client"

    def __init__(
        self,
        endpoint: str = "tcp/127.0.0.1:7447",
        vcan_iface: str = "vcan0",
        hand_can_ids: Optional[Set[int]] = None,
    ) -> None:
        """初始化 CAN 隧道客户端。

        Args:
            endpoint: litearm-server 的 zenoh 端点。
            vcan_iface: 本地虚拟 CAN 接口名称（默认 vcan0）。
            hand_can_ids: 灵巧手 CAN ID 白名单（可选，客户端侧过滤）。
                         如果不指定，依赖 server 端过滤。
                         建议指定以增加安全性。
        """
        self._endpoint = endpoint
        self._vcan_iface = vcan_iface
        self._hand_can_ids = hand_can_ids  # 客户端侧可选过滤
        self._session = None
        self._sock: Optional[socket.socket] = None
        self._running = False
        self._threads: list[threading.Thread] = []

        # 统计
        self._stats = {
            "to_server": 0,    # 发送给 server 的帧数
            "from_server": 0,  # 接收 server 的帧数
            "blocked": 0,      # 客户端过滤的帧数
            "errors": 0,
        }

    @property
    def stats(self) -> dict:
        return dict(self._stats)

    @property
    def interface(self) -> str:
        """返回虚拟 CAN 接口名称。"""
        return self._vcan_iface

    def setup_vcan(self) -> None:
        """创建虚拟 CAN 接口（需要 sudo 权限）。"""
        # 检查接口是否已存在
        result = subprocess.run(
            ["ip", "link", "show", self._vcan_iface],
            capture_output=True,
        )
        if result.returncode == 0:
            log.info("vcan interface %s already exists", self._vcan_iface)
            # 确保接口 up
            subprocess.run(
                ["sudo", "ip", "link", "set", self._vcan_iface, "up"],
                check=True,
            )
            return

        # 创建 vcan 接口
        log.info("Creating vcan interface: %s", self._vcan_iface)
        subprocess.run(
            ["sudo", "ip", "link", "add", self._vcan_iface, "type", "vcan"],
            check=True,
        )
        subprocess.run(
            ["sudo", "ip", "link", "set", self._vcan_iface, "up"],
            check=True,
        )
        log.info("vcan interface %s created and up", self._vcan_iface)

    def start(self) -> None:
        """启动 CAN 隧道。

        1. 创建 vcan 接口（需要 sudo）
        2. 连接 zenoh
        3. 启动双向桥接线程
        """
        import zenoh

        # 1. 创建 vcan 接口
        self.setup_vcan()

        # 2. 创建 zenoh session
        cfg = zenoh.Config()
        cfg.insert_json5("mode", '"peer"')
        cfg.insert_json5("scouting/multicast/enabled", "false")
        cfg.insert_json5("scouting/gossip/enabled", "false")
        cfg.insert_json5("connect/endpoints", f'["{self._endpoint}"]')
        self._session = zenoh.open(cfg)

        # 3. 创建 CAN socket
        self._sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, CAN_RAW)
        self._sock.bind((self._vcan_iface,))
        self._sock.settimeout(0.1)

        self._running = True

        # 线程 1: vcan0 → zenoh → server
        t1 = threading.Thread(
            target=self._vcan_to_zenoh, daemon=True, name="remote_can_tx"
        )
        t1.start()
        self._threads.append(t1)

        # 订阅 server 发来的帧
        def on_recv(sample):
            self._on_frame_from_server(sample)
        self._session.declare_subscriber(self.TOPIC_TO_CLIENT, on_recv)

        # 线程 2: 保持主线程可检测 _running
        t2 = threading.Thread(
            target=self._keepalive, daemon=True, name="remote_can_keepalive"
        )
        t2.start()
        self._threads.append(t2)

        log.info(
            "RemoteCAN started: %s ↔ %s",
            self._vcan_iface, self._endpoint,
        )
        if self._hand_can_ids:
            log.info(
                "Client-side filter enabled: [%s]",
                ", ".join(_format_can_id(i) for i in sorted(self._hand_can_ids)),
            )

    def stop(self) -> None:
        """停止 CAN 隧道。"""
        log.info("Stopping RemoteCAN...")
        self._running = False

        for t in self._threads:
            t.join(timeout=2.0)
        self._threads.clear()

        if self._sock is not None:
            try:
                self._sock.close()
            except Exception:
                pass
            self._sock = None

        if self._session is not None:
            try:
                self._session.close()
            except Exception:
                pass
            self._session = None

        log.info("RemoteCAN stopped. Stats: %s", self._stats)

    def _vcan_to_zenoh(self) -> None:
        """从 vcan0 读取帧，发送给 server。"""
        while self._running:
            try:
                data = self._sock.recv(CAN_FRAME_SIZE)
                can_id = _extract_can_id(data)

                # 客户端侧过滤（可选，增加安全性）
                if self._hand_can_ids and can_id not in self._hand_can_ids:
                    log.warning(
                        "RemoteCAN: blocked outgoing ID %s (not in whitelist)",
                        _format_can_id(can_id),
                    )
                    self._stats["blocked"] += 1
                    continue

                # 发送给 server
                self._session.put(self.TOPIC_FROM_CLIENT, data)
                self._stats["to_server"] += 1

            except socket.timeout:
                continue
            except Exception as e:
                if self._running:
                    log.error("vcan recv error: %s", e)
                    self._stats["errors"] += 1

    def _on_frame_from_server(self, sample) -> None:
        """处理 server 发来的 CAN 帧。"""
        try:
            payload = bytes(sample.payload) if hasattr(sample, 'payload') else sample

            if len(payload) != CAN_FRAME_SIZE:
                self._stats["errors"] += 1
                return

            can_id = _extract_can_id(payload)

            # 客户端侧过滤（可选）
            if self._hand_can_ids and can_id not in self._hand_can_ids:
                self._stats["blocked"] += 1
                return

            # 写入 vcan0
            self._sock.send(payload)
            self._stats["from_server"] += 1

        except Exception as e:
            log.error("RemoteCAN frame error: %s", e)
            self._stats["errors"] += 1

    def _keepalive(self) -> None:
        """保持线程存活。"""
        import time
        while self._running:
            time.sleep(1.0)


# ── 独立运行（调试用）────────────────────────────────────────────────────────

if __name__ == "__main__":
    """独立运行 RemoteCAN（调试用）。"""
    import argparse
    import time

    logging.basicConfig(level=logging.INFO)

    parser = argparse.ArgumentParser(description="RemoteCAN standalone test")
    parser.add_argument("--endpoint", default="tcp/127.0.0.1:7447")
    parser.add_argument("--vcan", default="vcan0", help="Virtual CAN interface")
    parser.add_argument(
        "--hand-ids", default=None,
        help="Hand CAN IDs for client-side filter (e.g. '0x08,0x18')",
    )
    args = parser.parse_args()

    hand_ids = None
    if args.hand_ids:
        hand_ids = set()
        for part in args.hand_ids.split(","):
            if part.strip():
                hand_ids.add(int(part.strip(), 0))

    can = RemoteCAN(
        endpoint=args.endpoint,
        vcan_iface=args.vcan,
        hand_can_ids=hand_ids,
    )

    try:
        can.start()
        print(f"RemoteCAN running. Press Ctrl+C to stop.")
        print(f"Use: Hand(can_interface='{args.vcan}')")
        while True:
            time.sleep(5)
            print(f"Stats: {can.stats}")
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        can.stop()
