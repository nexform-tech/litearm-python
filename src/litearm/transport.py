"""Pluggable transport layer for litearm v4.

Provides pub/sub and query/reply abstractions with two backends:
- InProcTransport: in-process queues (zero-dependency, for testing)
- ZenohTransport: zenoh networking (production use)
"""
from __future__ import annotations

import threading
from collections import deque
from typing import Any, Callable, List, Optional


# ── Abstract base classes ────────────────────────────────────────────────────

class Sub:
    """Subscription handle: FIFO, try_recv() non-blocking, drain_latest()."""

    def try_recv(self) -> Optional[bytes]:
        """Non-blocking receive. Returns bytes or None if empty."""
        raise NotImplementedError

    def drain_latest(self) -> Optional[bytes]:
        """Drain to the last message, discarding older ones. Returns latest or None."""
        latest = None
        while True:
            m = self.try_recv()
            if m is None:
                break
            latest = m
        return latest


class Transport:
    """Abstract transport: pub/sub + query/reply."""

    def pub(self, topic: str, payload: bytes) -> None:
        """Publish payload to topic."""
        raise NotImplementedError

    def sub(self, topic: str) -> Sub:
        """Subscribe to topic. Returns a Sub handle."""
        raise NotImplementedError

    def query(self, topic: str, payload: bytes) -> bytes:
        """Send a query and wait for a single reply. Returns reply bytes."""
        raise NotImplementedError

    def declare_queryable(self, topic: str, handler: Callable[[bytes], bytes]) -> Any:
        """Register a queryable on topic. handler(payload) -> reply_bytes."""
        raise NotImplementedError

    def close(self) -> None:
        """Close the transport and release resources."""
        pass


# ── InProc backend (testing / single-process mode) ──────────────────────────

class _InProcSub(Sub):
    def __init__(self, q: deque) -> None:
        self._q = q

    def try_recv(self) -> Optional[bytes]:
        return self._q.popleft() if self._q else None


class InProcTransport(Transport):
    """In-process pub/sub + query/reply. Zero-dependency, for testing."""

    def __init__(self, fifo_depth: int = 16) -> None:
        self._subs: dict = {}       # topic -> list[deque]
        self._all_queues: list = [] # all deques (for close cleanup)
        self._queryables: dict = {} # topic -> handler callable
        self._depth = fifo_depth
        self._lock = threading.Lock()

    def pub(self, topic: str, payload: bytes) -> None:
        with self._lock:
            queues = self._subs.get(topic, [])
            for q in queues:
                q.append(payload)
                while len(q) > self._depth:
                    q.popleft()

    def sub(self, topic: str) -> _InProcSub:
        q: deque = deque()
        with self._lock:
            self._subs.setdefault(topic, []).append(q)
            self._all_queues.append(q)
        return _InProcSub(q)

    def query(self, topic: str, payload: bytes) -> bytes:
        """Call the registered queryable handler synchronously."""
        handler = self._queryables.get(topic)
        if handler is None:
            raise RuntimeError(f"No queryable registered for topic: {topic}")
        return handler(payload)

    def declare_queryable(self, topic: str, handler: Callable[[bytes], bytes]) -> Any:
        """Register a handler for query/reply on topic."""
        with self._lock:
            if topic in self._queryables:
                raise RuntimeError(f"Queryable already registered for topic: {topic}")
            self._queryables[topic] = handler
        return topic  # return a handle for potential cleanup

    def close(self) -> None:
        with self._lock:
            for q in self._all_queues:
                q.clear()
            self._all_queues.clear()
            self._subs.clear()
            self._queryables.clear()


# ── Zenoh backend (production) ───────────────────────────────────────────────

class _ZenohSub(Sub):
    def __init__(self, subscriber: Any, handler: Any) -> None:
        self._sub = subscriber
        self._handler = handler  # FIFO handler with try_recv

    def try_recv(self) -> Optional[bytes]:
        try:
            sample = self._handler.try_recv()
        except Exception:
            return None
        if sample is None:
            return None
        try:
            return bytes(sample.payload)
        except Exception:
            return None


class ZenohTransport(Transport):
    """Zenoh transport with SHM, disabled discovery, explicit endpoints.

    Supports pub/sub (put/declare_subscriber) and query/reply (get/declare_queryable).
    """

    LOCAL_ENDPOINT = "tcp/127.0.0.1:7447"

    # RPC 默认超时（秒）。zenoh 默认仅约 10s，运动方法（movej/movel/...）
    # 可能跑几十秒甚至更久，会导致运动没走完客户端就误报 No valid reply。
    # 设成 5min 上限：单次运动调用不可超过 5 分钟，超了即报错。
    #   - 快调用（fk/get_state 等）：handler 一回就立即返回，大超时零额外延迟（实测）
    #   - 长运动：一直等到 server 真正回复完成（上限 5min）
    # 注意：只能用有限数，不能用 float("inf")——zenoh 会把 inf 转成负数并报
    #      ValueError: negative timeout。也不能太大（如 86400）——zenoh 1.x 有 bug
    #      会导致 query reply 永远收不到。300s 实测安全。
    DEFAULT_QUERY_TIMEOUT = 300.0  # 5 分钟

    def __init__(
        self,
        mode: str = "peer",
        connect_endpoints: Optional[List[str]] = None,
        listen_endpoints: Optional[List[str]] = None,
        enable_shm: bool = False,
        query_timeout: float = DEFAULT_QUERY_TIMEOUT,
    ) -> None:
        import zenoh
        self._zenoh = zenoh
        self._query_timeout = query_timeout
        cfg = zenoh.Config()
        # Disable discovery (fixed topology)
        cfg.insert_json5("scouting/multicast/enabled", "false")
        cfg.insert_json5("scouting/gossip/enabled", "false")
        cfg.insert_json5("mode", f'"{mode}"')
        # Query timeout（毫秒）— 默认 10s 对长运动太短，设 24 小时。
        # 客户端 get() 的 timeout 参数也需配合（已在 query() 里传了）。
        cfg.insert_json5("queries_default_timeout", str(int(query_timeout * 1000)))
        # Enable shared memory for large payloads
        if enable_shm:
            cfg.insert_json5("transport/shared_memory/enabled", "true")
        # Listen endpoints (server side)
        if listen_endpoints:
            eps = ",".join(f'"{e}"' for e in listen_endpoints)
            cfg.insert_json5("listen/endpoints", f"[{eps}]")
        # Connect endpoints (client side)
        if connect_endpoints:
            eps = ",".join(f'"{e}"' for e in connect_endpoints)
            cfg.insert_json5("connect/endpoints", f"[{eps}]")
        self._session = zenoh.open(cfg)
        self._subs: list = []
        self._queryables: list = []

    def pub(self, topic: str, payload: bytes) -> None:
        self._session.put(topic, payload)

    def sub(self, topic: str) -> _ZenohSub:
        # FIFO 容量必须能容纳最长 query timeout 期间的全部广播消息。
        # state 广播 50Hz × 最大 query timeout 300s = 15000 条消息。设 20000 留余量。
        # 若 FIFO 满，zenoh pipeline 阻塞 → query reply 永久丢失。
        handler = self._zenoh.handlers.FifoChannel(20000)
        subscriber = self._session.declare_subscriber(topic, handler)
        s = _ZenohSub(subscriber, subscriber)
        self._subs.append(subscriber)
        return s

    def query(self, topic: str, payload: bytes) -> bytes:
        """Send a get query and return the first reply payload.

        zenoh 1.x: payload 必须用关键字参数传（第二个位置参数是 handler）。
        get() 返回一个可迭代的 reply 通道；每个 reply 的 .ok 是成功 Sample，
        .err 是错误。

        显式传 consolidation=NONE（立即返回首个 reply，不等待其他 queryable）
        和 allowed_destination=all（确保发到 peer 模式的远端），避免默认行为
        导致长 RPC reply 丢失。
        """
        import zenoh
        replies = self._session.get(
            topic,
            payload=payload,
            timeout=self._query_timeout,
            consolidation=zenoh.QueryConsolidation(zenoh.ConsolidationMode.NONE),
        )
        for reply in replies:
            # zenoh 1.x: reply.ok 是成功 Sample（可能为 None），reply.err 是错误
            ok = getattr(reply, 'ok', None)
            if ok is not None:
                return bytes(ok.payload)
            # 回退：reply 直接带 payload
            if hasattr(reply, 'payload'):
                return bytes(reply.payload)
        raise RuntimeError("No valid reply received")

    def declare_queryable(self, topic: str, handler: Callable[[bytes], bytes]) -> Any:
        """Register a zenoh queryable. handler(payload) -> reply_bytes.

        zenoh 1.x: query.reply(key_expr, payload) —— 用 query 自身的 key_expr
        回复，payload 直接传 bytes。
        """

        def _query_callback(query: Any) -> None:
            key = query.key_expr
            try:
                req_payload = bytes(query.payload) if query.payload is not None else b""
                reply_bytes = handler(req_payload)
                query.reply(key, reply_bytes)
            except Exception as exc:
                # handler 内部异常兜底：回一个错误 reply（正常情况 handler 自己已把
                # 业务异常编码进 reply_bytes，这里只兜底传输层异常）
                try:
                    query.reply_err(str(exc).encode("utf-8"))
                except Exception:
                    pass

        queryable = self._session.declare_queryable(topic, _query_callback)
        self._queryables.append(queryable)
        return queryable

    def close(self) -> None:
        try:
            self._session.close()
        except Exception:
            pass


# ── Factory ──────────────────────────────────────────────────────────────────

def make_transport(kind: str = "zenoh", **kw: Any) -> Transport:
    """Factory: kind='zenoh' (default) / 'inproc' (testing)."""
    if kind == "inproc":
        return InProcTransport(**kw)
    return ZenohTransport(**kw)


def local_server(**kw: Any) -> ZenohTransport:
    """Local server (daemon): listens on loopback."""
    return ZenohTransport(listen_endpoints=[ZenohTransport.LOCAL_ENDPOINT], **kw)


def local_client(**kw: Any) -> ZenohTransport:
    """Local client (cli): connects to loopback daemon."""
    return ZenohTransport(connect_endpoints=[ZenohTransport.LOCAL_ENDPOINT], **kw)
