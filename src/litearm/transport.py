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

    def __init__(
        self,
        mode: str = "peer",
        connect_endpoints: Optional[List[str]] = None,
        listen_endpoints: Optional[List[str]] = None,
        enable_shm: bool = True,
    ) -> None:
        import zenoh
        self._zenoh = zenoh
        cfg = zenoh.Config()
        # Disable discovery (fixed topology)
        cfg.insert_json5("scouting/multicast/enabled", "false")
        cfg.insert_json5("scouting/gossip/enabled", "false")
        cfg.insert_json5("mode", f'"{mode}"')
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
        handler = self._zenoh.handlers.FifoChannel(64)
        subscriber = self._session.declare_subscriber(topic, handler)
        s = _ZenohSub(subscriber, subscriber)
        self._subs.append(subscriber)
        return s

    def query(self, topic: str, payload: bytes) -> bytes:
        """Send a get query and return the first reply payload."""
        replies = self._session.get(topic, payload)
        # zenoh 1.x: get() returns a list-like or single reply
        if hasattr(replies, '__iter__'):
            for reply in replies:
                if hasattr(reply, 'ok') and reply.ok:
                    return bytes(reply.ok.payload)
                # Older API: reply has .payload directly
                if hasattr(reply, 'payload'):
                    return bytes(reply.payload)
            raise RuntimeError("No valid reply received")
        # Single reply object
        if hasattr(replies, 'payload'):
            return bytes(replies.payload)
        return bytes(replies)

    def declare_queryable(self, topic: str, handler: Callable[[bytes], bytes]) -> Any:
        """Register a zenoh queryable. handler(payload) -> reply_bytes."""
        import zenoh as z

        def _query_callback(query: Any) -> None:
            try:
                req_payload = bytes(query.payload) if hasattr(query, 'payload') else b""
                reply_bytes = handler(req_payload)
                query.reply(
                    z.Sample(reply_bytes),
                )
            except Exception as exc:
                # Reply with error info packed
                import msgpack
                err_payload = msgpack.packb({
                    "v": 1, "ok": False,
                    "error_type": type(exc).__name__,
                    "error_msg": str(exc),
                })
                query.reply(z.Sample(err_payload))

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
