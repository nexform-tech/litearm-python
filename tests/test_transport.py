"""Tests for litearm.transport — InProc pub/sub and query/reply."""
import pytest

from litearm.transport import InProcTransport


class TestInProcPubSub:
    def test_basic_pub_sub(self):
        t = InProcTransport()
        s = t.sub("test/topic")
        assert s.try_recv() is None

        t.pub("test/topic", b"hello")
        t.pub("test/topic", b"world")
        assert s.try_recv() == b"hello"
        assert s.try_recv() == b"world"
        assert s.try_recv() is None

    def test_multiple_subscribers(self):
        t = InProcTransport()
        s1 = t.sub("test/topic")
        s2 = t.sub("test/topic")

        t.pub("test/topic", b"msg")
        assert s1.try_recv() == b"msg"
        assert s2.try_recv() == b"msg"

    def test_topic_isolation(self):
        t = InProcTransport()
        s1 = t.sub("topic_a")
        s2 = t.sub("topic_b")

        t.pub("topic_a", b"for_a")
        assert s1.try_recv() == b"for_a"
        assert s2.try_recv() is None

    def test_fifo_depth_overflow(self):
        t = InProcTransport(fifo_depth=3)
        s = t.sub("test/topic")

        for i in range(10):
            t.pub("test/topic", f"msg{i}".encode())

        # Only last 3 messages should remain
        msgs = []
        while True:
            m = s.try_recv()
            if m is None:
                break
            msgs.append(m)
        assert len(msgs) == 3
        assert msgs == [b"msg7", b"msg8", b"msg9"]


class TestInProcDrainLatest:
    def test_drain_empty(self):
        t = InProcTransport()
        s = t.sub("test/topic")
        assert s.drain_latest() is None

    def test_drain_single(self):
        t = InProcTransport()
        s = t.sub("test/topic")
        t.pub("test/topic", b"only")
        assert s.drain_latest() == b"only"
        assert s.try_recv() is None

    def test_drain_multiple(self):
        t = InProcTransport()
        s = t.sub("test/topic")
        t.pub("test/topic", b"first")
        t.pub("test/topic", b"second")
        t.pub("test/topic", b"latest")
        assert s.drain_latest() == b"latest"
        assert s.try_recv() is None


class TestInProcQueryReply:
    def test_basic_query_reply(self):
        t = InProcTransport()

        def handler(payload: bytes) -> bytes:
            # Echo back with prefix
            return b"reply:" + payload

        t.declare_queryable("test/rpc", handler)
        result = t.query("test/rpc", b"hello")
        assert result == b"reply:hello"

    def test_query_without_queryable_raises(self):
        t = InProcTransport()
        with pytest.raises(RuntimeError, match="No queryable registered"):
            t.query("test/rpc", b"hello")

    def test_double_queryable_raises(self):
        t = InProcTransport()
        t.declare_queryable("test/rpc", lambda p: p)
        with pytest.raises(RuntimeError, match="Queryable already registered"):
            t.declare_queryable("test/rpc", lambda p: p)

    def test_queryable_handler_exception_propagates(self):
        t = InProcTransport()

        def bad_handler(payload: bytes) -> bytes:
            raise ValueError("handler error")

        t.declare_queryable("test/rpc", bad_handler)
        with pytest.raises(ValueError, match="handler error"):
            t.query("test/rpc", b"hello")

    def test_complex_payload_roundtrip(self):
        """Test with msgpack-encoded payloads to simulate real RPC."""
        import msgpack

        t = InProcTransport()

        def rpc_handler(payload: bytes) -> bytes:
            msg = msgpack.unpackb(payload, raw=False)
            method = msg["method"]
            if method == "add":
                result = msg["kwargs"]["a"] + msg["kwargs"]["b"]
                return msgpack.packb({"ok": True, "result": result})
            return msgpack.packb({"ok": False, "error": "unknown"})

        t.declare_queryable("rpc", rpc_handler)

        req = msgpack.packb({"method": "add", "kwargs": {"a": 3, "b": 4}})
        reply = msgpack.unpackb(t.query("rpc", req), raw=False)
        assert reply["ok"] is True
        assert reply["result"] == 7


class TestInProcClose:
    def test_close_clears_state(self):
        t = InProcTransport()
        s = t.sub("test/topic")
        t.pub("test/topic", b"msg")
        t.declare_queryable("rpc", lambda p: p)

        t.close()
        # After close, internal state should be cleared
        # Publishing should not crash (no subscribers)
        t.pub("test/topic", b"after_close")
        # Old subscriber still holds reference to its deque but queue is cleared
        assert s.try_recv() is None
