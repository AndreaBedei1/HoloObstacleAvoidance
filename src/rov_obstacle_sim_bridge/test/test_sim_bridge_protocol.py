"""Tests for the dependency-free sim-bridge wire protocol.

These do not require ROS 2 or HoloOcean; they exercise the framing used to move
data between the two Python environments.
"""

import socket

import pytest

from rov_obstacle_sim_bridge.sim_bridge_protocol import (
    MSG_CMD_VEL,
    MSG_STATE,
    FrameStream,
    encode_frame,
    make_command_header,
)
from rov_obstacle_sim_bridge.sim_bridge_protocol import _decode_body  # noqa: E402


def test_encode_decode_roundtrip_no_blob():
    header = {"type": MSG_STATE, "seq": 7, "pose": {"x": 1.0}}
    frame = encode_frame(header)
    # strip the leading 4-byte total length, then decode the body
    body = frame[4:]
    decoded_header, blob = _decode_body(body)
    assert decoded_header == header
    assert blob == b""


def test_encode_decode_roundtrip_with_blob():
    header = {"type": MSG_STATE, "image": {"width": 2, "height": 1}}
    blob = bytes(range(6))
    frame = encode_frame(header, blob)
    decoded_header, decoded_blob = _decode_body(frame[4:])
    assert decoded_header == header
    assert decoded_blob == blob


def test_make_command_header():
    h = make_command_header(surge=0.5, yaw_rate=-0.3)
    assert h["type"] == MSG_CMD_VEL
    assert h["surge"] == pytest.approx(0.5)
    assert h["yaw_rate"] == pytest.approx(-0.3)
    assert h["sway"] == pytest.approx(0.0)


def test_framestream_send_and_try_read():
    a, b = socket.socketpair()
    try:
        writer = FrameStream(a)
        reader = FrameStream(b)
        writer.send({"type": MSG_STATE, "n": 1})
        writer.send({"type": MSG_STATE, "n": 2}, b"\x00\x01\x02")

        first = _read_blocking(reader)
        second = _read_blocking(reader)
        assert first[0]["n"] == 1 and first[1] == b""
        assert second[0]["n"] == 2 and second[1] == b"\x00\x01\x02"
    finally:
        a.close()
        b.close()


def test_framestream_read_latest_drops_stale():
    a, b = socket.socketpair()
    try:
        writer = FrameStream(a)
        reader = FrameStream(b)
        for n in range(5):
            writer.send({"type": MSG_CMD_VEL, "n": n})
        # Give the OS a moment to deliver, then read only the newest.
        latest = None
        for _ in range(1000):
            frame = reader.read_latest()
            if frame is not None:
                latest = frame
                break
        assert latest is not None
        assert latest[0]["n"] == 4
    finally:
        a.close()
        b.close()


def test_try_read_returns_none_when_empty():
    a, b = socket.socketpair()
    try:
        reader = FrameStream(b)
        assert reader.try_read() is None
    finally:
        a.close()
        b.close()


def _read_blocking(reader: FrameStream, max_iters: int = 100000):
    for _ in range(max_iters):
        frame = reader.try_read()
        if frame is not None:
            return frame
    raise AssertionError("no frame received")
