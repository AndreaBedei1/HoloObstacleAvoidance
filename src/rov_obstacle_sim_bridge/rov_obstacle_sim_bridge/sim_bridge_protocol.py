"""Length-prefixed socket protocol shared by the HoloOcean sim server and the
ROS 2 bridge node.

This module is deliberately dependency-free (Python standard library only) so it
can be imported from BOTH Python environments used by this project:

* the conda ``ocean`` environment (Python 3.9) that runs HoloOcean, and
* the pixi ROS 2 Lyrical environment (Python 3.12) that runs the ROS 2 nodes.

It contains NO ROS 2 and NO HoloOcean imports.

Wire format (one frame)::

    [4 bytes total_len][4 bytes json_len][json_len bytes UTF-8 JSON][blob bytes]

``total_len`` counts everything after itself (i.e. ``4 + json_len + len(blob)``).
The JSON header is a small dict; ``blob`` is optional raw binary (e.g. an RGB
image buffer).  Keeping the image out of the JSON avoids base64 overhead.

The server streams ``state`` frames; the client streams ``cmd_vel`` frames.  The
same frame format is used in both directions.
"""

from __future__ import annotations

import json
import socket
import struct
from typing import Any, Optional, Tuple

# Default localhost endpoint for the bridge.  Loopback only: this is a
# simulation-internal channel, never exposed to a network.
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 47654

_HEADER_STRUCT = struct.Struct(">I")  # 4-byte big-endian unsigned int

# Message type tags carried inside the JSON header under the "type" key.
MSG_STATE = "state"
MSG_CMD_VEL = "cmd_vel"
MSG_HELLO = "hello"


def encode_frame(header: dict, blob: bytes = b"") -> bytes:
    """Serialise *header* (dict) and optional *blob* into one wire frame."""
    json_bytes = json.dumps(header, separators=(",", ":")).encode("utf-8")
    json_len = _HEADER_STRUCT.pack(len(json_bytes))
    body = json_len + json_bytes + blob
    total_len = _HEADER_STRUCT.pack(len(body))
    return total_len + body


def _decode_body(body: bytes) -> Tuple[dict, bytes]:
    """Split a frame *body* (without the leading total-length) into header+blob."""
    (json_len,) = _HEADER_STRUCT.unpack_from(body, 0)
    start = _HEADER_STRUCT.size
    json_bytes = body[start:start + json_len]
    blob = body[start + json_len:]
    header = json.loads(json_bytes.decode("utf-8"))
    return header, blob


class FrameStream:
    """Buffered, non-blocking framed reader/writer over a TCP socket.

    ``send`` is blocking (uses ``sendall``).  ``try_read`` is non-blocking and
    returns ``None`` when no complete frame is buffered yet, so it is safe to
    poll from a ROS 2 timer or a sim loop without stalling.
    """

    def __init__(self, sock: socket.socket) -> None:
        self._sock = sock
        self._sock.setblocking(False)
        self._buf = bytearray()
        self._closed = False

    @property
    def closed(self) -> bool:
        return self._closed

    # -- writing -------------------------------------------------------------
    def send(self, header: dict, blob: bytes = b"") -> None:
        if self._closed:
            raise ConnectionError("FrameStream is closed")
        frame = encode_frame(header, blob)
        try:
            self._sock.setblocking(True)
            self._sock.sendall(frame)
        except OSError as exc:
            self._closed = True
            raise ConnectionError(f"send failed: {exc}") from exc
        finally:
            try:
                self._sock.setblocking(False)
            except OSError:
                self._closed = True

    # -- reading -------------------------------------------------------------
    def _pump(self) -> None:
        """Read whatever bytes are currently available into the buffer."""
        while True:
            try:
                chunk = self._sock.recv(65536)
            except BlockingIOError:
                return
            except OSError as exc:
                self._closed = True
                raise ConnectionError(f"recv failed: {exc}") from exc
            if chunk == b"":
                # Peer closed the connection.
                self._closed = True
                return
            self._buf.extend(chunk)

    def try_read(self) -> Optional[Tuple[dict, bytes]]:
        """Return the next complete ``(header, blob)`` frame, or ``None``."""
        self._pump()
        if len(self._buf) < _HEADER_STRUCT.size:
            return None
        (total_len,) = _HEADER_STRUCT.unpack_from(self._buf, 0)
        frame_end = _HEADER_STRUCT.size + total_len
        if len(self._buf) < frame_end:
            return None
        body = bytes(self._buf[_HEADER_STRUCT.size:frame_end])
        del self._buf[:frame_end]
        return _decode_body(body)

    def read_latest(self) -> Optional[Tuple[dict, bytes]]:
        """Drain all buffered frames and return only the most recent one.

        Useful for live sensor/command streams where stale frames should be
        dropped rather than queued (avoids latency build-up).
        """
        latest: Optional[Tuple[dict, bytes]] = None
        while True:
            frame = self.try_read()
            if frame is None:
                return latest
            latest = frame

    def close(self) -> None:
        self._closed = True
        try:
            self._sock.close()
        except OSError:
            pass


def connect(host: str = DEFAULT_HOST, port: int = DEFAULT_PORT,
            timeout_s: float = 5.0) -> FrameStream:
    """Connect to a sim server and return a :class:`FrameStream` (client side)."""
    sock = socket.create_connection((host, port), timeout=timeout_s)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    return FrameStream(sock)


def make_command_header(
    surge: float = 0.0,
    sway: float = 0.0,
    heave: float = 0.0,
    roll_rate: float = 0.0,
    pitch_rate: float = 0.0,
    yaw_rate: float = 0.0,
) -> dict:
    """Build a ``cmd_vel`` header from abstract body-frame velocity components."""
    return {
        "type": MSG_CMD_VEL,
        "surge": float(surge),
        "sway": float(sway),
        "heave": float(heave),
        "roll_rate": float(roll_rate),
        "pitch_rate": float(pitch_rate),
        "yaw_rate": float(yaw_rate),
    }


def coerce_float(value: Any, default: float = 0.0) -> float:
    """Best-effort float conversion used when decoding headers."""
    try:
        return float(value)
    except (TypeError, ValueError):
        return default
