import socket
import struct


MAGIC = b"K1TL"
ODOM_PACKET = 1
SCAN_PACKET = 2
HEADER = struct.Struct("!4sBQI")
ODOM_PAYLOAD = struct.Struct("!3f")
SCAN_META = struct.Struct("!7fI")
MAX_PAYLOAD_BYTES = 8 * 1024 * 1024


def recv_exact(sock: socket.socket, size: int) -> bytes:
    chunks = bytearray()
    while len(chunks) < size:
        chunk = sock.recv(size - len(chunks))
        if not chunk:
            raise ConnectionError("socket closed")
        chunks.extend(chunk)
    return bytes(chunks)


def pack_odometer(stamp_ns: int, x: float, y: float, theta: float) -> bytes:
    payload = ODOM_PAYLOAD.pack(x, y, theta)
    return HEADER.pack(MAGIC, ODOM_PACKET, stamp_ns, len(payload)) + payload


def pack_scan(
    stamp_ns: int,
    angle_min: float,
    angle_max: float,
    angle_increment: float,
    time_increment: float,
    scan_time: float,
    range_min: float,
    range_max: float,
    ranges_payload: bytes,
    range_count: int,
) -> bytes:
    meta = SCAN_META.pack(
        angle_min,
        angle_max,
        angle_increment,
        time_increment,
        scan_time,
        range_min,
        range_max,
        range_count,
    )
    payload = meta + ranges_payload
    return HEADER.pack(MAGIC, SCAN_PACKET, stamp_ns, len(payload)) + payload


def recv_packet(sock: socket.socket) -> tuple[int, int, bytes]:
    header = recv_exact(sock, HEADER.size)
    magic, kind, stamp_ns, payload_len = HEADER.unpack(header)
    if magic != MAGIC:
        raise ValueError("invalid telemetry stream magic")
    if payload_len < 0 or payload_len > MAX_PAYLOAD_BYTES:
        raise ValueError("invalid telemetry payload length")
    payload = recv_exact(sock, payload_len)
    return kind, stamp_ns, payload
