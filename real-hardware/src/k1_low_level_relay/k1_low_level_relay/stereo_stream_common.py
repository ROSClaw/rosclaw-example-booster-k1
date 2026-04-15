import socket
import struct


MAGIC = b"K1ST"
LEFT_EYE = 0
RIGHT_EYE = 1
HEADER = struct.Struct("!4sBQI")
MAX_PAYLOAD_BYTES = 16 * 1024 * 1024


def pack_frame(eye_id: int, stamp_ns: int, payload: bytes) -> bytes:
    return HEADER.pack(MAGIC, eye_id, stamp_ns, len(payload)) + payload


def recv_exact(sock: socket.socket, size: int) -> bytes:
    chunks = bytearray()
    while len(chunks) < size:
        chunk = sock.recv(size - len(chunks))
        if not chunk:
            raise ConnectionError("socket closed")
        chunks.extend(chunk)
    return bytes(chunks)


def recv_frame(sock: socket.socket) -> tuple[int, int, bytes]:
    header = recv_exact(sock, HEADER.size)
    magic, eye_id, stamp_ns, payload_len = HEADER.unpack(header)
    if magic != MAGIC:
        raise ValueError("invalid stereo stream magic")
    if payload_len < 0 or payload_len > MAX_PAYLOAD_BYTES:
        raise ValueError("invalid stereo stream payload length")
    payload = recv_exact(sock, payload_len)
    return eye_id, stamp_ns, payload
