import json
import struct
import time


LENGTH = struct.Struct("!I")
MAX_OBSERVATION = 8 * 1024 * 1024
MAX_TARGET = 4096
TARGET_AGE_NS = 250_000_000
PAIR_WINDOW_NS = 50_000_000
LOCAL_HEADER = struct.Struct("<QQQ")


def send_message(connection, payload):
    connection.sendall(LENGTH.pack(len(payload)) + payload)


def receive_message(connection, limit, deadline_ns):
    def read_exact(size):
        data = bytearray(size)
        received = 0
        while received < size:
            remaining = deadline_ns - time.monotonic_ns()
            if remaining <= 0:
                raise TimeoutError("reply deadline exceeded")
            connection.settimeout(remaining / 1e9)
            count = connection.recv_into(memoryview(data)[received:])
            if not count:
                raise EOFError("connection closed")
            received += count
        return data

    size, = LENGTH.unpack(read_exact(LENGTH.size))
    if not 0 < size <= limit:
        raise ValueError("invalid message length")
    return read_exact(size)


def encode_json(value):
    return json.dumps(value, separators=(",", ":"), allow_nan=False).encode("utf-8")
