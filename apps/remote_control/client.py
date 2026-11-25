import argparse
from importlib import import_module
import json
import socket
from pathlib import Path
import sys
import time

import numpy as np

from protocol import LENGTH, MAX_OBSERVATION, encode_json, receive_message, send_message


def main():
    parser = argparse.ArgumentParser(description="Run a preset or custom G1 remote planner.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8770)
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument("--preset", choices=("stationary", "walk", "turn", "hold"))
    selection.add_argument("--planner", metavar="MODULE:FUNCTION")
    args = parser.parse_args()
    planner = None
    if args.planner:
        module, separator, name = args.planner.partition(":")
        if not separator or not module or not name:
            parser.error("--planner requires MODULE:FUNCTION")
        sys.path.insert(0, str(Path.cwd()))
        planner = getattr(import_module(module), name)
    preset = args.preset or "stationary"
    first_capture_ns = None
    held_pose = None
    with socket.create_connection((args.host, args.port), timeout=5) as connection:
        while True:
            payload = receive_message(connection, MAX_OBSERVATION, time.monotonic_ns() + 5_000_000_000)
            metadata_length, = LENGTH.unpack_from(payload)
            start = LENGTH.size
            if not 0 < metadata_length <= len(payload) - start:
                raise ValueError("invalid observation metadata length")
            metadata = json.loads(payload[start:start + metadata_length])
            if metadata["version"] != 1 or metadata["dtype"] != "<f4":
                raise ValueError("bridge protocol version or depth format mismatch")
            depth = np.frombuffer(memoryview(payload)[start + metadata_length:].toreadonly(),
                                  dtype="<f4").reshape(metadata["shape"])
            mode = metadata["command"]
            capture_ns = metadata["capture_ns"]
            if first_capture_ns is None:
                first_capture_ns = capture_ns
                print(f"Connected: {mode}, depth {tuple(depth.shape)}, session {metadata['session']}", flush=True)
            elapsed = (capture_ns - first_capture_ns) / 1e9
            started = time.monotonic_ns()
            if planner is not None:
                target = planner(metadata, depth)
            elif mode == "joints" and preset == "hold":
                if held_pose is None:
                    held_pose = metadata["state"]["q"]
                target = {"q": held_pose, "dq": [0.0] * 29}
            elif mode == "velocity" and preset != "hold":
                moving = 3 <= elapsed < 5
                target = {"vx": 0.10 if moving and preset == "walk" else 0.0, "vy": 0.0,
                          "wz": 0.15 if moving and preset == "turn" else 0.0}
            else:
                raise ValueError("choose --preset hold for joints mode or --planner MODULE:FUNCTION")
            compute_ms = (time.monotonic_ns() - started) / 1e6
            reply = {**target, "session": metadata["session"], "sequence": metadata["sequence"],
                     "compute_ms": compute_ms}
            connection.settimeout(0.25)
            send_message(connection, encode_json(reply))


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    except (EOFError, OSError, ValueError) as error:
        raise SystemExit(f"Remote planner stopped: {error}")
