import argparse
from collections import deque
from contextlib import ExitStack
import json
import math
from pathlib import Path
import secrets
import select
import socket
import statistics
import struct
import sys
import threading
import time

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from depth_stream import DEFAULT_CAMERA_CONFIG, DepthCadence, profile_camera
from protocol import (LENGTH, LOCAL_HEADER, MAX_OBSERVATION, MAX_TARGET, PAIR_WINDOW_NS,
                      TARGET_AGE_NS, encode_json, receive_message, send_message)


class ObservationBridge:
    def __init__(self, port, target_socket, mode, stream):
        self.port = port
        self.target_socket = target_socket
        self.mode = mode
        self.stream = stream
        self.condition = threading.Condition()
        self.states = deque(maxlen=256)
        self.frame = None
        self.stopped = False
        self.subscriber = None
        self.thread = None
        self.listener = None
        self.connection = None

    def start(self, domain, interface):
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
        from unitree_sdk2py.utils.crc import CRC

        crc = CRC() if domain == 0 else None

        def receive(state):
            timestamp = time.monotonic_ns()
            if len(state.motor_state) < 29:
                return
            if crc is not None and (state.mode_machine != 5 or crc.Crc(state) != state.crc):
                return
            joints = state.motor_state[:29]
            snapshot = {"q": [motor.q for motor in joints], "dq": [motor.dq for motor in joints],
                        "imu_wxyz": list(state.imu_state.quaternion), "gyro_rad_s": list(state.imu_state.gyroscope),
                        "accelerometer_m_s2": list(state.imu_state.accelerometer)}
            if any(len(snapshot[key]) != size for key, size in
                   (("imu_wxyz", 4), ("gyro_rad_s", 3), ("accelerometer_m_s2", 3))):
                return
            if not all(math.isfinite(value) for vector in snapshot.values() for value in vector):
                return
            if not 0.5 <= sum(value * value for value in snapshot["imu_wxyz"]) <= 1.5:
                return
            with self.condition:
                self.states.append((timestamp, snapshot))
                self.condition.notify()

        self.listener = socket.socket()
        self.listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.listener.bind(("127.0.0.1", self.port))
        self.listener.listen(1)
        self.listener.settimeout(0.1)
        ChannelFactoryInitialize(domain, interface)
        self.subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.subscriber.Init(receive, 1)
        self.thread = threading.Thread(target=self._serve, name="remote-observations", daemon=True)
        self.thread.start()
        print(f"Remote bridge: 127.0.0.1:{self.listener.getsockname()[1]} ({self.mode}), "
              f"depth {self.stream.width}x{self.stream.height} @ {self.stream.fps:g} Hz target; "
              "use an SSH tunnel. No motor commands from bridge.", flush=True)

    def deposit(self, depth, captured_ns, intrinsics):
        if depth.shape != (self.stream.height, self.stream.width):
            raise ValueError(f"Depth shape {depth.shape} does not match the configured stream")
        with self.condition:
            self.frame = (depth, captured_ns, intrinsics)
            self.condition.notify()

    def _serve(self):
        with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as local:
            local.setblocking(False)
            while not self.stopped:
                try:
                    connection, _ = self.listener.accept()
                except TimeoutError:
                    continue
                except OSError:
                    if self.stopped:
                        return
                    raise
                self.connection = connection
                session = secrets.randbits(63) or 1
                cadence = DepthCadence(self.stream)
                last_frame_ns = time.monotonic_ns()
                first_sent_at = 0
                sent_at = 0
                last_forwarded_at = 0
                sequence = 0
                accepted = skipped = 0
                recent_rtts = deque(maxlen=6000)
                try:
                    with connection:
                        while not self.stopped:
                            if select.select([connection], [], [], 0)[0]:
                                if not connection.recv(1, socket.MSG_PEEK | socket.MSG_DONTWAIT):
                                    raise EOFError("connection closed")
                                raise ValueError("target sent without an outstanding observation")
                            with self.condition:
                                self.condition.wait_for(
                                    lambda: self.stopped or (self.frame is not None and
                                            self.frame[1] > last_frame_ns and
                                             time.monotonic_ns() >= cadence.next_ns),
                                    timeout=0.025)
                                if self.stopped:
                                    break
                                if self.frame is None or self.frame[1] <= last_frame_ns or (
                                        time.monotonic_ns() < cadence.next_ns):
                                    continue
                                depth, captured_ns, intrinsics = self.frame
                                if not self.states or self.states[-1][0] < captured_ns:
                                    self.condition.wait_for(
                                        lambda: self.stopped or (self.states and self.states[-1][0] >= captured_ns),
                                        timeout=0.01)
                                paired = min(self.states, key=lambda sample: abs(sample[0] - captured_ns), default=None)
                            last_frame_ns = captured_ns
                            if paired is None or abs(paired[0] - captured_ns) > PAIR_WINDOW_NS:
                                skipped += 1
                                if skipped == 1 or skipped % 60 == 0:
                                    print("Remote observation skipped: no state within 50 ms", flush=True)
                                continue
                            clock = time.monotonic_ns()
                            if clock - captured_ns >= TARGET_AGE_NS:
                                skipped += 1
                                if skipped == 1 or skipped % 60 == 0:
                                    print("Remote observation skipped: depth already expired", flush=True)
                                continue
                            depth = np.ascontiguousarray(depth, dtype="<f4")
                            height, width = depth.shape
                            sequence += 1
                            metadata = encode_json({"version": 1, "session": session, "sequence": sequence,
                                                    "capture_ns": captured_ns, "state_ns": paired[0],
                                                    "shape": [height, width], "dtype": "<f4", "depth_fps": self.stream.fps,
                                                    "intrinsics": intrinsics, "state": paired[1], "command": self.mode})
                            if LENGTH.size + len(metadata) + depth.nbytes > MAX_OBSERVATION:
                                raise ValueError("depth frame exceeds protocol size limit")
                            sent_at = time.monotonic_ns()
                            time_left = captured_ns + TARGET_AGE_NS - sent_at
                            if time_left <= 0:
                                skipped += 1
                                continue
                            connection.settimeout(time_left / 1e9)
                            if not first_sent_at:
                                first_sent_at = sent_at
                            cadence.advance(sent_at)
                            send_message(connection, LENGTH.pack(len(metadata)) + metadata + depth.tobytes())
                            target = json.loads(receive_message(connection, MAX_TARGET, captured_ns + TARGET_AGE_NS))
                            received_ns = time.monotonic_ns()
                            if received_ns - captured_ns >= TARGET_AGE_NS:
                                raise ValueError("expired target")
                            if (not isinstance(target, dict) or type(target.get("session")) is not int or
                                    type(target.get("sequence")) is not int or
                                    (target["session"], target["sequence"]) != (session, sequence)):
                                raise ValueError("target does not match the outstanding observation")
                            fields = ("vx", "vy", "wz") if self.mode == "velocity" else ("q", "dq")
                            if set(target) != {"session", "sequence", "compute_ms", *fields}:
                                raise ValueError("wrong target fields")
                            compute_ms = target["compute_ms"]
                            if type(compute_ms) not in (float, int) or not 0 <= compute_ms <= 250:
                                raise ValueError("invalid compute_ms")
                            if self.mode == "velocity":
                                values = [target[key] for key in fields]
                            else:
                                if any(not isinstance(target[key], list) or len(target[key]) != 29 for key in fields):
                                    raise ValueError("whole-body targets require 29 positions and velocities")
                                values = target["q"] + target["dq"]
                            if not all(type(value) in (float, int) and math.isfinite(value) and abs(value) <= 1e6
                                       for value in values):
                                raise ValueError("invalid target values")
                            if self.stopped:
                                break
                            packet = LOCAL_HEADER.pack(session, sequence, captured_ns) + struct.pack(f"<{len(values)}f", *values)
                            local.sendto(packet, self.target_socket)
                            accepted += 1
                            last_forwarded_at = sent_at
                            rtt_ms = (received_ns - sent_at) / 1e6
                            recent_rtts.append(rtt_ms)
                            print(f"Remote target session={session} seq={sequence} compute={compute_ms:.2f}ms "
                                  f"rtt={rtt_ms:.2f}ms age={(received_ns - captured_ns) / 1e6:.2f}ms",
                                  flush=True)
                except (OSError, EOFError, ValueError, OverflowError) as error:
                    print(f"Remote bridge locked session {session}: {error}", flush=True)
                finally:
                    self.connection = None
                    try:
                        local.sendto(LOCAL_HEADER.pack(0, 0, time.monotonic_ns()), self.target_socket)
                    except OSError:
                        pass
                    if recent_rtts:
                        samples = sorted(recent_rtts)
                        rate = (accepted - 1) * 1e9 / max(1, last_forwarded_at - first_sent_at)
                        print(f"Remote session {session}: forwarded={accepted} skipped={skipped} "
                              f"rate={rate:.2f}Hz rtt_p50={statistics.median(samples):.2f}ms "
                              f"rtt_p95={samples[math.ceil(len(samples) * 0.95) - 1]:.2f}ms "
                              f"rtt_samples={len(samples)}", flush=True)

    def close(self):
        with self.condition:
            self.stopped = True
            self.condition.notify_all()
        connection = self.connection
        if connection is not None:
            try:
                connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        if self.listener is not None:
            self.listener.close()
        if self.thread is not None:
            self.thread.join()
        if self.subscriber is not None:
            self.subscriber.Close()


def main():
    parser = argparse.ArgumentParser(description="Export G1 depth/state and return local dry-run targets.")
    parser.add_argument("--interface", required=True)
    camera_options = parser.add_mutually_exclusive_group()
    camera_options.add_argument("--profile", help="Task name or profile JSON (uses camera.config)")
    camera_options.add_argument("--camera-config", type=Path, help="Explicit camera module JSON")
    parser.add_argument("--port", type=int, default=8770)
    parser.add_argument("--target-socket", required=True, type=Path)
    parser.add_argument("--command", choices=("velocity", "joints"), default="velocity")
    args = parser.parse_args()
    if not args.target_socket.is_absolute() or not 1 <= args.port <= 65535:
        parser.error("Use an absolute Unix socket path and TCP port 1..65535")
    if args.interface == "lo" or args.interface not in {name for _, name in socket.if_nameindex()}:
        parser.error("Use the actual non-loopback G1 DDS interface")

    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "monitor"))
    from camera_sources import RealSenseSource

    try:
        camera_config = profile_camera(args.profile) if args.profile else args.camera_config or DEFAULT_CAMERA_CONFIG
        camera = RealSenseSource(camera_config)
    except (OSError, ValueError) as error:
        parser.error(str(error))
    with ExitStack() as stack:
        stack.callback(camera.close)
        camera.open()
        intrinsics = camera.intrinsics()
        bridge = ObservationBridge(args.port, str(args.target_socket), args.command, camera.stream)
        stack.callback(bridge.close)
        bridge.start(0, args.interface)
        while True:
            frame = camera.read()
            if frame is not None:
                bridge.deposit(frame.raw, round(frame.capture_monotonic * 1e9), intrinsics)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
