"""Viewer-only cameras and Select/Back input for the MuJoCo scene."""
from array import array
from contextlib import ExitStack, contextmanager
import fcntl
import os
from pathlib import Path
from queue import SimpleQueue
from struct import Struct
import time
import xml.etree.ElementTree as ET


CHASE_CAMERA = "g1_viewer_chase"
EGO_CAMERA = "g1_viewer_ego"
JS_EVENT = Struct("=IhBB")
JSIOCGBTNMAP = 0x84006A34  # _IOR('j', 0x34, uint16_t[512])
BTN_SELECT = 0x13A


def add_viewer_cameras(scene_path, output_path, ego_camera):
    """Copy the scene and add viewer cameras without resaving the compiled model."""
    scene = ET.parse(scene_path)
    torso = scene.find(".//body[@name='torso_link']")
    robot = None
    if torso is None:
        robot_include = None
        for include in scene.getroot().iter("include"):
            included = scene_path.parent / include.attrib["file"]
            include.set("file", str(included.resolve()))
            if included.name == "g1_29dof.xml":
                robot_include = include
                robot_path = included
        if robot_include is None:
            raise ValueError(f"Missing G1 robot definition in {scene_path}")
        robot = ET.parse(robot_path)
        torso = robot.find(".//body[@name='torso_link']")
        for compiler in robot.getroot().iter("compiler"):
            for attribute in ("assetdir", "meshdir", "texturedir"):
                if attribute in compiler.attrib:
                    compiler.set(attribute, str((robot_path.parent / compiler.attrib[attribute]).resolve()))
        robot_copy = output_path.with_name("viewer_robot.xml")
        robot_include.set("file", str(robot_copy))
    if torso is None:
        raise ValueError(f"Missing torso_link in {scene_path}")

    ET.SubElement(torso, "camera", name=CHASE_CAMERA, mode="fixed",
                  pos="-2.35 0 0.86", xyaxes="0 -1 0 0.342 0 0.940")
    if ego_camera is None:
        ego_camera = EGO_CAMERA
        ET.SubElement(torso, "camera", name=ego_camera, mode="fixed",
                      pos="0.10 0 0.44", xyaxes="0 -1 0 0.342 0 0.940", fovy="55")
    if robot is not None:
        robot.write(robot_copy, encoding="utf-8")
    scene.write(output_path, encoding="utf-8")
    return ego_camera


class SelectButtons:
    """Read Select independently of the policy process using Linux joystick events."""
    def __init__(self):
        self.devices = {}
        self.scan_at = 0.0

    def poll(self):
        now = time.monotonic()
        if now >= self.scan_at:
            self.scan_at = now + 1.0
            for path in Path("/dev/input").glob("js[0-9]*"):
                if path in self.devices or not path.name[2:].isdigit():
                    continue
                try:
                    fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK | os.O_CLOEXEC)
                except OSError:
                    continue
                mapping = array("H", [0]) * 512
                try:
                    fcntl.ioctl(fd, JSIOCGBTNMAP, mapping, True)
                    button = mapping.index(BTN_SELECT)
                except (OSError, ValueError):
                    os.close(fd)
                    continue
                self.devices[path] = (fd, button)

        presses = 0
        for path, (fd, button) in list(self.devices.items()):
            try:
                events = os.read(fd, JS_EVENT.size * 128)
            except BlockingIOError:
                continue
            except OSError:
                events = b""
            if not events:
                os.close(fd)
                del self.devices[path]
                continue
            for _, value, kind, index in JS_EVENT.iter_unpack(events):
                # Linux sends one button event per edge. Ignore initial state packets (0x81).
                if kind == 0x01 and value == 1 and index == button:
                    presses += 1
        return presses

    def close(self):
        for fd, _ in self.devices.values():
            os.close(fd)
        self.devices.clear()


@contextmanager
def viewer_camera(follow, ego_camera):
    import mujoco
    import mujoco.viewer

    original_launch = mujoco.viewer.launch_passive
    buttons = SelectButtons()
    requested = SimpleQueue()

    def launch(model, data, *, key_callback=None, **kwargs):
        torso = model.body("torso_link").id
        chase = model.camera(CHASE_CAMERA).id
        ego = model.camera(ego_camera).id
        tracking = mujoco.mjtCamera.mjCAMERA_TRACKING
        fixed = mujoco.mjtCamera.mjCAMERA_FIXED

        def handle_key(key):
            if key == mujoco.glfw.glfw.KEY_V:
                requested.put(None)
            elif key_callback is not None:
                key_callback(key)

        viewer = original_launch(model, data, key_callback=handle_key, **kwargs)
        original_sync = viewer.sync

        def set_view(index):
            cam = viewer.cam
            if index == 0:
                cam.type = tracking
                cam.trackbodyid = torso
                cam.lookat[:] = data.subtree_com[torso]
                cam.distance = 3.0
                cam.azimuth = 135
                cam.elevation = -20
            else:
                cam.type = fixed
                cam.fixedcamid = (chase, ego)[index - 1]

        def sync(*args, **kwargs):
            presses = buttons.poll()
            while not requested.empty():
                requested.get_nowait()
                presses += 1
            if presses:
                with viewer.lock():
                    cam = viewer.cam
                    current = -1
                    if cam.type == tracking and cam.trackbodyid == torso:
                        current = 0
                    elif cam.type == fixed and cam.fixedcamid in (chase, ego):
                        current = 1 if cam.fixedcamid == chase else 2
                    set_view((current + presses) % 3)
            return original_sync(*args, **kwargs)

        if follow:
            with viewer.lock():
                set_view(0)
        viewer.sync = sync
        cleanup.callback(delattr, viewer, "sync")
        print("Viewer camera: Select/Back (View) or V cycles follow, chase, and first person.", flush=True)
        return viewer

    with ExitStack() as cleanup:
        cleanup.callback(buttons.close)
        cleanup.callback(setattr, mujoco.viewer, "launch_passive", original_launch)
        mujoco.viewer.launch_passive = launch
        yield
