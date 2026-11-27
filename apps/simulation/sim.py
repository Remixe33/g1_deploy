"""Start a profile-driven G1 29DOF MuJoCo environment.

Controllers communicate through DDS on lo. The selected observation pipeline can
publish Parkour state/depth or render native metric depth for the task camera.
The physics engine, base scene and dependencies are vendored in
third_party/unitree_mujoco (source: wangsy1999/unitree_mujoco fork, see ORIGIN.md).
"""
import argparse
from contextlib import ExitStack
import json
import math
import os
from pathlib import Path
import runpy
import sys
import time
from tempfile import TemporaryDirectory


PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT / "apps"))
from depth_stream import DepthStream


def resolve_profile(name):
    path = Path(name)
    if not path.is_absolute() and not path.exists():
        path = PROJECT_ROOT / "profiles" / f"{name}.json"
    if not path.exists():
        raise FileNotFoundError(f"Profile not found: {name}")
    return path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mujoco-root", type=Path,
                        default=PROJECT_ROOT / "third_party" / "unitree_mujoco",
                        help="Simulator root (default: vendored third_party/unitree_mujoco)")
    parser.add_argument("--domain", type=int, default=1)
    parser.add_argument("--depth", action=argparse.BooleanOptionalAction, default=True,
                        help="Enable the selected depth pipeline; --no-depth selects DDS-only simulation")
    parser.add_argument("--depth-preview", action="store_true", default=None,
                        help="Show the pipeline's Raw and optional Model outputs "
                             "(default: on when the task profile carries a camera)")
    parser.add_argument("--fov-debug", action="store_true",
                        help="Draw four depth-camera FOV edge rays (requires --depth-preview)")
    parser.add_argument("--follow", action="store_true",
                        help="Start with a camera tracking the robot torso")
    parser.add_argument("--profile", help="Task profile: name in profiles/ or a JSON path; controls scene, robot and camera defaults")
    parser.add_argument("--terrain", default=None,
                        choices=("original", "mixed", "platforms", "stones", "gaps", "stairs", "hurdles", "doors"),
                        help="Obstacle course; default: original external scene, or the task profile's scene")
    parser.add_argument("--window-backend", choices=("auto", "x11", "wayland"), default="x11",
                        help="Window backend (default: x11); auto prefers X11/XWayland when DISPLAY is available")
    parser.add_argument("--data-web", action=argparse.BooleanOptionalAction, default=True,
                        help="Host the robot data web (joints/IMU/camera/recording) for native-camera profiles")
    parser.add_argument("--data-web-port", type=int, default=8765)
    parser.add_argument("--record", action="store_true",
                        help="Start data-web recording immediately (requires the data web)")
    parser.add_argument("--remote-socket", help="Enable remote observations with this absolute Unix target path")
    parser.add_argument("--remote-port", type=int, default=8770)
    parser.add_argument("--remote-control", choices=("velocity", "joints"), default="velocity")
    args = parser.parse_args()
    profile = None
    if args.profile:
        profile_path = resolve_profile(args.profile)
        profile = json.loads(profile_path.read_text())
        for section, field in (("camera", "config"), ("robot", "config")):
            referenced = profile.get(section, {}).get(field)
            if referenced:
                module_path = Path(referenced)
                if not module_path.is_absolute():
                    module_path = profile_path.parent / module_path
                if not module_path.exists():
                    parser.error(f"Profile {section} config missing: {module_path}")
                profile[section][field] = str(module_path)
                if section == "camera":
                    profile[section]["module"] = json.loads(module_path.read_text())
    profile_pipeline = (profile or {}).get("camera", {}).get("pipeline")
    if not profile_pipeline:
        profile_pipeline = (profile or {}).get("camera", {}).get("module", {}).get("pipeline")
    native_camera = bool(profile) and profile_pipeline == "none"
    depth_stream = DepthStream.from_module(profile["camera"]["module"], simulation=True) if native_camera else None
    profile_has_camera = bool(profile) and bool(profile.get("camera", {}).get("config"))
    data_web_source = None
    if profile_has_camera and args.data_web and args.depth:
        sys.path.insert(0, str(PROJECT_ROOT / "apps" / "monitor"))
        from camera_sources import SimSource
        data_web_source = SimSource()
        data_web_source.set_module(profile["camera"]["module"])
    if native_camera and args.terrain:
        parser.error(f"The native camera follows the generated scene; remove --terrain {args.terrain}")
    profile_terrain = (profile or {}).get("scene", {}).get("terrain")
    if args.terrain and profile_terrain and args.terrain != profile_terrain:
        parser.error(f"--terrain {args.terrain} conflicts with the profile scene '{profile_terrain}'; remove --terrain")
    args.terrain = args.terrain or profile_terrain or "original"
    if args.depth_preview is None:
        args.depth_preview = bool(profile and profile.get("camera", {}).get("config"))
    if args.remote_socket and (not Path(args.remote_socket).is_absolute() or not args.depth or
                               not args.depth_preview or not native_camera or not 1 <= args.remote_port <= 65535):
        parser.error("--remote-socket requires an absolute path, native depth preview and a TCP port in 1..65535")
    if args.domain <= 0:
        parser.error("Use a positive simulation domain")
    if args.depth_preview and not args.depth:
        parser.error("--depth-preview requires depth; remove --no-depth")
    if args.fov_debug and not args.depth_preview:
        parser.error("--fov-debug requires depth-preview")

    root = args.mujoco_root.resolve()
    simulation = root / "simulate_python"
    simulator = simulation / "unitree_mujoco.py"
    scene = root / "unitree_robots/g1/scene_29dof.xml"
    if not simulator.is_file():
        parser.error("Missing external simulate_python/unitree_mujoco.py")
    if not scene.is_file():
        parser.error("Missing external unitree_robots/g1/scene_29dof.xml")

    previous_path, previous_argv = sys.path[:], sys.argv
    previous_backend = os.environ.get("PYGLFW_LIBRARY_VARIANT")
    window_backend = args.window_backend
    if window_backend == "auto":
        window_backend = previous_backend or ("x11" if sys.platform == "linux" and os.environ.get("DISPLAY") else "auto")
    if window_backend != "auto":
        os.environ["PYGLFW_LIBRARY_VARIANT"] = window_backend
    temporary_scene = None
    try:
        sys.path.insert(0, str(simulation))
        sys.path.insert(0, str(Path(__file__).resolve().parent))
        head_back_override = None
        if profile:
            robot_config = profile.get("robot", {}).get("config")
            if robot_config:
                head_back_override = json.loads(Path(robot_config).read_text()).get("head_back_deg")
        head_back = 0.0
        depth_camera = profile["camera"]["module"] if native_camera else None
        if args.terrain != "original":
            from terrain import generate_scene, head_back_deg
            head_back = head_back_deg() if head_back_override is None else float(head_back_override)
            temporary_scene = TemporaryDirectory(prefix="g1-terrain-")
            scene = Path(temporary_scene.name) / "scene.xml"
            sections = generate_scene(root, args.terrain, scene, head_back=head_back_override,
                                      depth_camera=depth_camera)
            for category, start, end, y in sections:
                print(f"Terrain {category}: x={start:.2f}..{end:.2f} m, lane center y={y:.2f} m, along +X", flush=True)
        import config
        from observation_bridge import install_observation_bridge
        from viewer_camera import add_viewer_cameras, viewer_camera

        if temporary_scene is None:
            temporary_scene = TemporaryDirectory(prefix="g1-viewer-")
        viewer_scene = Path(temporary_scene.name) / "viewer_scene.xml"
        ego_camera = None
        if depth_camera:
            ego_camera = (depth_camera["rgb"].get("scene_camera", "rgb_camera") if depth_camera.get("rgb")
                          else depth_camera.get("scene_camera", "depth_camera"))
        ego_camera = add_viewer_cameras(scene, viewer_scene, ego_camera)

        config.ROBOT = "g1"
        config.ROBOT_SCENE = str(viewer_scene)
        config.PHYSICS_BACKEND = "mujoco"
        config.DOMAIN_ID, config.INTERFACE = args.domain, "lo"
        config.USE_JOYSTICK = 0
        if not args.depth:
            depth_description = "off"
        elif native_camera:
            module = profile["camera"]["module"]
            depth_description = (f"native Raw{'+RGB' if module.get('rgb') else ''} "
                                 f"{depth_stream.width}x{depth_stream.height}@{depth_stream.fps:g}")
        else:
            depth_description = "Parkour Raw/Model"
        print(f"Shared G1 simulation: DDS domain {args.domain}, interface lo, depth {depth_description}"
              + (f", head back {head_back:g} deg" if head_back else "")
              + (f", profile {args.profile}" if args.profile else "")
              + (f", data web :{args.data_web_port}" if data_web_source is not None else "")
              + f", window backend {window_backend}", flush=True)
        sys.argv = [str(simulator)]
        with ExitStack() as extensions:
            bridge = None
            if args.remote_socket:
                sys.path.insert(0, str(PROJECT_ROOT / "apps" / "remote_control"))
                from bridge import ObservationBridge

                width, height = depth_stream.width, depth_stream.height
                focal = height / (2 * math.tan(math.radians(depth_camera["vertical_fov_deg"]) / 2))
                intrinsics = {"fx": focal, "fy": focal, "ppx": width / 2, "ppy": height / 2}
                bridge = ObservationBridge(args.remote_port, args.remote_socket, args.remote_control, depth_stream)
                extensions.callback(bridge.close)
                bridge.start(args.domain, "lo")

            def deposit_frame(frame):
                if bridge is not None:
                    bridge.deposit(frame.raw, time.monotonic_ns(), intrinsics)
                if data_web_source is not None:
                    data_web_source.deposit(frame)

            extensions.enter_context(install_observation_bridge(
                config, args.depth, args.depth_preview, native_camera,
                profile.get("camera", {}).get("config") if profile else None, head_back,
                frame_sink=deposit_frame if bridge is not None or data_web_source is not None else None))
            if data_web_source is not None:
                from joint_monitor import JointMonitor
                data_web = JointMonitor(host="127.0.0.1", port=args.data_web_port, rate_hz=50,
                                        camera_source=data_web_source, camera_module=depth_camera,
                                        record_root="logs/recordings", record_start=args.record)
                extensions.enter_context(data_web)
                data_web.start_subscribers(args.domain, "lo", init_factory=True)
                data_web.start()
                data_web_url = f"http://127.0.0.1:{args.data_web_port}/"
                print(f"Robot data web: {data_web_url} (joints/IMU, depth, "
                      f"record button on the page)", flush=True)
                try:
                    import webbrowser
                    webbrowser.open(data_web_url)
                except Exception:
                    pass  # Headless environments keep the printed link only.
            extensions.enter_context(viewer_camera(args.follow, ego_camera))
            if args.depth_preview:
                from viewer_depth_overlay import install_depth_overlay
                extensions.enter_context(install_depth_overlay(fov_debug=args.fov_debug, native=native_camera))
            runpy.run_path(str(simulator), run_name="__main__")
    finally:
        sys.path[:] = previous_path
        sys.argv = previous_argv
        if previous_backend is None:
            os.environ.pop("PYGLFW_LIBRARY_VARIANT", None)
        else:
            os.environ["PYGLFW_LIBRARY_VARIANT"] = previous_backend
        if temporary_scene is not None:
            temporary_scene.cleanup()


if __name__ == "__main__":
    main()
