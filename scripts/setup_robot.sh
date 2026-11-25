#!/usr/bin/env bash
set -euo pipefail
scripts="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
root="$(dirname -- "$scripts")"
cache="$root/.deploy-cache/robot-arm64"
lock="$scripts/robot-deps-release.json"
jobs=4
deps_only=0
build_only=0
verify_camera=0
install_system_deps=0
input_mode=cache
input_selected=0
archive=
usage() {
    cat <<'HELP'
Usage: bash scripts/setup_robot.sh [--online | --offline FILE | --cache DIR] [options]
  --online                Download the exact dependency bundle pinned to this release
  --offline FILE          Use the same dependency bundle from a local archive (no network)
  --cache DIR             Use an unpacked local cache (default: .deploy-cache/robot-arm64)
  --jobs N                Parallel build jobs (default: 4)
  --deps-only             Prepare and verify dependencies without building controllers
  --build-only            Build and install from dependencies already prepared in this checkout
  --verify-camera         Test native C++ depth capture; no robot controller is started
  --install-system-deps   Authorize sudo apt-get to install missing packages from bundled .deb files only
  --help                  Show this help
No options means local cache only. See docs/ROBOT_SETUP.md.
No robot motion, system glibc replacement or auto-start is performed.
HELP
}
while (($#)); do
    case "$1" in
        --online|--offline|--cache)
            if (( input_selected )); then echo 'Select only one dependency source.' >&2; exit 2; fi
            input_selected=1
            input_mode="${1#--}"
            if [[ "$input_mode" == online ]]; then
                shift
            else
                [[ $# -ge 2 && -n "$2" && "$2" != --* ]] || { usage >&2; exit 2; }
                if [[ "$input_mode" == offline ]]; then archive="$2"; else cache="$2"; fi
                shift 2
            fi ;;
        --jobs) [[ $# -ge 2 && "$2" =~ ^[1-9][0-9]*$ ]] || { usage; exit 2; }; jobs="$2"; shift 2 ;;
        --deps-only) deps_only=1; shift ;;
        --build-only) build_only=1; shift ;;
        --verify-camera) verify_camera=1; shift ;;
        --install-system-deps) install_system_deps=1; shift ;;
        --help|-h) usage; exit 0 ;;
        *) echo "Unknown option: $1" >&2; usage; exit 2 ;;
    esac
done
if (( build_only && (deps_only || input_selected || install_system_deps) )); then
    echo '--build-only cannot be combined with --deps-only, an input bundle or --install-system-deps.' >&2
    exit 2
fi
for tool in python3 tar sha256sum; do
    command -v "$tool" >/dev/null || { echo "Missing base system tool: $tool." >&2; exit 1; }
done
source "$scripts/robot_system.sh"
check_robot_platform "$lock"
# Do not replace a controller binary while an operator is using it.
python3 - "$root" <<'PY'
import os, pathlib, sys
root = pathlib.Path(sys.argv[1]).resolve()
for process in pathlib.Path('/proc').iterdir():
    if not process.name.isdigit():
        continue
    try:
        executable = pathlib.Path(os.readlink(str(process / 'exe')).removesuffix(' (deleted)') if hasattr(str, 'removesuffix') else os.readlink(str(process / 'exe')).replace(' (deleted)', ''))
    except (OSError, PermissionError):
        continue
    if executable.name in ('g1_dwaq', 'g1_parkour', 'g1_hybrid') and root in executable.parents:
        raise SystemExit('Controller is running (PID {}). Exit it before setup.'.format(process.name))
PY
deps="$root/.deps"
if ((!deps_only)) && [[ -d "$root/install" ]]; then
    differences=()
    while IFS= read -r -d '' path; do
        relative="${path#"$root"/}"
        if [[ -e "$root/install/$relative" ]] && ! cmp -s "$path" "$root/install/$relative"; then
            differences+=("$relative")
        fi
    done < <(find "$root/profiles" "$root/apps/policies/"{dwaq,parkour,hybrid}/config -type f -print0)
    if (( ${#differences[@]} )); then
        printf 'Setup would overwrite differing installed configuration files:\n' >&2
        printf '  %s\n' "${differences[@]}" >&2
        echo 'Compare and merge these files, then back up and move the old install aside before retrying.' >&2
        exit 1
    fi
fi
extract() {
    local archive="$1" destination="$2" sentinel="$3"
    if [[ ! -e "$sentinel" ]]; then
        mkdir -p "$destination"
        tar --no-same-owner --warning=no-timestamp -xzf "$cache/$archive" -C "$destination"
        if [[ ! -e "$sentinel" ]]; then
            echo "Archive $archive did not install $sentinel." >&2
            exit 1
        fi
    fi
}
if ((!build_only)); then
    case "$input_mode" in
        online)
            cache="$(python3 "$scripts/robot_deps.py" --lock "$lock" prepare "$root/.deploy-cache/bundles")" ;;
        offline)
            cache="$(python3 "$scripts/robot_deps.py" --lock "$lock" prepare "$root/.deploy-cache/bundles" --archive "$archive")" ;;
        cache)
            if [[ ! -f "$cache/SHA256SUMS" ]]; then
                echo "Missing local dependency cache: $cache" >&2
                echo 'Use bash scripts/setup_robot.sh --online or bash scripts/setup_robot.sh --offline /path/to/robot-deps.tar.gz.' >&2
                exit 1
            fi
            cache="$(cd -- "$cache" && pwd)"
            if [[ -f "$cache/BUNDLE_VERSION" ]]; then
                python3 "$scripts/robot_deps.py" --lock "$lock" check-cache "$cache"
            else
                (cd -- "$cache" && sha256sum --strict -c SHA256SUMS)
            fi ;;
    esac
    if [[ -f "$cache/BUNDLE_VERSION" ]]; then
        bundle_id="$(sha256sum "$cache/SHA256SUMS" | cut -d ' ' -f 1)"
        if [[ -d "$deps" && -n "$(ls -A -- "$deps")" ]] &&
           [[ ! -f "$deps/robot-bundle.sha256" || "$(<"$deps/robot-bundle.sha256")" != "$bundle_id" ]]; then
            echo "Existing $deps is not from this bundle. Use a clean checkout, or move that directory aside." >&2
            exit 1
        fi
        mapfile -t required < "$scripts/robot-system-packages.txt"
        missing=()
        for package in "${required[@]}"; do
            if [[ "$(dpkg-query -W -f='${db:Status-Status}' "$package" 2>/dev/null)" != installed ]]; then
                missing+=("$package")
            fi
        done
        if (( ${#missing[@]} )); then
            printf 'Missing system packages: %s\n' "${missing[*]}" >&2
            if ((!install_system_deps)); then
                echo 'Rerun with --install-system-deps to authorize installing the bundled packages.' >&2
                exit 1
            fi
            shopt -s nullglob
            debs=("$cache/deb/"*.deb)
            shopt -u nullglob
            if (( !${#debs[@]} )); then echo 'The bundle has no .deb files for this image.' >&2; exit 1; fi
            apt_options=(-o Dir::Etc::sourcelist=/dev/null -o Dir::Etc::sourceparts=- \
                         --no-remove --no-install-recommends install "${debs[@]}" "${missing[@]}")
            if ! plan="$(LC_ALL=C apt-get -s "${apt_options[@]}")"; then
                echo 'Bundled .deb files cannot satisfy this image. Use a bundle prepared for its original system image.' >&2
                exit 1
            fi
            check_robot_apt_plan "$plan"
            sudo apt-get --yes "${apt_options[@]}"
        fi
        mkdir -p "$deps/setup-logs"
        printf '%s\n' "$bundle_id" > "$deps/robot-bundle.sha256"
        extract runtime.tar.gz "$deps/runtime" "$deps/runtime/python/bin/python3"
        extract unitree-sdk2.tar.gz "$deps/unitree-sdk2" "$deps/unitree-sdk2/lib/cmake/unitree_sdk2/unitree_sdk2Config.cmake"
        extract cyclonedds.tar.gz "$deps/cyclonedds" "$deps/cyclonedds/lib/libddsc.so"
        extract onnxruntime.tar.gz "$deps/onnxruntime" "$deps/onnxruntime/lib/libonnxruntime.so"
        extract realsense.tar.gz "$deps/realsense" "$deps/realsense/lib/cmake/realsense2/realsense2Config.cmake"
        export LD_LIBRARY_PATH="$deps/runtime/python/lib:$deps/cyclonedds/lib:$deps/realsense/lib"
        runtime="$deps/runtime/python/bin/python3"
        "$runtime" -c 'import ssl, sys, venv; assert sys.version_info[:3] == (3, 10, 15), "Expected project Python 3.10.15"'
        if [[ ! -x "$deps/python/bin/python3" ]]; then "$runtime" -m venv "$deps/python"; fi
        python="$deps/python/bin/python3"
        PIP_CONFIG_FILE=/dev/null "$python" -m pip --isolated --disable-pip-version-check install \
            --no-index --no-deps --only-binary=:all: "$cache/wheels/"*.whl
        "$python" -m pip --disable-pip-version-check check
        "$python" -c 'import cv2, numpy, pyrealsense2; from unitree_sdk2py.core.channel import ChannelSubscriber; from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_, LowCmd_, IMUState_; print("Monitor and RealSense dependencies OK")'
    else
        if [[ -f "$deps/robot-bundle.sha256" ]]; then
            echo 'Do not mix a legacy cache and a versioned bundle in the same .deps directory.' >&2
            exit 1
        fi
        if (( install_system_deps )); then
            echo 'Legacy caches do not include offline system packages. Use a versioned bundle for a new robot.' >&2
            exit 1
        fi
        mkdir -p "$deps/setup-logs"
        source "$scripts/setup_robot_legacy_python.sh"
    fi
    "$python" -m pip --disable-pip-version-check freeze > "$deps/setup-logs/python-packages.txt"
fi
if ((!deps_only)); then
    for tool in gcc g++ cmake make; do
        command -v "$tool" >/dev/null || { echo "Missing $tool. Run dependency setup first." >&2; exit 1; }
    done
    real_args=(-DCMAKE_PREFIX_PATH="$deps/realsense")
    if [[ -f "$deps/robot-bundle.sha256" ]]; then
        if [[ ! -f "$deps/realsense/lib/cmake/realsense2/realsense2Config.cmake" ]]; then
            echo 'Missing bundled native RealSense SDK. Run dependency setup first.' >&2
            exit 1
        fi
        real_args+=(-Drealsense2_DIR="$deps/realsense/lib/cmake/realsense2")
    fi
    for path in "$deps/onnxruntime/lib/libonnxruntime.so" "$deps/unitree-sdk2/lib/cmake/unitree_sdk2/unitree_sdk2Config.cmake"; do
        [[ -f "$path" ]] || { echo "Missing prepared dependency: $path. Run dependency setup first." >&2; exit 1; }
    done
    cmake -S "$root" -B "$root/build" -DCMAKE_BUILD_TYPE=Release \
        -U ORT_INCLUDE -U ORT_LIBRARY -DG1_REQUIRE_REALSENSE=ON "${real_args[@]}" \
        -DONNXRUNTIME_ROOT="$deps/onnxruntime" \
        -Dunitree_sdk2_DIR="$deps/unitree-sdk2/lib/cmake/unitree_sdk2" \
        -DCMAKE_INSTALL_PREFIX="$root/install"
    cmake --build "$root/build" -j"$jobs"
    if [[ -d "$root/install" ]]; then
        mkdir -p "$root/deployment-backups"
        tar -czf "$root/deployment-backups/install-$(date +%Y%m%d-%H%M%S)-$$.tar.gz" -C "$root" install
    fi
    cmake --install "$root/build"
    (cd "$root/install" && ./g1 dwaq check && ./g1 parkour check && ./g1 hybrid check)
fi
if ((verify_camera)); then
    [[ -x "$root/build/g1_camera_check" ]] || { echo 'Build the native camera check before --verify-camera.' >&2; exit 1; }
    "$root/build/g1_camera_check"
fi
printf '\nSetup complete. No controller was started.\n'
printf 'In %s, copy .g1-onboard.conf.example to .g1-onboard.conf and set ROBOT_INTERFACE.\n' "$root"
printf 'Start with ./install/g1 read-only INTERFACE and ./install/g1 monitor.\n'
