#!/usr/bin/env bash
set -euo pipefail
scripts="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == --help || $# == 0 ]]; then
    cat <<'HELP'
Usage:
  bash scripts/package_robot_deps.sh collect-debs DEB_DIR
  bash scripts/package_robot_deps.sh pack INPUT_DIR OUTPUT_DIR [HTTPS_BUNDLE_URL]

collect-debs runs on an untouched, online target image with matching APT sources.
pack takes the five installed prefixes, deb/, wheels/ and SOURCES.txt.
Omit the URL to create an offline-only bundle.
See docs/ROBOT_SETUP.md for release preparation and validation.
HELP
    exit 0
fi
source "$scripts/robot_system.sh"
check_robot_platform "$scripts/robot-deps-release.json"
case "$1" in
    collect-debs)
        [[ $# == 2 ]] || { echo 'Expected output DEB_DIR.' >&2; exit 2; }
        mapfile -t required < "$scripts/robot-system-packages.txt"
        plan="$(LC_ALL=C apt-get -s --no-install-recommends --no-remove install "${required[@]}")"
        check_robot_apt_plan "$plan"
        mapfile -t packages < <(awk '
            $1 == "Inst" {
                version = ($3 ~ /^\[/ ? $4 : $3)
                sub(/^\(/, "", version)
                print $2 "=" version
            }' <<< "$plan")
        mkdir -p "$2"
        if (( ${#packages[@]} )); then
            (cd -- "$2" && apt-get download "${packages[@]}")
        fi ;;
    pack)
        (( $# == 3 || $# == 4 )) || { echo 'Expected INPUT_DIR OUTPUT_DIR [HTTPS_BUNDLE_URL].' >&2; exit 2; }
        input="$(realpath "$2")"
        output="$(realpath -m "$3")"
        for path in runtime/python/bin/python3 unitree-sdk2/lib/cmake/unitree_sdk2/unitree_sdk2Config.cmake \
            cyclonedds/lib/libddsc.so onnxruntime/lib/libonnxruntime.so onnxruntime/include/onnxruntime_cxx_api.h \
            realsense/lib/cmake/realsense2/realsense2Config.cmake; do
            [[ -f "$input/$path" ]] || { echo "Missing bundle input: $input/$path" >&2; exit 1; }
        done
        for distribution in numpy opencv_python cyclonedds unitree_sdk2py pyrealsense2; do
            compgen -G "$input/wheels/$distribution-*.whl" >/dev/null ||
                { echo "Missing wheel for $distribution in $input/wheels" >&2; exit 1; }
        done
        stage="$(mktemp -d)"
        trap 'rm -rf -- "$stage"' EXIT
        export LD_LIBRARY_PATH="$input/runtime/python/lib:$input/cyclonedds/lib:$input/realsense/lib:$input/onnxruntime/lib"
        for library in "$input/unitree-sdk2/lib/libddsc.so" "$input/unitree-sdk2/lib/libddscxx.so" \
            "$input/onnxruntime/lib/libonnxruntime.so" "$input/realsense/lib/librealsense2.so"; do
            if ! dependencies="$(ldd "$library" 2>&1)" || [[ "$dependencies" == *'not found'* ]]; then
                printf 'Cannot load %s on the target platform:\n%s\n' "$library" "$dependencies" >&2
                exit 1
            fi
        done
        "$input/runtime/python/bin/python3" -c 'import ssl, sys; assert sys.version_info[:3] == (3, 10, 15)'
        "$input/runtime/python/bin/python3" -m venv "$stage/check-env"
        python="$stage/check-env/bin/python"
        PIP_CONFIG_FILE=/dev/null "$python" -m pip --isolated --disable-pip-version-check install \
            --no-index --no-deps --only-binary=:all: "$input/wheels/"*.whl
        "$python" -m pip --disable-pip-version-check check
        "$python" -c 'import cv2, numpy, pyrealsense2; from unitree_sdk2py.core.channel import ChannelSubscriber; from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_, LowCmd_, IMUState_'
        mkdir "$stage/payload"
        for component in runtime unitree-sdk2 cyclonedds onnxruntime realsense; do
            tar -czf "$stage/payload/$component.tar.gz" -C "$input/$component" .
        done
        cp -r -- "$input/deb" "$input/wheels" "$stage/payload/"
        cp -- "$input/SOURCES.txt" "$stage/payload/"
        python3 "$scripts/robot_deps.py" --lock "$scripts/robot-deps-release.json" pack "$stage/payload" "$output" "${@:4}" ;;
    *) echo 'Unknown mode. See --help.' >&2; exit 2 ;;
esac
