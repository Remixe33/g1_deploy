#!/usr/bin/env bash
set -euo pipefail
scripts="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == --help || $# != 3 ]]; then
    echo 'Usage: bash scripts/build_robot_realsense.sh LIBREALSENSE_V2_55_1_SOURCE PYTHON_3_10 OUTPUT_DIR'
    [[ "${1:-}" == --help ]] && exit 0
    exit 2
fi
source "$scripts/robot_system.sh"
check_robot_platform "$scripts/robot-deps-release.json"
realsense_source="$(realpath "$1")"
python="$2"
output="$(realpath -m "$3")"
"$python" -c 'import sys, wheel; assert sys.version_info[:3] == (3, 10, 15)'
scratch="$(mktemp -d)"
trap 'rm -rf -- "$scratch"' EXIT
cmake -S "$realsense_source" -B "$scratch/build" \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/g1-realsense -DCMAKE_INSTALL_LIBDIR=lib \
    -DBUILD_SHARED_LIBS=ON -DBUILD_WITH_CUDA=OFF -DBUILD_GLSL_EXTENSIONS=OFF \
    -DBUILD_EXAMPLES=OFF -DBUILD_GRAPHICAL_EXAMPLES=OFF -DBUILD_TOOLS=OFF \
    -DIMPORT_DEPTH_CAM_FW=OFF -DCHECK_FOR_UPDATES=OFF \
    -DBUILD_PYTHON_BINDINGS=ON -DBUILD_LEGACY_PYBACKEND=ON \
    -DPYTHON_EXECUTABLE="$python" -DPython_EXECUTABLE="$python" -DPYTHON_INSTALL_DIR:STRING=lib/g1-python
cmake --build "$scratch/build" -j"${JOBS:-4}"
DESTDIR="$scratch/install" cmake --install "$scratch/build"
sdk="$scratch/install/g1-realsense"
[[ -f "$sdk/lib/cmake/realsense2/realsense2Config.cmake" && -f "$sdk/lib/librealsense2.so.2.55.1" ]] ||
    { echo 'Expected the librealsense 2.55.1 SDK and CMake configuration.' >&2; exit 1; }
suffix="$("$python" -c 'import sysconfig; print(sysconfig.get_config_var("EXT_SUFFIX"))')"
module="$(find "$scratch/install" -type f -name "pyrealsense2${suffix}*" -print -quit)"
[[ -n "$module" ]] || { echo "Missing built pyrealsense2$suffix." >&2; exit 1; }

wheel_root="$scratch/wheel"
metadata="$wheel_root/pyrealsense2-2.55.1.dist-info"
mkdir -p "$metadata" "$output/realsense" "$output/wheels"
cp -- "$module" "$wheel_root/pyrealsense2$suffix"
cp -- "$realsense_source/LICENSE" "$metadata/LICENSE"
cat > "$metadata/METADATA" <<'METADATA'
Metadata-Version: 2.1
Name: pyrealsense2
Version: 2.55.1
Summary: Python bindings for librealsense built for G1 Focal ARM64
Requires-Python: ==3.10.*
METADATA
cat > "$metadata/WHEEL" <<'WHEEL'
Wheel-Version: 1.0
Root-Is-Purelib: false
Tag: cp310-cp310-linux_aarch64
WHEEL
LD_LIBRARY_PATH="$sdk/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" PYTHONPATH="$wheel_root" \
    "$python" -c 'import pyrealsense2; assert hasattr(pyrealsense2, "pipeline")'
"$python" -m wheel pack "$wheel_root" --dest-dir "$output/wheels"
rm -rf -- "$sdk/lib/g1-python" "$sdk/lib/cmake/pyrealsense2"
cp -a -- "$sdk/." "$output/realsense/"
