extract cpython-arm64.tar.gz "$deps/runtime" "$deps/runtime/python/bin/python3"
extract unitree-sdk2-arm64.tar.gz "$deps/unitree-sdk2" "$deps/unitree-sdk2/lib/cmake/unitree_sdk2/unitree_sdk2Config.cmake"
extract cyclonedds-0.10.2-arm64.tar.gz "$deps/cyclonedds" "$deps/cyclonedds/include/idl/retcode.h"
extract onnxruntime-1.18.1-arm64.tar.gz "$deps/onnxruntime" "$deps/onnxruntime/lib/libonnxruntime.so"
if [[ ! -e "$deps/glibc-2.35/lib/aarch64-linux-gnu/ld-linux-aarch64.so.1" ]]; then
    mkdir -p "$deps/glibc-2.35"
    dpkg-deb -x "$cache/libc6_2.35-0ubuntu3.15_arm64.deb" "$deps/glibc-2.35"
fi
if [[ ! -x "$deps/python/bin/python3" ]]; then "$deps/runtime/python/bin/python3" -m venv "$deps/python"; fi
python="$deps/python/bin/python3"
"$python" -c 'import sys; assert sys.version_info[:2] == (3, 10), "Expected project Python 3.10"'
if [[ ! -e "$deps/python/lib/libpython3.10.so.1.0" ]]; then
    ln -s ../../runtime/python/lib/libpython3.10.so.1.0 "$deps/python/lib/libpython3.10.so.1.0"
fi
cp "$root/apps/realsense/python3-realsense.sh" "$deps/python/bin/python3-realsense"
chmod +x "$deps/python/bin/python3-realsense"
"$python" -m pip install --no-index --find-links="$cache" \
    'pip==25.0.1' 'setuptools==75.9.1' 'wheel==0.48.0' \
    'numpy==1.26.4' 'rich-click==1.9.9' 'opencv-python-headless==4.11.0.86' 'pyrealsense2==2.55.1.6486'
if ! "$python" -c 'import importlib.metadata as m; assert m.version("cyclonedds")=="0.10.2"' 2>/dev/null; then
    CFLAGS="-I$deps/runtime/python/include/python3.10 ${CFLAGS:-}" \
    CYCLONEDDS_HOME="$deps/cyclonedds" \
        "$python" -m pip install --no-index --find-links="$cache" --no-build-isolation "$cache/cyclonedds-0.10.2.tar.gz" \
        > "$deps/setup-logs/cyclonedds.log" 2>&1 || { tail -n 45 "$deps/setup-logs/cyclonedds.log"; exit 1; }
fi
mkdir -p "$deps/sdk-python"
tar --warning=no-timestamp -xzf "$cache/unitree_sdk2_python.tar.gz" -C "$deps/sdk-python" --strip-components=1
"$python" -m pip install --no-index --no-build-isolation --no-deps "$deps/sdk-python"
LD_LIBRARY_PATH="$deps/cyclonedds/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" "$python" -c \
    'from unitree_sdk2py.core.channel import ChannelSubscriber; from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_,LowCmd_,IMUState_; print("Monitor dependencies OK")'
"$deps/python/bin/python3-realsense" -c 'import cv2,pyrealsense2; print("Camera dependencies and private glibc OK")'
