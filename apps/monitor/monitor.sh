#!/usr/bin/env bash
# Read-only monitor; works in the source checkout and in the CMake install tree.
# Extra options for joint_monitor.py follow the optional host and port.
set -euo pipefail
if (($# < 1)); then
    echo "Usage: $0 ROBOT_INTERFACE [HOST=127.0.0.1] [PORT=8765] [extra --options]" >&2
    exit 2
fi
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ -f "$script_dir/apps/monitor/joint_monitor.py" ]]; then
    root_dir="$script_dir"
else
    root_dir="$(cd -- "$script_dir/../.." && pwd)"
fi
interface="$1"; shift
[[ -n "$interface" && "$interface" != -* && "$interface" != */* && -d "/sys/class/net/$interface" ]] || {
    echo "Specify an existing network interface." >&2
    exit 2
}
[[ -f "$root_dir/apps/monitor/joint_monitor.py" ]] || { echo "Missing apps/monitor/joint_monitor.py" >&2; exit 1; }
host=127.0.0.1
port=8765
if (( $# )) && [[ "$1" != --* ]]; then host="${1:-$host}"; shift; fi
if (( $# )) && [[ "$1" != --* ]]; then port="${1:-$port}"; shift; fi

python="${PYTHON_BIN:-}"
dependency_roots=("$root_dir/.deps" "$root_dir/../.deps")
if [[ -z "$python" ]]; then
    source=none
    previous=
    for argument in "$@"; do
        if [[ "$previous" == --source ]]; then
            source="$argument"
        elif [[ "$argument" == --source=* ]]; then
            source="${argument#*=}"
        fi
        previous="$argument"
    done
    interpreter_names=(python3)
    if [[ "$source" == realsense ]]; then interpreter_names=(python3-realsense python3); fi
    for name in "${interpreter_names[@]}"; do
        for dependency_root in "${dependency_roots[@]}"; do
            candidate="$dependency_root/python/bin/$name"
            if [[ -x "$candidate" ]]; then python="$candidate"; break 2; fi
        done
    done
    python="${python:-python3}"
fi
if [[ -d "$root_dir/.python-deps" ]]; then
    export PYTHONPATH="$root_dir/.python-deps${PYTHONPATH:+:$PYTHONPATH}"
fi
# Match the native SDK used by the controller, without changing the system config.
dds_library_dir=/usr/local/lib
for dependency_root in "${dependency_roots[@]}"; do
    if [[ -f "$dependency_root/cyclonedds/lib/libddsc.so" ]]; then
        dds_library_dir="$dependency_root/cyclonedds/lib"
        break
    fi
done
export LD_LIBRARY_PATH="$dds_library_dir${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
exec "$python" -u "$root_dir/apps/monitor/joint_monitor.py" \
    --domain 0 --interface "$interface" --host "$host" --port "$port" --rate 50 "$@"
