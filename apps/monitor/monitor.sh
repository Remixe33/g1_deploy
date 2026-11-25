#!/usr/bin/env bash
set -euo pipefail
usage() {
    cat <<'HELP'
Usage: ./g1 monitor [--interface IFACE] [--host IP] [--port PORT]
                    [--joints-only | --source none|realsense|mock|sim]
                    [--camera-config JSON] [--rate HZ] [--record] [--record-dir DIR]
       ./g1 monitor IFACE [HOST] [PORT] [options]

Uses DDS domain 0. Options override .g1-onboard.conf. Without a positional
interface, the default is joints + D435 depth. Use --camera-config to select
the depth profile. The legacy positional form defaults to joints only.
HTTP defaults to 127.0.0.1:8765. Use --host 0.0.0.0 for direct LAN access.
HELP
}
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ -f "$script_dir/apps/monitor/joint_monitor.py" ]]; then
    root_dir="$script_dir"
else
    root_dir="$(cd -- "$script_dir/../.." && pwd)"
fi
interface=
host=
port=
source_mode=
camera_config=
joints_only=0
legacy=0
extra=()
if [[ -n "${1:-}" && "$1" != -* ]]; then
    legacy=1
    interface="$1"; shift
    if (( $# )) && [[ "$1" != -* ]]; then host="$1"; shift; fi
    if (( $# )) && [[ "$1" != -* ]]; then port="$1"; shift; fi
fi
while (( $# )); do
    option="${1%%=*}"
    case "$1" in
        --help|-h) usage; exit 0 ;;
        --joints-only) joints_only=1; shift; continue ;;
        --record) extra+=(--record); shift; continue ;;
    esac
    case "$option" in
        --interface|--host|--port|--source|--camera-config|--rate|--record-dir|--domain) ;;
        *) echo "Unknown monitor option: $1. See ./g1 monitor --help." >&2; exit 2 ;;
    esac
    if [[ "$1" == *=* ]]; then
        value="${1#*=}"; shift
    else
        value="${2:-}"
        [[ "$value" != --* ]] || value=
        if [[ -z "$value" ]]; then echo "Missing value for $option." >&2; exit 2; fi
        shift 2
    fi
    if [[ -z "$value" ]]; then echo "Missing value for $option." >&2; exit 2; fi
    case "$option" in
        --interface) interface="$value" ;;
        --host) host="$value" ;;
        --port) port="$value" ;;
        --source) source_mode="$value" ;;
        --camera-config) camera_config="$value" ;;
        --domain)
            if [[ "$value" != 0 ]]; then echo 'The hardware monitor uses DDS domain 0.' >&2; exit 2; fi ;;
        *) extra+=("$option" "$value") ;;
    esac
done
source "$root_dir/scripts/onboard_config.sh"
read_onboard_config "$root_dir"
interface="${interface:-$robot_interface}"
if [[ -z "$interface" || "$interface" == -* || "$interface" == */* ||
      ! -d "/sys/class/net/$interface" ]]; then
    echo "Specify an existing DDS interface with --interface or set ROBOT_INTERFACE in $onboard_file. See ./g1 interfaces." >&2
    exit 2
fi
host="${host:-${monitor_host:-127.0.0.1}}"
port="${port:-${monitor_port:-8765}}"
if [[ ! "$port" =~ ^[0-9]{1,5}$ ]] || (( 10#$port < 1 || 10#$port > 65535 )); then
    echo "Set --port (or MONITOR_PORT in $onboard_file) to a port from 1 to 65535." >&2
    exit 2
fi
if (( joints_only )); then
    if [[ -n "$camera_config" || ( -n "$source_mode" && "$source_mode" != none ) ]]; then
        echo '--joints-only cannot be combined with a camera source or config.' >&2
        exit 2
    fi
    source_mode=none
elif [[ -z "$source_mode" ]]; then
    if (( legacy )) && [[ -z "$camera_config" ]]; then source_mode=none; else source_mode=realsense; fi
fi
if [[ "$source_mode" == realsense && -z "$camera_config" ]]; then
    camera_config="$root_dir/profiles/configs/cameras/d435_depth_480x270_60hz.json"
fi
if [[ -n "$camera_config" ]]; then extra=(--camera-config "$camera_config" "${extra[@]}"); fi

deps="$onboard_dir/.deps"
python="${PYTHON_BIN:-}"
if [[ -z "$python" ]]; then
    if [[ "$source_mode" == realsense && -x "$deps/python/bin/python3-realsense" &&
          ! -f "$deps/robot-bundle.sha256" ]]; then
        python="$deps/python/bin/python3-realsense"
    elif [[ -x "$deps/python/bin/python3" ]]; then
        python="$deps/python/bin/python3"
    else
        python=python3
    fi
fi
if [[ -d "$root_dir/.python-deps" ]]; then
    export PYTHONPATH="$root_dir/.python-deps${PYTHONPATH:+:$PYTHONPATH}"
fi
export LD_LIBRARY_PATH="$deps/cyclonedds/lib:$deps/realsense/lib:$deps/runtime/python/lib:$root_dir/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
exec "$python" -u "$root_dir/apps/monitor/joint_monitor.py" \
    --domain 0 --interface "$interface" --host "$host" --port "$port" --source "$source_mode" \
    --record-dir "$onboard_dir/logs/recordings" "${extra[@]}"
