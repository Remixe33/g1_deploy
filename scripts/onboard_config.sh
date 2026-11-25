read_onboard_config() {
    onboard_dir="$1"
    if [[ "$onboard_dir" == */install && -f "$onboard_dir/../CMakeLists.txt" &&
          -f "$onboard_dir/../g1" ]]; then
        onboard_dir="$(cd -- "$onboard_dir/.." && pwd)"
    fi
    onboard_file="$onboard_dir/.g1-onboard.conf"
    local line
    local assignment='^[[:space:]]*([A-Z_]+)[[:space:]]*=[[:space:]]*([^[:space:]#]+)[[:space:]]*(#.*)?$'
    robot_interface=
    monitor_host=
    monitor_port=
    [[ -f "$onboard_file" ]] || return 0
    while IFS= read -r line || [[ -n "$line" ]]; do
        [[ "$line" =~ ^[[:space:]]*(#.*)?$ ]] && continue
        if [[ "$line" =~ $assignment ]]; then
            case "${BASH_REMATCH[1]}" in
                ROBOT_INTERFACE) robot_interface="${BASH_REMATCH[2]}"; continue ;;
                MONITOR_HOST) monitor_host="${BASH_REMATCH[2]}"; continue ;;
                MONITOR_PORT) monitor_port="${BASH_REMATCH[2]}"; continue ;;
            esac
        fi
        echo "Invalid setting in $onboard_file: $line" >&2
        exit 2
    done < "$onboard_file"
}
