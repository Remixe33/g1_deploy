check_robot_platform() {
    local platform target_system target_arch target_glibc
    platform="$(. /etc/os-release; printf '%s %s' "$ID" "$VERSION_ID")"
    read -r target_system target_arch target_glibc < <(python3 -c '
import json, sys
with open(sys.argv[1]) as stream:
    platform = json.load(stream)["platform"]
print(platform["ubuntu"], platform["architecture"], platform["glibc"])
' "$1")
    if [[ "$(uname -m)" != "$target_arch" || "$platform" != "ubuntu $target_system" ||
          "$(getconf GNU_LIBC_VERSION)" != "glibc $target_glibc" ]]; then
        echo "This dependency bundle requires Ubuntu $target_system, $target_arch and glibc $target_glibc." >&2
        exit 1
    fi
}

check_robot_apt_plan() {
    local plan="$1" operation package rest
    while read -r operation package rest; do
        case "$operation:$package" in
            Remv:*|Inst:libc6|Inst:libc6:*|Inst:libc-bin|Inst:libc-bin:*|Inst:linux-image*|Inst:linux-headers*|Inst:linux-modules*|Inst:linux-firmware*|Inst:nvidia-*|Inst:cuda-*)
                echo "Refusing system package change: $operation $package $rest" >&2
                return 1 ;;
        esac
    done <<< "$plan"
}
