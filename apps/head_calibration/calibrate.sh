#!/usr/bin/env bash
set -euo pipefail
app=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
root=$(cd -- "$app/../.." && pwd -P)
case "${1:-}" in
    sim|real) mode=$1; shift ;;
    *) printf '用法  bash apps/head_calibration/calibrate.sh {sim|real} [选项]\n'; exit 2 ;;
esac
python="${G1_CALIBRATION_PYTHON:-$app/.venv/bin/python}"
if [[ ! -x "$python" ]]; then
    printf '找不到标定环境 %s，请先按 %s/README.md 准备。\n' "$python" "$app" >&2
    exit 1
fi
cd -- "$root"
exec env PYTHONPATH="$root" PYTHONDONTWRITEBYTECODE=1 "$python" -m "apps.head_calibration.$mode" "$@"
