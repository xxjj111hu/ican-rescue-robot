#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="${SCRIPT_DIR}/../logs"
LOG_FILE="${LOG_DIR}/randomize_ground_textures.log"
mkdir -p "${LOG_DIR}" 2>/dev/null || true

on_exit() {
  local status="$?"
  if [[ "$status" -ne 0 ]]; then
    printf '%s [ERROR] wrapper exited with status %s\n' "$(date '+%Y-%m-%d %H:%M:%S')" "$status" >>"${LOG_FILE}" 2>/dev/null || true
    printf '详细日志：%s\n' "${LOG_FILE}" >&2
  fi
}
trap on_exit EXIT

# Resolve the installed simulator through the workspace environment instead
# of embedding a user's home directory or a fixed /opt path.
set +u
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/setup_env.sh"
set -u
exec python3 "${SCRIPT_DIR}/randomize_ground_textures.py" "$@"
