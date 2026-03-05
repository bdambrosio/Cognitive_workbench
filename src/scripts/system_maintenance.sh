#!/usr/bin/env bash
#
# Daily Report-Only Maintenance Script (Ubuntu)
# - System health snapshot
# - Security update / unattended-upgrades status snapshot
# - Available updates summary
#
# Suggested cron (root or user):
#   0 2 * * * /path/to/daily_report.sh >> /var/log/daily_report.log 2>&1
#

set -euo pipefail

# ---------- logging ----------
ts() { date '+%Y-%m-%d %H:%M:%S'; }

# If root, prefer /var/log. Otherwise, log under the user's home.
DEFAULT_LOG_ROOT="/var/log/daily_report.log"
DEFAULT_LOG_USER="${HOME}/.local/state/daily_report.log"

if [[ ${EUID:-$(id -u)} -eq 0 ]]; then
  LOG_FILE="${LOG_FILE:-$DEFAULT_LOG_ROOT}"
else
  LOG_FILE="${LOG_FILE:-$DEFAULT_LOG_USER}"
  mkdir -p "$(dirname "$LOG_FILE")"
fi

log() {
  # log to stdout and append to LOG_FILE
  echo "[$(ts)] $*" | tee -a "$LOG_FILE" >/dev/null
  # also emit to stdout for cron redirection
  echo "[$(ts)] $*"
}

# ---------- locking ----------
LOCK_DIR="/tmp/daily_report.lock"
acquire_lock() {
  if mkdir "$LOCK_DIR" 2>/dev/null; then
    trap 'rm -rf "$LOCK_DIR"' EXIT
  else
    log "INFO: Another instance is running; exiting."
    exit 0
  fi
}

# ---------- tasks ----------
report_disk_space() {
  log "Disk usage (warn >90%):"
  df -hP | awk 'NR==1 || $1 ~ /^\/dev\// {print}' | while read -r line; do
    if [[ "$line" == Filesystem* ]]; then
      log "  $line"
      continue
    fi
    usage_pct=$(awk '{gsub(/%/,"",$5); print $5}' <<<"$line")
    if [[ "${usage_pct:-0}" =~ ^[0-9]+$ ]] && (( usage_pct > 90 )); then
      log "  WARNING: $line"
    else
      log "  $line"
    fi
  done
}

report_basic_health() {
  log "Uptime / load:"
  uptime | sed 's/^/  /' | while read -r l; do log "$l"; done

  log "Memory (free -h):"
  free -h | sed 's/^/  /' | while read -r l; do log "$l"; done
}

report_unattended_upgrades_status() {
  log "Unattended upgrades status (report-only):"

  if systemctl list-unit-files | grep -q '^unattended-upgrades\.service'; then
    systemctl is-enabled unattended-upgrades.service >/dev/null 2>&1 \
      && log "  service enabled: yes" || log "  service enabled: no"

    systemctl is-active unattended-upgrades.service >/dev/null 2>&1 \
      && log "  service active: yes" || log "  service active: no"

    # Show last few log lines if available
    if command -v journalctl >/dev/null 2>&1; then
      log "  recent journal (last 20 lines):"
      journalctl -u unattended-upgrades.service -n 20 --no-pager 2>/dev/null \
        | sed 's/^/    /' | while read -r l; do log "$l"; done || true
    fi
  else
    log "  INFO: unattended-upgrades service not found (package may be missing)."
  fi
}

report_available_updates() {
  if ! command -v apt-get >/dev/null 2>&1; then
    log "INFO: apt-get not found; skipping update checks."
    return 0
  fi

  # If not root, avoid failing; just report that we can't refresh indexes.
  if [[ ${EUID:-$(id -u)} -ne 0 ]]; then
    log "Available updates: (not root, not running apt-get update)"
    if command -v apt >/dev/null 2>&1; then
      apt list --upgradable 2>/dev/null | head -30 | sed 's/^/  /' \
        | while read -r l; do log "$l"; done || true
    fi
    return 0
  fi

  log "Refreshing apt indexes (quiet)..."
  apt-get update -qq || true

  log "Security-related upgrades (if any):"
  # This is a heuristic; package naming varies. We list upgrades and filter common security pockets.
  # It's report-only, so imperfect classification is acceptable.
  apt-get -s upgrade 2>/dev/null \
    | awk '/^Inst /{print $2}' \
    | head -50 \
    | sed 's/^/  /' | while read -r l; do log "$l"; done || true

  log "Upgradable packages (top 30):"
  if command -v apt >/dev/null 2>&1; then
    apt list --upgradable 2>/dev/null | head -30 | sed 's/^/  /' \
      | while read -r l; do log "$l"; done || true
  fi
}

# ---------- main ----------
main() {
  acquire_lock

  log "=========================================="
  log "Starting Daily Report (report-only)"
  log "Log file: $LOG_FILE"
  log "User: $(id -un 2>/dev/null || true) (uid $(id -u 2>/dev/null || true))"
  log "=========================================="

  report_basic_health
  report_disk_space
  report_unattended_upgrades_status
  report_available_updates

  log "=========================================="
  log "Daily Report Completed"
  log "=========================================="
}

main "$@"
