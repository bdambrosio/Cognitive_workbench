#!/usr/bin/env bash
#
# System Health Collector for Cognitive Workbench
#
# Collects hardware and OS health metrics, outputs structured JSON to stdout.
# Designed to be called synchronously by the check-health Python tool.
#
# Signals collected:
#   - GPU: temperature, VRAM, power draw, utilization (via nvidia-smi)
#   - CPU: temperature, load averages, core count
#   - RAM: used/total/available, swap
#   - Disk: usage % for key mount points
#   - Network: default route, DNS resolution
#   - Processes: Zenoh, FastAPI, resource browser liveness
#

set -uo pipefail

# ---- Thresholds ----
GPU_TEMP_WARN=85
GPU_TEMP_CRIT=92
GPU_VRAM_WARN_PCT=90
CPU_TEMP_WARN=90
CPU_TEMP_CRIT=100
DISK_WARN_PCT=85
DISK_CRIT_PCT=95
RAM_WARN_PCT=90
SWAP_WARN_PCT=80

# ---- Helpers ----
json_str()  { printf '"%s"' "$(echo "$1" | sed 's/"/\\"/g')"; }
json_num()  { printf '%s' "${1:-0}"; }
json_bool() { if [[ "$1" == "true" ]]; then printf 'true'; else printf 'false'; fi; }

classify() {
    # classify value warn_threshold crit_threshold
    local val="$1" warn="$2" crit="${3:-}"
    if [[ -n "$crit" ]] && (( $(echo "$val >= $crit" | bc -l 2>/dev/null || echo 0) )); then
        echo "critical"
    elif (( $(echo "$val >= $warn" | bc -l 2>/dev/null || echo 0) )); then
        echo "warning"
    else
        echo "ok"
    fi
}

# ---- GPU ----
collect_gpu() {
    if ! command -v nvidia-smi &>/dev/null; then
        echo '{"available": false}'
        return
    fi

    # Query CSV: temp, vram_used, vram_total, power_draw, power_limit, utilization
    local csv
    csv=$(nvidia-smi --query-gpu=temperature.gpu,memory.used,memory.total,power.draw,power.limit,utilization.gpu \
          --format=csv,noheader,nounits 2>/dev/null | head -1)

    if [[ -z "$csv" ]]; then
        echo '{"available": false}'
        return
    fi

    IFS=',' read -r temp vram_used vram_total power_draw power_limit util_pct <<< "$csv"
    # Trim whitespace
    temp="${temp// /}"; vram_used="${vram_used// /}"; vram_total="${vram_total// /}"
    power_draw="${power_draw// /}"; power_limit="${power_limit// /}"; util_pct="${util_pct// /}"

    local vram_pct=0
    if [[ "${vram_total:-0}" != "0" ]]; then
        vram_pct=$(echo "scale=1; $vram_used * 100 / $vram_total" | bc -l 2>/dev/null || echo 0)
    fi

    local temp_status; temp_status=$(classify "$temp" "$GPU_TEMP_WARN" "$GPU_TEMP_CRIT")
    local vram_status; vram_status=$(classify "$vram_pct" "$GPU_VRAM_WARN_PCT")

    cat <<EOJSON
{
  "available": true,
  "temperature_c": $(json_num "$temp"),
  "temperature_status": $(json_str "$temp_status"),
  "vram_used_mib": $(json_num "$vram_used"),
  "vram_total_mib": $(json_num "$vram_total"),
  "vram_used_pct": $(json_num "$vram_pct"),
  "vram_status": $(json_str "$vram_status"),
  "power_draw_w": $(json_num "$power_draw"),
  "power_limit_w": $(json_num "$power_limit"),
  "utilization_pct": $(json_num "$util_pct")
}
EOJSON
}

# ---- CPU ----
collect_cpu() {
    local nproc_val
    nproc_val=$(nproc 2>/dev/null || echo 1)

    # Load averages
    local load1 load5 load15
    read -r load1 load5 load15 _ < /proc/loadavg 2>/dev/null || { load1=0; load5=0; load15=0; }

    # CPU temperature (try hwmon, then thermal_zone)
    local cpu_temp=0
    local temp_file
    # Try coretemp / k10temp hwmon
    for temp_file in /sys/class/hwmon/hwmon*/temp1_input; do
        if [[ -r "$temp_file" ]]; then
            local raw; raw=$(cat "$temp_file" 2>/dev/null || echo 0)
            cpu_temp=$(( raw / 1000 ))
            break
        fi
    done
    # Fallback: thermal zone
    if [[ "$cpu_temp" == "0" ]] && [[ -r /sys/class/thermal/thermal_zone0/temp ]]; then
        local raw; raw=$(cat /sys/class/thermal/thermal_zone0/temp 2>/dev/null || echo 0)
        cpu_temp=$(( raw / 1000 ))
    fi

    local load_status="ok"
    if (( $(echo "$load5 > $nproc_val * 2" | bc -l 2>/dev/null || echo 0) )); then
        load_status="warning"
    elif (( $(echo "$load5 > $nproc_val" | bc -l 2>/dev/null || echo 0) )); then
        load_status="ok"
    fi

    local temp_status; temp_status=$(classify "$cpu_temp" "$CPU_TEMP_WARN" "$CPU_TEMP_CRIT")

    cat <<EOJSON
{
  "cores": $(json_num "$nproc_val"),
  "load_1m": $(json_num "$load1"),
  "load_5m": $(json_num "$load5"),
  "load_15m": $(json_num "$load15"),
  "load_status": $(json_str "$load_status"),
  "temperature_c": $(json_num "$cpu_temp"),
  "temperature_status": $(json_str "$temp_status")
}
EOJSON
}

# ---- RAM ----
collect_ram() {
    local mem_total mem_avail mem_used swap_total swap_free
    mem_total=$(awk '/^MemTotal:/ {print $2}' /proc/meminfo 2>/dev/null || echo 0)
    mem_avail=$(awk '/^MemAvailable:/ {print $2}' /proc/meminfo 2>/dev/null || echo 0)
    swap_total=$(awk '/^SwapTotal:/ {print $2}' /proc/meminfo 2>/dev/null || echo 0)
    swap_free=$(awk '/^SwapFree:/ {print $2}' /proc/meminfo 2>/dev/null || echo 0)

    mem_used=$(( mem_total - mem_avail ))
    local swap_used=$(( swap_total - swap_free ))

    local ram_pct=0 swap_pct=0
    if [[ "$mem_total" != "0" ]]; then
        ram_pct=$(echo "scale=1; $mem_used * 100 / $mem_total" | bc -l 2>/dev/null || echo 0)
    fi
    if [[ "${swap_total:-0}" != "0" ]]; then
        swap_pct=$(echo "scale=1; $swap_used * 100 / $swap_total" | bc -l 2>/dev/null || echo 0)
    fi

    # Convert to MiB for readability
    local total_mib=$(( mem_total / 1024 ))
    local used_mib=$(( mem_used / 1024 ))
    local avail_mib=$(( mem_avail / 1024 ))
    local swap_total_mib=$(( swap_total / 1024 ))
    local swap_used_mib=$(( swap_used / 1024 ))

    local ram_status; ram_status=$(classify "$ram_pct" "$RAM_WARN_PCT")
    local swap_status="ok"
    if [[ "${swap_total:-0}" != "0" ]]; then
        swap_status=$(classify "$swap_pct" "$SWAP_WARN_PCT")
    fi

    cat <<EOJSON
{
  "total_mib": $(json_num "$total_mib"),
  "used_mib": $(json_num "$used_mib"),
  "available_mib": $(json_num "$avail_mib"),
  "used_pct": $(json_num "$ram_pct"),
  "ram_status": $(json_str "$ram_status"),
  "swap_total_mib": $(json_num "$swap_total_mib"),
  "swap_used_mib": $(json_num "$swap_used_mib"),
  "swap_used_pct": $(json_num "$swap_pct"),
  "swap_status": $(json_str "$swap_status")
}
EOJSON
}

# ---- Disk ----
collect_disk() {
    echo '['
    local first=true
    # Size and available, not just the percentage. A reader given "92%,
    # warning" and nothing else cannot tell 141 GB free on a 1.8 TB volume
    # from 4 GB free on a 50 GB one, and will fill the gap with a guess —
    # which is exactly what happened on 2026-08-21, where a roomy disk was
    # written up as a hard blocker on a project plan.
    df -PBG 2>/dev/null | awk 'NR>1 && $1 ~ /^\/dev\// {print $6, $5, $2, $4}' \
        | sed 's/%//; s/G//g' | while read -r mount pct size_gb avail_gb; do
        local status; status=$(classify "$pct" "$DISK_WARN_PCT" "$DISK_CRIT_PCT")
        if [[ "$first" == "true" ]]; then
            first=false
        else
            echo ','
        fi
        printf '  {"mount": %s, "used_pct": %s, "size_gb": %s, "available_gb": %s, "status": %s}' \
            "$(json_str "$mount")" "$(json_num "$pct")" "$(json_num "$size_gb")" \
            "$(json_num "$avail_gb")" "$(json_str "$status")"
    done
    echo ''
    echo ']'
}

# ---- Network ----
collect_network() {
    local has_route="false"
    if ip route show default &>/dev/null && [[ -n "$(ip route show default 2>/dev/null)" ]]; then
        has_route="true"
    fi

    local dns_ok="false"
    if host -W 2 dns.google &>/dev/null 2>&1 || nslookup -timeout=2 dns.google &>/dev/null 2>&1; then
        dns_ok="true"
    fi

    local net_status="ok"
    if [[ "$has_route" == "false" || "$dns_ok" == "false" ]]; then
        net_status="warning"
    fi

    cat <<EOJSON
{
  "default_route": $(json_bool "$has_route"),
  "dns_resolution": $(json_bool "$dns_ok"),
  "network_status": $(json_str "$net_status")
}
EOJSON
}

# ---- Processes ----
collect_processes() {
    # Check for key CWB companion processes
    local zenoh_alive="false"
    if pgrep -f "zenoh" &>/dev/null; then
        zenoh_alive="true"
    fi

    local fastapi_alive="false"
    if pgrep -f "fastapi_action_display" &>/dev/null || pgrep -f "uvicorn.*fastapi" &>/dev/null; then
        fastapi_alive="true"
    fi

    local resource_browser_alive="false"
    if pgrep -f "resource_browser" &>/dev/null; then
        resource_browser_alive="true"
    fi

    local proc_status="ok"
    if [[ "$zenoh_alive" == "false" || "$fastapi_alive" == "false" ]]; then
        proc_status="warning"
    fi

    cat <<EOJSON
{
  "zenoh_router": $(json_bool "$zenoh_alive"),
  "fastapi_ui": $(json_bool "$fastapi_alive"),
  "resource_browser": $(json_bool "$resource_browser_alive"),
  "process_status": $(json_str "$proc_status")
}
EOJSON
}

# ---- Main: assemble JSON ----
main() {
    local gpu cpu ram disk network processes
    gpu=$(collect_gpu)
    cpu=$(collect_cpu)
    ram=$(collect_ram)
    disk=$(collect_disk)
    network=$(collect_network)
    processes=$(collect_processes)

    cat <<EOJSON
{
  "timestamp": "$(date -u '+%Y-%m-%dT%H:%M:%SZ')",
  "gpu": $gpu,
  "cpu": $cpu,
  "ram": $ram,
  "disk": $disk,
  "network": $network,
  "processes": $processes
}
EOJSON
}

main
