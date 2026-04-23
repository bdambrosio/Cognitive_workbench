# `body/status` — optional `host` object (desktop consumer spec)

**Audience:** Body operator / desktop dev console implementers.  
**Normative wire:** [body_project_spec.md §5.9](body_project_spec.md) (status message); this document only details the **`host`** extension.

---

## When it appears

The watchdog may include a top-level **`host`** object on each `body/status` sample (same 1 Hz cadence as status, unless `watchdog.status_publish_hz` changes).

- **Present** when `watchdog.host_metrics` is `true` in Pi `config.json` (default).
- **Absent** when `host_metrics` is `false` — desktop must treat missing `host` as normal.

---

## Purpose

Surface **machine health** hints for field debugging (thermal, SoC power rail, Pi firmware “throttled” flags). This is **diagnostic**, not a safety interlock.

---

## 5 V input vs `core_volts` (common confusion)

- Values like **~0.84 V** for **`core_volts`** are **normal**: that field is the **SoC “core” logic rail** from `vcgencmd measure_volts core`, not the **5 V USB** supply.
- The Pi does **not** expose a reliable **“5 V input voltage”** number through `vcgencmd` or sysfs on standard boards. To **detect 5 V sag in software**, use **`under_voltage_now`** and **`under_voltage_occurred`** (from `get_throttled`): the PMIC/firmware asserts these when the **input** is out of spec.
- For a **numeric** 5 V rail reading, use external hardware (multimeter, INA219, dedicated power monitor HAT, etc.).

---

## JSON shape

`host` is a flat object (all fields optional except where noted). Clients should **ignore unknown keys** for forward compatibility.

| Field | Type | Meaning |
|-------|------|--------|
| `ts` | number | Unix time when metrics were read (seconds). Always present when `host` is present. |
| `cpu_temp_c` | number \| omitted | CPU temperature in °C from `/sys/class/thermal/thermal_zone0/temp` when available. |
| `core_volts` | number \| omitted | SoC **core** rail (~0.8–0.95 V typical), from `vcgencmd measure_volts core`. **Not** 5 V input; do not use it to infer USB sag (see section above). |
| `throttled` | string \| omitted | Raw `get_throttled` value, e.g. `"0x50000"`. |
| `under_voltage_now` | boolean \| omitted | Bit 0 of throttled register — under-voltage **currently** detected. |
| `arm_freq_capped_now` | boolean \| omitted | Bit 1 — ARM frequency capped now. |
| `throttled_now` | boolean \| omitted | Bit 2 — active thermal throttle now. |
| `soft_temp_limit_now` | boolean \| omitted | Bit 3 — soft temperature limit active now. |
| `under_voltage_occurred` | boolean \| omitted | Bit 16 — under-voltage has occurred since last boot. |
| `arm_freq_capped_occurred` | boolean \| omitted | Bit 17. |
| `throttled_occurred` | boolean \| omitted | Bit 18. |
| `soft_temp_limit_occurred` | boolean \| omitted | Bit 19. |

### Example

```json
{
  "ts": 1713264000.12,
  "processes": { "motor_controller": "ok", "lidar_driver": "ok", "oakd_driver": "ok" },
  "heartbeat_ok": true,
  "e_stop_active": false,
  "uptime_s": 123.4,
  "host": {
    "ts": 1713264000.12,
    "cpu_temp_c": 52.3,
    "core_volts": 0.8563,
    "throttled": "0x0",
    "under_voltage_now": false,
    "arm_freq_capped_now": false,
    "throttled_now": false,
    "soft_temp_limit_now": false,
    "under_voltage_occurred": false,
    "arm_freq_capped_occurred": false,
    "throttled_occurred": false,
    "soft_temp_limit_occurred": false
  }
}
```

On non-Pi Linux or when `vcgencmd` is missing, `host` may contain only `ts` and `cpu_temp_c` (or sparser).

---

## UI guidance (non-normative)

- Treat **`under_voltage_now`** / **`under_voltage_occurred`** as the primary **“is my 5 V input bad?”** indicators; label them clearly (e.g. “Input power: undervoltage”).
- Show **`cpu_temp_c`** when present.
- If showing **`core_volts`**, label it **“SoC core (not 5 V)”** or similar so operators do not read ~0.84 V as a sagging main supply.
- Highlight **`throttled_now`** / **`throttled_occurred`** when thermal limiting matters.
- If `host` is missing, hide the panel or show “host metrics disabled”.

---

## References

- [body_project_spec.md §5.9](body_project_spec.md) — full `body/status` schema.
- Implementation: `body/lib/host_metrics.py`, `body/watchdog.py`, `watchdog.host_metrics` in `config.json`.
