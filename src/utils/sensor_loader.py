"""
Sensor loading utility for cognitive workbench.
Scans sensor directories and returns metadata for SensorRunner.
Mirrors the pattern in tool_loader.py.
"""
import json
import logging
from pathlib import Path
from typing import Dict, Optional
import yaml

from utils.tool_loader import parse_yaml_frontmatter

logger = logging.getLogger(__name__)

# Schedule shorthand pattern: digits followed by s/m/h units, repeatable
import re
_SCHEDULE_RE = re.compile(r'^(?:(\d+)h)?(?:(\d+)m)?(?:(\d+)s)?$')


def parse_schedule(schedule_str: str) -> Optional[float]:
    """Parse schedule shorthand into seconds.

    Accepts: "30s", "5m", "1h", "2h30m", "startup", "0s"
    Returns seconds, or 0.0 for one-shot/startup sensors.
    Returns None on parse failure.
    """
    s = schedule_str.strip().lower()
    if s in ('startup', '0s', '0'):
        return 0.0

    m = _SCHEDULE_RE.match(s)
    if not m:
        return None

    hours = int(m.group(1) or 0)
    minutes = int(m.group(2) or 0)
    seconds = int(m.group(3) or 0)
    total = hours * 3600 + minutes * 60 + seconds
    return float(total) if total > 0 else None


def load_sensors(sensors_dir_path: str) -> Dict[str, Dict]:
    """Load all sensors from directory and return metadata dict.

    Args:
        sensors_dir_path: Absolute path to sensors directory

    Returns:
        Dict mapping sensor_name -> metadata dict
    """
    sensors_dir = Path(sensors_dir_path)

    if not sensors_dir.exists():
        logger.info(f"Sensors directory not found: {sensors_dir_path} (no sensors loaded)")
        return {}

    if not sensors_dir.is_dir():
        logger.error(f"Sensors path is not a directory: {sensors_dir_path}")
        return {}

    sensors = {}

    for sensor_dir in sorted(sensors_dir.iterdir()):
        if not sensor_dir.is_dir():
            continue

        # Check for SKILL.md or Skill.md
        skill_path = sensor_dir / "SKILL.md"
        if not skill_path.exists():
            skill_path = sensor_dir / "Skill.md"
        if not skill_path.exists():
            logger.warning(f"No SKILL.md found in sensor {sensor_dir.name}, skipping")
            continue

        with open(skill_path, 'r', encoding='utf-8') as f:
            content = f.read()

        metadata = parse_yaml_frontmatter(content)
        if metadata is None:
            logger.error(f"Failed to parse YAML frontmatter in {skill_path}")
            continue

        sensor_name = metadata.get('name', sensor_dir.name)
        sensor_type = metadata.get('type', 'code')
        description = metadata.get('description', '')
        schedule_str = metadata.get('schedule', '5m')
        gate = metadata.get('gate', None)
        disposition = metadata.get('disposition', 'inform')
        tools = metadata.get('tools', None)
        parameters = metadata.get('parameters', {})

        if sensor_type not in ('plan', 'code'):
            logger.error(f"Sensor {sensor_name} has unknown type '{sensor_type}', skipping")
            continue

        schedule_seconds = parse_schedule(schedule_str)
        if schedule_seconds is None:
            logger.error(f"Sensor {sensor_name} has invalid schedule '{schedule_str}', skipping")
            continue

        # Duplicate check
        if sensor_name in sensors:
            logger.warning(f"Duplicate sensor name '{sensor_name}' in {sensor_dir}, skipping")
            continue

        # Type-specific validation
        plan_data = None
        python_file = None

        if sensor_type == 'plan':
            plan_file = sensor_dir / 'plan.json'
            if not plan_file.exists():
                logger.error(f"Plan sensor {sensor_name} missing plan.json, skipping")
                continue
            with open(plan_file, 'r', encoding='utf-8') as f:
                plan_data = json.load(f)
            if 'plan' not in plan_data:
                logger.error(f"Plan sensor {sensor_name} plan.json missing 'plan' field, skipping")
                continue
            if 'out' not in plan_data:
                logger.error(f"Plan sensor {sensor_name} plan.json missing 'out' field, skipping")
                continue

        elif sensor_type == 'code':
            candidate = sensor_dir / 'sensor.py'
            if not candidate.exists():
                logger.error(f"Code sensor {sensor_name} missing sensor.py, skipping")
                continue
            python_file = str(candidate.absolute())

        sensors[sensor_name] = {
            'name': sensor_name,
            'description': description,
            'type': sensor_type,
            'schedule': schedule_str,
            'schedule_seconds': schedule_seconds,
            'gate': gate,
            'disposition': disposition,
            'tools': tools,
            'parameters': parameters,
            'path': str(sensor_dir.absolute()),
            'plan_data': plan_data,
            'python_file': python_file,
        }
        logger.info(f"Loaded sensor: {sensor_name} (type: {sensor_type}, schedule: {schedule_str})")

    logger.info(f"Loaded {len(sensors)} sensors from {sensors_dir_path}")
    return sensors
