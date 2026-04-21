"""
Peripheral loading utility for cognitive workbench.
Scans peripheral directories and returns metadata for the peripheral runner.
Mirrors the pattern in sensor_loader.py.
"""
import logging
from pathlib import Path
from typing import Dict

from utils.tool_loader import parse_yaml_frontmatter

logger = logging.getLogger(__name__)


def load_peripherals(peripherals_dir_path: str) -> Dict[str, Dict]:
    """Load all peripherals from directory and return metadata dict.

    Args:
        peripherals_dir_path: Absolute path to peripherals directory

    Returns:
        Dict mapping peripheral_name -> metadata dict with keys:
            name, description, type, config, registered_tools,
            registered_alerts, path, python_file
    """
    peripherals_dir = Path(peripherals_dir_path)

    if not peripherals_dir.exists():
        logger.info(
            f"Peripherals directory not found: {peripherals_dir_path} "
            f"(no peripherals loaded)"
        )
        return {}

    if not peripherals_dir.is_dir():
        logger.error(f"Peripherals path is not a directory: {peripherals_dir_path}")
        return {}

    peripherals: Dict[str, Dict] = {}

    for peripheral_dir in sorted(peripherals_dir.iterdir()):
        if not peripheral_dir.is_dir():
            continue

        skill_path = peripheral_dir / "SKILL.md"
        if not skill_path.exists():
            skill_path = peripheral_dir / "Skill.md"
        if not skill_path.exists():
            logger.warning(
                f"No SKILL.md found in peripheral {peripheral_dir.name}, skipping"
            )
            continue

        with open(skill_path, 'r', encoding='utf-8') as f:
            content = f.read()

        metadata = parse_yaml_frontmatter(content)
        if metadata is None:
            logger.error(f"Failed to parse YAML frontmatter in {skill_path}")
            continue

        peripheral_name = metadata.get('name', peripheral_dir.name)
        peripheral_type = metadata.get('type', 'peripheral')
        description = metadata.get('description', '')
        config = metadata.get('config', {}) or {}
        registered_tools = metadata.get('registered_tools', []) or []
        registered_alerts = metadata.get('registered_alerts', []) or []

        if peripheral_type != 'peripheral':
            logger.error(
                f"Peripheral {peripheral_name} has unexpected type "
                f"'{peripheral_type}' (expected 'peripheral'), skipping"
            )
            continue

        if peripheral_name in peripherals:
            logger.warning(
                f"Duplicate peripheral name '{peripheral_name}' in "
                f"{peripheral_dir}, skipping"
            )
            continue

        candidate = peripheral_dir / 'peripheral.py'
        if not candidate.exists():
            logger.error(
                f"Peripheral {peripheral_name} missing peripheral.py, skipping"
            )
            continue
        python_file = str(candidate.absolute())

        peripherals[peripheral_name] = {
            'name': peripheral_name,
            'description': description,
            'type': peripheral_type,
            'config': config,
            'registered_tools': registered_tools,
            'registered_alerts': registered_alerts,
            'path': str(peripheral_dir.absolute()),
            'python_file': python_file,
        }
        logger.info(f"Loaded peripheral: {peripheral_name}")

    logger.info(
        f"Loaded {len(peripherals)} peripherals from {peripherals_dir_path}"
    )
    return peripherals
