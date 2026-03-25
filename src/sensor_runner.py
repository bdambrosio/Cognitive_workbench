"""
SensorRunner — per-sensor execution thread.

Each SensorRunner runs as a daemon thread, executing a sensor's plan or code
on a schedule and pushing results to the agent via the Zenoh sense_data channel.
"""
import importlib.util
import json
import logging
import threading
import time
import traceback
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Optional

logger = logging.getLogger(__name__)


class SensorRunner:
    """Autonomous sensor execution thread."""

    def __init__(self,
                 sensor_name: str,
                 sensor_meta: dict,
                 character_name: str,
                 scenario_overrides: dict,
                 resource_manager,
                 zenoh_session,
                 available_tools: dict,
                 shutdown_event: threading.Event,
                 # For plan-type sensors: shared LLM config to copy into isolated executor
                 executor_factory=None):
        """
        Args:
            sensor_name: Sensor identifier
            sensor_meta: Metadata from sensor_loader
            character_name: Owning character name
            scenario_overrides: Per-character overrides from scenario.yaml sensors[] entry
            resource_manager: Shared InfospaceResourceManager (thread-safe reads)
            zenoh_session: Zenoh session for publishing sense_data
            available_tools: Agent's loaded tools dict
            shutdown_event: Shared shutdown event
            executor_factory: Callable() -> InfospaceExecutor for plan-type sensors.
                              Called lazily on first plan execution.
        """
        self.sensor_name = sensor_name
        self.character_name = character_name
        self.zenoh_session = zenoh_session
        self.shutdown_event = shutdown_event

        # Merge: scenario overrides take precedence over SKILL.md defaults
        self.sensor_type = sensor_meta['type']
        self.schedule_seconds = scenario_overrides.get('schedule_seconds', sensor_meta['schedule_seconds'])
        self.gate = scenario_overrides.get('gate', sensor_meta.get('gate'))
        self.disposition = scenario_overrides.get('disposition', sensor_meta.get('disposition', 'inform'))
        self.parameters = {**sensor_meta.get('parameters', {}), **scenario_overrides.get('parameters', {})}

        # Plan-type state
        self.plan_data = sensor_meta.get('plan_data')
        self.executor_factory = executor_factory

        # Code-type state
        self._run_fn = None
        self._gate_fn = None
        if self.sensor_type == 'code':
            self._load_code_sensor(sensor_meta['python_file'])

        # Tool subset filtering
        if sensor_meta.get('tools'):
            self.available_tools = {k: v for k, v in available_tools.items() if k in sensor_meta['tools']}
        else:
            self.available_tools = available_tools

        # Execution tracking
        self.last_run: Optional[datetime] = None
        self.run_count: int = 0

        # Resource manager for gate evaluation
        self.resource_manager = resource_manager

    def _load_code_sensor(self, python_file: str):
        """Import sensor.py and extract run() and optional gate() functions."""
        spec = importlib.util.spec_from_file_location(f"sensor_{self.sensor_name}", python_file)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        self._run_fn = getattr(module, 'run', None)
        if self._run_fn is None:
            raise ValueError(f"Sensor {self.sensor_name}: sensor.py missing run() function")
        self._gate_fn = getattr(module, 'gate', None)

    def _evaluate_gate(self) -> bool:
        """Evaluate gate condition. Returns True if sensor should execute."""
        if self.gate is None:
            return True

        # Code-type sensors with a gate() function
        if self._gate_fn is not None:
            try:
                return bool(self._gate_fn(self._build_context()))
            except Exception as e:
                logger.warning(f"Sensor {self.sensor_name} gate() raised: {e}")
                return False

        # Declarative gate expressions
        if isinstance(self.gate, str):
            if self.gate.startswith('changed:'):
                resource_name = self.gate[len('changed:'):]
                return self._check_resource_changed(resource_name)
            elif self.gate.startswith('nonempty:'):
                resource_name = self.gate[len('nonempty:'):]
                return self._check_resource_nonempty(resource_name)

        logger.warning(f"Sensor {self.sensor_name}: unknown gate expression '{self.gate}', skipping")
        return False

    def _check_resource_changed(self, resource_name: str) -> bool:
        """Check if a resource has been modified since last sensor run."""
        if self.resource_manager is None:
            return True
        resource = self.resource_manager.get_resource_by_name(resource_name)
        if resource is None:
            return False
        modified = getattr(resource, 'modified_at', None) or getattr(resource, 'created_at', None)
        if modified is None:
            return True
        if self.last_run is None:
            return True
        if isinstance(modified, str):
            modified = datetime.fromisoformat(modified)
        return modified > self.last_run

    def _check_resource_nonempty(self, resource_name: str) -> bool:
        """Check if a resource exists and has content."""
        if self.resource_manager is None:
            return False
        resource = self.resource_manager.get_resource_by_name(resource_name)
        if resource is None:
            return False
        content = getattr(resource, 'content', None)
        return bool(content and str(content).strip())

    def _build_context(self) -> dict:
        """Build context dict for code-type sensor run() calls."""
        return {
            'parameters': self.parameters,
            'resource_manager': self.resource_manager,
            'zenoh_session': self.zenoh_session,
            'last_run': self.last_run,
            'run_count': self.run_count,
        }

    def _execute_code(self) -> dict:
        """Execute code-type sensor."""
        context = self._build_context()
        result = self._run_fn(context)
        if not isinstance(result, dict):
            return {'status': 'error', 'content': '', 'metadata': {'error': f'run() returned {type(result).__name__}, expected dict'}}
        return result

    def _execute_plan(self) -> dict:
        """Execute plan-type sensor with an isolated executor."""
        if self.executor_factory is None:
            return {'status': 'error', 'content': '', 'metadata': {'error': 'No executor_factory provided for plan sensor'}}

        executor = self.executor_factory()

        # Bind sensor parameters into the executor's bindings
        for key, value in self.parameters.items():
            executor.plan_bindings[0][f'${key}'] = value

        result = executor.execute_plan_sync(self.plan_data)
        status = result.get('status', 'failed')

        if status == 'success':
            # Extract the output binding
            out_var = self.plan_data.get('out', '')
            bindings = result.get('bindings', {})
            out_resource_id = bindings.get(out_var)
            content = ''
            if out_resource_id and self.resource_manager:
                resource = self.resource_manager.get_resource(out_resource_id)
                if resource:
                    content = getattr(resource, 'content', str(resource))
            return {'status': 'ok', 'content': content, 'metadata': {'resource_id': out_resource_id}}
        else:
            reason = result.get('reason', 'unknown')
            return {'status': 'error', 'content': '', 'metadata': {'error': reason}}

    def _push_to_agent(self, content):
        """Publish sensor result to agent's sense_data channel.

        Args:
            content: Either a plain string or a dict with 'summary' and 'data' keys.
        """
        # Structured content: carry summary + data as-is
        if isinstance(content, dict) and 'summary' in content:
            content_payload = {
                'source': f'sensor:{self.sensor_name}',
                'summary': content['summary'],
                'data': content.get('data', []),
                'disposition': self.disposition,
            }
        else:
            # Legacy plain-text sensors
            prefixed_text = f"sensor {self.sensor_name} report [{self.disposition}]\n{content}"
            content_payload = {
                'source': f'sensor:{self.sensor_name}',
                'text': prefixed_text,
                'disposition': self.disposition,
            }
        sense_data = {
            'timestamp': datetime.now().isoformat(),
            'sequence_id': 0,
            'mode': 'text',
            'content': json.dumps(content_payload),
        }
        try:
            self.zenoh_session.put(
                f"cognitive/{self.character_name}/sense_data",
                json.dumps(sense_data)
            )
            logger.info(f"Sensor {self.sensor_name}: pushed sense_data to {self.character_name}")
        except Exception as e:
            logger.error(f"Sensor {self.sensor_name}: failed to publish: {e}")

    def _push_heartbeat(self):
        """Publish a heartbeat (empty text, filtered by sense_data_callback)."""
        content_payload = {
            'source': f'sensor:{self.sensor_name}',
            'text': '',
            'heartbeat': True,
            'timestamp': datetime.now().isoformat(),
        }
        sense_data = {
            'timestamp': datetime.now().isoformat(),
            'sequence_id': 0,
            'mode': 'text',
            'content': json.dumps(content_payload),
        }
        try:
            self.zenoh_session.put(
                f"cognitive/{self.character_name}/sense_data",
                json.dumps(sense_data)
            )
        except Exception:
            pass

    def run(self):
        """Main sensor loop. Runs until shutdown_event is set."""
        one_shot = self.schedule_seconds == 0.0
        logger.info(f"Sensor {self.sensor_name} started for {self.character_name} "
                     f"(type={self.sensor_type}, interval={self.schedule_seconds}s, one_shot={one_shot})")

        while not self.shutdown_event.is_set():
            # Sleep first (unless one-shot)
            if not one_shot and self.run_count > 0:
                # Sleep in small increments so we can respond to shutdown quickly
                sleep_end = time.time() + self.schedule_seconds
                while time.time() < sleep_end and not self.shutdown_event.is_set():
                    time.sleep(min(1.0, sleep_end - time.time()))
                if self.shutdown_event.is_set():
                    break
            elif not one_shot and self.run_count == 0:
                # First run: small delay to let agent initialize
                if self.shutdown_event.wait(timeout=5.0):
                    break

            # Gate check
            if not self._evaluate_gate():
                self._push_heartbeat()
                if one_shot:
                    break
                continue

            # Execute
            try:
                if self.sensor_type == 'code':
                    result = self._execute_code()
                else:
                    result = self._execute_plan()
            except Exception as e:
                logger.error(f"Sensor {self.sensor_name} execution error: {e}")
                traceback.print_exc()
                result = {'status': 'error', 'content': '', 'metadata': {'error': str(e)}}

            self.last_run = datetime.now()
            self.run_count += 1

            # Push result or heartbeat
            status = result.get('status', 'error')
            if status == 'ok' and result.get('content'):
                self._push_to_agent(result['content'])
            elif status == 'error':
                logger.warning(f"Sensor {self.sensor_name} returned error: {result.get('metadata', {}).get('error', 'unknown')}")
                self._push_heartbeat()
            else:
                # status == 'nothing' or empty content
                self._push_heartbeat()

            if one_shot:
                logger.info(f"Sensor {self.sensor_name}: one-shot complete")
                break

        logger.info(f"Sensor {self.sensor_name} stopped (runs={self.run_count})")
