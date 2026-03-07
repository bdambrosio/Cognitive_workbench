from pathlib import Path
import importlib.util
import sys
import types


ROOT = Path(__file__).resolve().parent.parent
SRC = ROOT / "src"
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))


plan_guidance_stub = types.ModuleType("plan_guidance")
plan_guidance_stub.PlanGuidance = type("PlanGuidance", (), {})
sys.modules.setdefault("plan_guidance", plan_guidance_stub)

tool_model_stub = types.ModuleType("tool_model")
tool_model_stub.ToolModel = type("ToolModel", (), {})
sys.modules.setdefault("tool_model", tool_model_stub)

world_model_stub = types.ModuleType("world_model")
world_model_stub.WORLD_MODEL_SCHEMA = {}
world_model_stub.empty_world_model = lambda: {}
sys.modules.setdefault("world_model", world_model_stub)


from incremental_planner import (  # noqa: E402
    _allow_llm_eval_target_override,
    _select_primary_artifact_id,
)
from infospace_executor import InfospaceExecutor  # noqa: E402
from task_manager import STEP_READY, TaskManager  # noqa: E402


class DummyResourceManager:
    def __init__(self, resources=None, named_notes=None, named_collections=None):
        self.resources = resources or {}
        self.named_notes = named_notes or {}
        self.named_collections = named_collections or {}
        self._note_counter = 0

    def _resolve_resource_id(self, name_or_id):
        if name_or_id in self.resources:
            return name_or_id
        return self.named_notes.get(name_or_id) or self.named_collections.get(name_or_id)

    def get_resource(self, name_or_id):
        rid = self._resolve_resource_id(name_or_id)
        return self.resources.get(rid)

    def create_note(self, character_name, content, format_type, source_skill, source_value, note_name, extra_props):
        self._note_counter += 1
        note_id = f"Note_{self._note_counter}"
        self.resources[note_id] = {
            "id": note_id,
            "properties": {
                "content": content,
                "source_skill": source_skill,
                "source_value": source_value,
                "note_name": note_name,
                **(extra_props or {}),
            },
        }
        return True, note_id, None, None


class DummyPlannerExecutor:
    def __init__(self, bindings=None, resource_manager=None):
        self.plan_bindings = [bindings or {}]
        self.resource_manager = resource_manager or DummyResourceManager()

    @property
    def plan_bindings_flat(self):
        return self.plan_bindings[-1]


class DummyTrackedExecutor:
    def __init__(self, prior_success):
        self._plan_actions = []
        self._side_effect_cache = {}
        self._successful_side_effect_results = {"send-email": prior_success}
        self._done_gate_retry_active = True
        self.plan_bindings = [{}]
        self.resource_manager = DummyResourceManager(
            resources={
                prior_success["resource_id"]: {
                    "id": prior_success["resource_id"],
                    "properties": {"content": prior_success["value"], "source_skill": "send-email"},
                }
            }
        )
        self.executive_node = None

    @property
    def plan_bindings_flat(self):
        return self.plan_bindings[-1]

    def _bind_variable(self, var_name, resource_id):
        self.plan_bindings[-1][var_name] = resource_id

    def execute_action(self, action):
        raise AssertionError("execute_action should not run when done-gate retry blocks the side effect")

    def _create_uniform_return(self, status, value=None, reason=None, extra=None, resource_id=None):
        return {
            "type": "uniform_return",
            "status": status,
            "value": value,
            "reason": reason,
            "extra": extra,
            "resource_id": resource_id,
        }


class DummyEmailExecutor:
    def __init__(self):
        self._tool_dedup_cache = {}

    def _create_uniform_return(self, status, value=None, reason=None, extra=None, resource_id=None):
        return {
            "type": "uniform_return",
            "status": status,
            "value": value,
            "reason": reason,
            "extra": extra,
            "resource_id": resource_id,
        }


def _load_send_email_module():
    tool_path = SRC / "tools" / "send-email" / "tool.py"
    spec = importlib.util.spec_from_file_location("test_send_email_tool", tool_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_task_manager_preserves_declared_outputs_and_stores_resolved_outputs():
    task = {
        "task_id": "task_1",
        "abstract_plan": [{"step": 1, "status": STEP_READY, "description": "make report", "output_artifacts": ["report"]}],
        "current_step": 1,
        "artifacts": [],
    }
    manager = TaskManager(resource_manager=None, executor=None, character_name="Jill")
    manager.get_task = lambda task_id: task
    manager.update_task = lambda task_id, **updates: task.update(updates) or task

    manager.complete_current_step(
        "task_1",
        outcome="done",
        output_artifacts=["Note_101"],
        output_artifact_names={"report": "Note_101"},
    )

    step = task["abstract_plan"][0]
    assert step["output_artifacts"] == ["report"]
    assert step["resolved_output_artifacts"] == ["Note_101"]
    assert step["output_artifact_names"] == {"report": "Note_101"}


def test_primary_artifact_prefers_declared_report_over_email_confirmation():
    resources = {
        "Note_report": {"id": "Note_report", "properties": {"content": "report", "source_skill": "generate-note"}},
        "Note_email": {"id": "Note_email", "properties": {"content": "Sent to x", "source_skill": "send-email"}},
    }
    executor = DummyPlannerExecutor(
        bindings={"report": "Note_report", "email_conf": "Note_email"},
        resource_manager=DummyResourceManager(resources=resources),
    )

    artifact_id = _select_primary_artifact_id(
        executor,
        declared_output_artifacts=["report"],
        resolved_output_artifacts=[],
        output_artifact_names={},
    )

    assert artifact_id == "Note_report"


def test_llm_eval_target_override_rejects_nonexplicit_email_confirmation():
    resources = {
        "Note_email": {"id": "Note_email", "properties": {"content": "Sent to x", "source_skill": "send-email"}},
    }
    executor = DummyPlannerExecutor(
        bindings={"email_conf": "Note_email"},
        resource_manager=DummyResourceManager(resources=resources),
    )

    allowed = _allow_llm_eval_target_override(
        raw_reference="$email_conf",
        resource_id="Note_email",
        executor=executor,
        declared_output_artifacts=["report"],
        resolved_output_artifacts=[],
        output_artifact_names={},
    )

    assert allowed is False


def test_done_gate_retry_blocks_repeat_send_and_rebinds_output_variable():
    prior_success = {
        "type": "uniform_return",
        "status": "success",
        "value": "Sent to bruce@example.com: Test",
        "reason": None,
        "extra": {"message_id": "abc"},
        "resource_id": "Note_200",
    }
    executor = DummyTrackedExecutor(prior_success)

    result = InfospaceExecutor.execute_action_tracked(
        executor,
        {"type": "send-email", "target": "$report", "to": "bruce@example.com", "subject": "Test", "out": "$email_retry"},
        "codegen",
    )

    assert result == prior_success
    assert executor.plan_bindings_flat["email_retry"] == "Note_200"


def test_send_email_uses_executor_dedup_cache_across_tool_reload(monkeypatch):
    module = _load_send_email_module()
    executor = DummyEmailExecutor()
    resource_manager = DummyResourceManager()
    send_count = {"value": 0}

    class DummySMTP:
        def __init__(self, host, port):
            self.host = host
            self.port = port

        def __enter__(self):
            return self

        def __exit__(self, exc_type, exc, tb):
            return False

        def starttls(self, context=None):
            return None

        def login(self, username, password):
            return None

        def sendmail(self, from_addr, to_addrs, message):
            send_count["value"] += 1
            return {}

    monkeypatch.setenv("GMAIL_ADDRESS", "sender@example.com")
    monkeypatch.setenv("GMAIL_APP_PASSWORD", "app-password")
    monkeypatch.setattr(module.smtplib, "SMTP", DummySMTP)

    first = module.tool(
        "hello world",
        executor=executor,
        resource_manager=resource_manager,
        agent_name="Jill",
        to="bruce@example.com",
        subject="Artifact test",
    )
    module._dedup_cache.clear()
    second = module.tool(
        "hello world",
        executor=executor,
        resource_manager=resource_manager,
        agent_name="Jill",
        to="bruce@example.com",
        subject="Artifact test",
    )

    assert first["status"] == "success"
    assert second["status"] == "success"
    assert send_count["value"] == 1
    assert second["value"] == first["value"]
