#!/usr/bin/env python3
"""
Test suite for filesystem (fs) tools.

Validates that:
- fs-list returns a single Note with a text directory listing
- fs-read reads text and JSON files
- fs-head returns line slices
- fs-grep returns matches
- fs-stat returns metadata

Usage:
    python3 tests/test_fs_tools.py --character Jill
    python3 tests/test_fs_tools.py --character Jill --test fs-list

Requirements:
    - Running Jill session with fs world (jill-fs.yaml)
"""

import argparse
import json
import logging
import time
import traceback
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import zenoh
from zenoh import QueryTarget, ConsolidationMode

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)


def execute_plan(session: zenoh.Session, character: str, plan: List[Dict], timeout: float = 60.0) -> Dict:
    start_time = time.time()
    query_key = f"cognitive/{character}/execute_plan_sync"
    payload = json.dumps({"plan": plan})

    try:
        replies = session.get(
            query_key,
            target=QueryTarget.BEST_MATCHING,
            consolidation=ConsolidationMode.NONE,
            payload=payload.encode("utf-8"),
            timeout=timeout
        )
        reply_count = 0
        for reply in replies:
            reply_count += 1
            if hasattr(reply, "ok") and reply.ok is not None:
                return json.loads(reply.ok.payload.to_bytes().decode("utf-8"))

        elapsed = time.time() - start_time
        if elapsed >= timeout:
            return {"success": False, "error": f"Timeout after {elapsed:.2f}s", "timeout": True}
        if reply_count == 0:
            return {"success": False, "error": f"No response (elapsed: {elapsed:.2f}s)", "no_response": True}
        return {"success": False, "error": f"Invalid response (elapsed: {elapsed:.2f}s)"}
    except Exception as e:
        logger.error(f"Exception in execute_plan: {e}")
        logger.error(traceback.format_exc())
        return {"success": False, "error": f"Exception: {str(e)}", "exception": True}


def execute_action(session: zenoh.Session, character: str, action_dict: Dict, timeout: float = 60.0) -> Dict:
    plan = [action_dict]
    result = execute_plan(session, character, plan, timeout)
    if result.get("success"):
        return result.get("last_action_result", {})
    error_reason = result.get("reason") or result.get("error") or "Unknown error"
    error_info = {"status": "failed", "reason": error_reason}
    if "timeout" in result:
        error_info["timeout"] = True
    if "no_response" in result:
        error_info["no_response"] = True
    return error_info


def execute_multi_step_plan(session: zenoh.Session, character: str, plan: List[Dict], timeout: float = 120.0) -> Dict:
    return execute_plan(session, character, plan, timeout)


def get_binding(result: Dict, var_name: str) -> Optional[str]:
    bindings = result.get("bindings", {})
    if isinstance(bindings, list) and len(bindings) > 0:
        bindings = bindings[0]
    if not isinstance(bindings, dict):
        return None
    var_key = var_name.lstrip("$")
    return bindings.get(var_key) or bindings.get(f"${var_key}")


def ensure_fs_fixture() -> Path:
    project_root = Path(__file__).resolve().parent.parent
    fs_root = project_root / "scenarios" / "fs" / "fs"
    fs_root.mkdir(parents=True, exist_ok=True)

    (fs_root / "sample.txt").write_text("alpha\nbeta\ngamma\n", encoding="utf-8")
    (fs_root / "data.json").write_text(json.dumps({"name": "test", "items": [1, 2, 3]}), encoding="utf-8")

    subdir = fs_root / "subdir"
    subdir.mkdir(parents=True, exist_ok=True)
    (subdir / "nested.txt").write_text("nested line one\nsecond line\n", encoding="utf-8")

    logs = fs_root / "logs"
    logs.mkdir(parents=True, exist_ok=True)
    (logs / "app.log").write_text("error: one\nok\nerror: two\n", encoding="utf-8")

    # Debug: verify files were created
    created_files = list(fs_root.glob("*"))
    logger.debug(f"ensure_fs_fixture: Created files in {fs_root}: {[f.name for f in created_files]}")
    
    return fs_root


def test_fs_list(session: zenoh.Session, character: str) -> Tuple[bool, str]:
    print("\n=== Test: fs-list ===")
    fs_root = ensure_fs_fixture()
    # Verify files exist
    files = list(fs_root.glob("*"))
    print(f"  Debug: Files in {fs_root}: {[f.name for f in files]}")
    if len(files) == 0:
        print(f"  ⚠ Warning: No files found in fixture directory {fs_root}")
        return False, "fs-list: fixture files not created"
    result = execute_action(session, character, {"type": "fs-list", "path": ".", "out": "$root"})
    if result.get("status") != "success":
        return False, f"fs-list failed: {result.get('reason', 'unknown')}"
    rid = str(result.get("resource_id", ""))
    if not rid.startswith("Note_"):
        return False, f"fs-list returned unexpected resource_id (expected Note): {result.get('resource_id')}"
    value = result.get("value", "")
    print(f"  ✓ fs-list: {value}")
    if "sample.txt" not in str(value) or "data.json" not in str(value):
        return False, f"fs-list listing missing expected fixture files (value preview): {str(value)[:500]}"
    return True, "fs-list: OK"


def test_fs_read_text(session: zenoh.Session, character: str) -> Tuple[bool, str]:
    print("\n=== Test: fs-read (text) ===")
    fs_root = ensure_fs_fixture()
    sample_file = fs_root / "sample.txt"
    print(f"  Debug: sample.txt exists: {sample_file.exists()}, path: {sample_file}")
    if not sample_file.exists():
        return False, f"fs-read: fixture file not found at {sample_file}"
    result = execute_action(session, character, {"type": "fs-read", "path": "sample.txt", "out": "$note"})
    if result.get("status") != "success":
        reason = result.get("reason", "unknown")
        print(f"  Debug: fs-read failed with reason: {reason}")
        return False, f"fs-read failed: {reason}"
    if not str(result.get("resource_id", "")).startswith("Note_"):
        return False, f"fs-read returned unexpected resource_id: {result.get('resource_id')}"
    plan_result = execute_multi_step_plan(session, character, [
        {"type": "load", "target": "$note", "out": "$loaded"}
    ])
    if not plan_result.get("success"):
        return False, f"load failed: {plan_result.get('error', 'unknown')}"
    value = plan_result.get("last_action_result", {}).get("value", "")
    if "Note Content:" in str(value) and "alpha" in str(value):
        print("  ✓ fs-read text content loaded")
        return True, "fs-read (text): OK"
    return False, f"fs-read text unexpected content: {value}"


def test_fs_read_json(session: zenoh.Session, character: str) -> Tuple[bool, str]:
    print("\n=== Test: fs-read (json) ===")
    fs_root = ensure_fs_fixture()
    data_file = fs_root / "data.json"
    print(f"  Debug: data.json exists: {data_file.exists()}, path: {data_file}")
    if not data_file.exists():
        return False, f"fs-read json: fixture file not found at {data_file}"
    result = execute_action(session, character, {"type": "fs-read", "path": "data.json", "as_json": True, "out": "$json_note"})
    if result.get("status") != "success":
        return False, f"fs-read json failed: {result.get('reason', 'unknown')}"
    if not str(result.get("resource_id", "")).startswith("Note_"):
        return False, f"fs-read json returned unexpected resource_id: {result.get('resource_id')}"
    plan_result = execute_multi_step_plan(session, character, [
        {"type": "load", "target": "$json_note", "out": "$loaded"}
    ])
    if not plan_result.get("success"):
        return False, f"load failed: {plan_result.get('error', 'unknown')}"
    value = plan_result.get("last_action_result", {}).get("value", "")
    if "Note Content:" in str(value) and "items" in str(value):
        print("  ✓ fs-read JSON content loaded")
        return True, "fs-read (json): OK"
    return False, f"fs-read json unexpected content: {value}"


def test_fs_head(session: zenoh.Session, character: str) -> Tuple[bool, str]:
    print("\n=== Test: fs-head ===")
    fs_root = ensure_fs_fixture()
    sample_file = fs_root / "sample.txt"
    print(f"  Debug: sample.txt exists: {sample_file.exists()}, path: {sample_file}")
    if not sample_file.exists():
        return False, f"fs-head: fixture file not found at {sample_file}"
    result = execute_action(session, character, {"type": "fs-head", "path": "sample.txt", "lines": 2, "out": "$head"})
    if result.get("status") != "success":
        return False, f"fs-head failed: {result.get('reason', 'unknown')}"
    if not str(result.get("resource_id", "")).startswith("Note_"):
        return False, f"fs-head returned unexpected resource_id: {result.get('resource_id')}"
    plan_result = execute_multi_step_plan(session, character, [
        {"type": "load", "target": "$head", "out": "$loaded"}
    ])
    if not plan_result.get("success"):
        return False, f"load failed: {plan_result.get('error', 'unknown')}"
    value = plan_result.get("last_action_result", {}).get("value", "")
    if "Note Content:" in str(value) and "alpha" in str(value) and "gamma" not in str(value):
        print("  ✓ fs-head returned first lines")
        return True, "fs-head: OK"
    return False, f"fs-head unexpected content: {value}"


def test_fs_grep(session: zenoh.Session, character: str) -> Tuple[bool, str]:
    print("\n=== Test: fs-grep ===")
    fs_root = ensure_fs_fixture()
    logs_dir = fs_root / "logs"
    app_log = logs_dir / "app.log"
    print(f"  Debug: logs/app.log exists: {app_log.exists()}, path: {app_log}")
    if not app_log.exists():
        return False, f"fs-grep: fixture file not found at {app_log}"
    result = execute_action(session, character, {
        "type": "fs-grep",
        "path": "logs",
        "pattern": "error",
        "context": 0,
        "max_matches": 5,
        "out": "$matches"
    })
    if result.get("status") != "success":
        return False, f"fs-grep failed: {result.get('reason', 'unknown')}"
    if not str(result.get("resource_id", "")).startswith("Collection_"):
        return False, f"fs-grep returned unexpected resource_id: {result.get('resource_id')}"
    print(f"  ✓ fs-grep: {result.get('value')}")
    return True, "fs-grep: OK"


def test_fs_stat(session: zenoh.Session, character: str) -> Tuple[bool, str]:
    print("\n=== Test: fs-stat ===")
    fs_root = ensure_fs_fixture()
    sample_file = fs_root / "sample.txt"
    print(f"  Debug: sample.txt exists: {sample_file.exists()}, path: {sample_file}")
    if not sample_file.exists():
        return False, f"fs-stat: fixture file not found at {sample_file}"
    result = execute_action(session, character, {"type": "fs-stat", "path": "sample.txt", "out": "$stat"})
    if result.get("status") != "success":
        return False, f"fs-stat failed: {result.get('reason', 'unknown')}"
    if not str(result.get("resource_id", "")).startswith("Note_"):
        return False, f"fs-stat returned unexpected resource_id: {result.get('resource_id')}"
    plan_result = execute_multi_step_plan(session, character, [
        {"type": "load", "target": "$stat", "out": "$loaded"}
    ])
    if not plan_result.get("success"):
        return False, f"load failed: {plan_result.get('error', 'unknown')}"
    value = plan_result.get("last_action_result", {}).get("value", "")
    if "Note Content:" in str(value) and "size_bytes" in str(value):
        print("  ✓ fs-stat metadata loaded")
        return True, "fs-stat: OK"
    return False, f"fs-stat unexpected content: {value}"


def run_all_tests(session: zenoh.Session, character: str) -> Dict[str, Tuple[bool, str]]:
    tests = [
        ("fs-list", test_fs_list),
        ("fs-read-text", test_fs_read_text),
        ("fs-read-json", test_fs_read_json),
        ("fs-head", test_fs_head),
        ("fs-grep", test_fs_grep),
        ("fs-stat", test_fs_stat),
    ]
    results = {}
    for test_name, test_func in tests:
        try:
            success, message = test_func(session, character)
            results[test_name] = (success, message)
        except Exception as e:
            results[test_name] = (False, f"Exception: {str(e)}")
            logger.error(f"Test {test_name} raised exception: {e}")
            logger.error(traceback.format_exc())
    return results


def main():
    parser = argparse.ArgumentParser(description="Test filesystem tools")
    parser.add_argument("--character", type=str, default="Jill", help="Character name (default: Jill)")
    parser.add_argument("--test", type=str, default="all", help="Specific test to run (default: all)")
    args = parser.parse_args()

    print("=" * 60)
    print("FS Tools Test Suite")
    print("=" * 60)
    print(f"Character: {args.character}")
    print(f"Test: {args.test}")
    print()

    print("Connecting to Zenoh...")
    config = zenoh.Config()
    session = zenoh.open(config)
    print("✓ Zenoh session opened")

    try:
        test_result = execute_action(session, args.character, {"type": "think", "value": "connectivity test"}, timeout=10.0)
        if test_result.get("status") != "success":
            print(f"✗ Error: Character '{args.character}' not responding")
            print(f"  Reason: {test_result.get('reason', 'unknown')}")
            return 1
        print(f"✓ Character '{args.character}' is responding")
        print()

        if args.test == "all":
            results = run_all_tests(session, args.character)
        else:
            test_map = {
                "fs-list": ("fs-list", test_fs_list),
                "fs-read-text": ("fs-read-text", test_fs_read_text),
                "fs-read-json": ("fs-read-json", test_fs_read_json),
                "fs-head": ("fs-head", test_fs_head),
                "fs-grep": ("fs-grep", test_fs_grep),
                "fs-stat": ("fs-stat", test_fs_stat),
            }
            if args.test not in test_map:
                print(f"✗ Unknown test: {args.test}")
                print(f"Available tests: {', '.join(test_map.keys())}")
                return 1
            test_name, test_func = test_map[args.test]
            try:
                success, message = test_func(session, args.character)
                results = {test_name: (success, message)}
            except Exception as e:
                results = {test_name: (False, f"Exception: {str(e)}")}
                logger.error(traceback.format_exc())

        print("\n" + "=" * 60)
        print("Test Summary")
        print("=" * 60)
        passed = sum(1 for success, _ in results.values() if success)
        total = len(results)
        for test_name, (success, message) in results.items():
            status_icon = "✅" if success else "❌"
            print(f"{status_icon} {test_name}: {message}")
        print()
        print(f"Total: {passed}/{total} tests passed")
        return 0 if passed == total else 1

    finally:
        session.close()
        print("\n✓ Zenoh session closed")


if __name__ == "__main__":
    exit(main())
