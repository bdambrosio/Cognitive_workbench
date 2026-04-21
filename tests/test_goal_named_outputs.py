#!/usr/bin/env python3
"""
Integration test: submit a goal to a running agent via zenoh and verify
that the expected named Notes survive post-completion cleanup.

Prerequisites:
  - Agent (e.g. Jill) must be running with zenoh transport.
  - The filesystem sandbox must contain Clippings/_processed.md and at least
    one unprocessed clip file with a YAML frontmatter 'source' URL.

Usage:
    python tests/test_goal_named_outputs.py [--character Jill] [--timeout 120]
"""

import argparse
import json
import sys
import time

import zenoh


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def zenoh_query(session: zenoh.Session, key_expr: str, timeout: float = 5.0):
    """Send a zenoh query and return the parsed JSON response (or None)."""
    replies = session.get(key_expr, timeout=timeout)
    for reply in replies:
        try:
            if reply.ok:
                return json.loads(reply.ok.payload.to_bytes().decode("utf-8"))
        except Exception:
            pass
    return None


def wait_for_goal_completion(
    session: zenoh.Session, character: str, goal_id: str, timeout: float
) -> dict | None:
    """Poll scheduled_goals until *goal_id* reaches a terminal status."""
    deadline = time.monotonic() + timeout
    start = time.monotonic()
    key = f"cognitive/{character}/scheduled_goals"
    last_report = 0.0
    while time.monotonic() < deadline:
        resp = zenoh_query(session, key)
        if resp and resp.get("success"):
            for g in resp.get("goals", []):
                if g.get("goal_id") == goal_id:
                    status = g.get("status", "")
                    is_running = g.get("is_running", False)
                    if status in ("completed", "failed"):
                        return g
                    # periodic status report every 30s
                    elapsed = time.monotonic() - start
                    if elapsed - last_report >= 30:
                        print(f"  [{int(elapsed)}s] {goal_id}: status={status}, running={is_running}")
                        last_report = elapsed
                    break
        time.sleep(5)
    return None


def find_named_resource(
    session: zenoh.Session, character: str, name: str
) -> dict | None:
    """Search the resources list for a Note or Collection with the given name."""
    key = f"cognitive/{character}/resources"
    resp = zenoh_query(session, key)
    if not resp or not resp.get("success"):
        return None
    for r in resp.get("resources", []):
        props = r.get("properties", {})
        if props.get("note_name") == name or props.get("collection_name") == name:
            return r
    return None


def get_resource_content(
    session: zenoh.Session, character: str, resource_id: str
) -> dict | None:
    """Fetch full resource content by ID."""
    key = f"cognitive/{character}/resource/view/{resource_id}"
    return zenoh_query(session, key)


# ---------------------------------------------------------------------------
# Test
# ---------------------------------------------------------------------------

GOAL_TEXT = (
    "get file fs/Clippings/_processed.md into a Note named processed. "
    "It is a list of filenames. for each file in fs/Clippings/ *not* listed "
    "in the processed Note, extract the url value of the \"source\" field in "
    "the yaml format frontmatter of that file, fetch-text the content of that "
    'url and name the Note "new_paper", then use the obsidian tool to write '
    "the fetched content into a new obsidian note in the obsidian "
    "KnowledgeBases/ folder."
)

# "processed" is always created. "new_paper" only if there were unprocessed clips.
REQUIRED_NOTES = ["processed"]
CONDITIONAL_NOTES = ["new_paper"]  # only expected when unprocessed clips exist


def run_test(character: str, timeout: float) -> bool:
    config = zenoh.Config()
    session = zenoh.open(config)
    ok = True

    try:
        # 1. Pre-test: snapshot existing goals and verify none running
        print("Step 1: Pre-test — querying existing goals and status...")
        pre_resp = zenoh_query(session, f"cognitive/{character}/scheduled_goals")
        pre_goals = {}  # goal_id -> status snapshot
        if pre_resp and pre_resp.get("success"):
            goals_list = pre_resp.get("goals", [])
            for g in goals_list:
                pre_goals[g["goal_id"]] = g.get("status", "?")
            running = [gid for gid, s in pre_goals.items()
                       if s == "running" or any(
                           gg.get("is_running") for gg in goals_list if gg["goal_id"] == gid)]
            print(f"  Existing goals: {sorted(pre_goals) if pre_goals else '(none)'}")
            for g in goals_list:
                gid = g["goal_id"]
                gstatus = g.get("status", "?")
                gname = g.get("name", g.get("goal_text", "")[:60])
                print(f"    {gid}: [{gstatus}] {gname}")
            if running:
                print(f"  FAIL: goals currently running: {running} — cannot start test")
                return False
            print("  No goals currently running — OK to proceed")
        else:
            print("  WARN: could not query scheduled_goals")

        # 2. Submit goal via command channel
        print("Step 2: Submitting goal...")
        cmd = json.dumps({"cmd": "/goal add", "goal_text": GOAL_TEXT})
        session.put(f"cognitive/{character}/command", cmd.encode("utf-8"))
        print(f"  Submitted goal to {character}")

        # 3. Wait for goal to start running (may be new ID or reused existing)
        print("Step 3: Waiting for goal to start running...")
        target_goal_id = None
        started = False
        for attempt in range(5):
            time.sleep(5)
            resp = zenoh_query(session, f"cognitive/{character}/scheduled_goals")
            if not resp or not resp.get("success"):
                print(f"  [{attempt*5+5}s] query failed, retrying...")
                continue
            for g in resp.get("goals", []):
                gid = g["goal_id"]
                gstatus = g.get("status", "?")
                is_running = g.get("is_running", False)
                # Detect: new goal ID, or existing goal whose status changed to running
                is_new = gid not in pre_goals
                was_not_running = pre_goals.get(gid) in ("completed", "failed", "ready", "?")
                if is_new or (was_not_running and (is_running or gstatus == "running")):
                    target_goal_id = gid
                    print(f"  [{attempt*5+5}s] {'New' if is_new else 'Reused'} {gid}: "
                          f"status={gstatus}, running={is_running}")
                    if is_running or gstatus in ("running", "completed", "failed"):
                        started = True
                    break
            if started:
                break

        if not target_goal_id:
            print("  FAIL: could not detect goal start after 25s")
            return False
        if not started:
            print(f"  WARN: goal {target_goal_id} detected but not yet running after 25s")
        print(f"  Goal {target_goal_id} detected and started")
        new_goal_id = target_goal_id

        # 4. Wait for completion
        print(f"Step 4: Waiting up to {timeout}s for completion...")
        goal = wait_for_goal_completion(session, character, new_goal_id, timeout)
        if not goal:
            print(f"  FAIL: goal {new_goal_id} did not complete within {timeout}s")
            return False

        status = goal.get("status")
        print(f"  Goal {new_goal_id} finished with status: {status}")
        if status != "completed":
            print(f"  FAIL: goal status is '{status}', expected 'completed'")
            print(f"    last_result: {goal.get('last_result', '')[:200]}")
            return False

        # 5. Verify goal persisted (not ephemeral — /goal add goals should persist)
        print("Step 5: Checking goal persistence...")
        ephemeral = goal.get("ephemeral", None)
        if ephemeral:
            print(f"  FAIL: goal {new_goal_id} is ephemeral — /goal add goals should persist")
            ok = False
        else:
            print(f"  PASS: goal {new_goal_id} is persistent (ephemeral={ephemeral})")

        # Re-query to confirm the goal record still exists after completion
        post_resp = zenoh_query(session, f"cognitive/{character}/scheduled_goals")
        if post_resp and post_resp.get("success"):
            found = any(g["goal_id"] == new_goal_id for g in post_resp.get("goals", []))
            if found:
                print(f"  PASS: goal {new_goal_id} still present in scheduled_goals")
            else:
                print(f"  FAIL: goal {new_goal_id} was deleted after completion")
                ok = False
        else:
            print("  WARN: could not re-query scheduled_goals")

        # 6. Verify named Notes survived cleanup
        print("Step 6: Checking named Notes...")
        time.sleep(2)  # brief settle after cleanup

        def check_note(name: str, required: bool) -> bool:
            """Check a named note exists, print content info. Returns pass/fail."""
            resource = find_named_resource(session, character, name)
            if not resource:
                if required:
                    print(f"  FAIL: named Note '{name}' not found after goal completion")
                    return False
                else:
                    print(f"  OK: named Note '{name}' not present (conditional — no unprocessed clips)")
                    return True

            rid = resource.get("id", "")
            print(f"  PASS: '{name}' exists as {rid}")

            # Verify non-empty content and print first line
            content_resp = get_resource_content(session, character, rid)
            if content_resp and content_resp.get("success"):
                res_data = content_resp.get("resource", {})
                content = res_data.get("properties", {}).get("content", "")
                content_len = len(content) if isinstance(content, str) else 0
                if content_len == 0:
                    print(f"    WARN: '{name}' ({rid}) has empty content")
                else:
                    first_line = content.split("\n", 1)[0][:120]
                    print(f"    PASS: '{name}' ({rid}) has {content_len} chars")
                    print(f"    first line: {first_line}")
            else:
                print(f"    WARN: could not fetch content for '{name}' ({rid})")
            return True

        for name in REQUIRED_NOTES:
            if not check_note(name, required=True):
                ok = False
        for name in CONDITIONAL_NOTES:
            if not check_note(name, required=False):
                ok = False

    finally:
        session.close()

    if ok:
        print("\nAll checks passed.")
    else:
        print("\nSome checks failed.")
    return ok


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Test goal submission and named-output survival"
    )
    parser.add_argument("--character", default="Jill", help="Agent character name")
    parser.add_argument(
        "--timeout", type=float, default=900, help="Max seconds to wait for goal"
    )
    args = parser.parse_args()

    passed = run_test(args.character, args.timeout)
    sys.exit(0 if passed else 1)


if __name__ == "__main__":
    main()
