#!/usr/bin/env python3
"""
Standalone Task & Concern Manager — manage the autonomy pipeline.

Connects to executive_node via Zenoh (through the FastAPI display node) and
provides a web UI for:
  - Viewing concerns (user + derived) with activation levels and trends
  - Viewing tasks across all lifecycle states
  - Approving, editing, and abandoning proposed tasks
  - Viewing triage pipeline status
  - Viewing task learnings from distillation

Standalone app on port 3002, following the resource_browser.py pattern.
"""

import argparse
import json
import logging
import signal
import sys
import webbrowser
from pathlib import Path
from typing import Dict, List, Optional

import zenoh
from fastapi import Body, FastAPI, HTTPException
from fastapi.responses import HTMLResponse
from fastapi.middleware.cors import CORSMiddleware
import uvicorn

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class TaskManager:
    """Browse and manage tasks and concerns from executive_node."""

    def __init__(self, port: int = 3002, open_browser: bool = True):
        self.port = port
        self.open_browser = open_browser
        self.app = FastAPI(title="Task & Concern Manager")
        self.app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"], allow_methods=["*"], allow_headers=["*"],
        )
        self.shutdown_requested = False

        # Zenoh session (localhost only)
        from utils.zenoh_utils import make_localhost_config
        self.session = zenoh.open(make_localhost_config())
        logger.info("Connected to Zenoh")

        signal.signal(signal.SIGTERM, self._signal_handler)
        signal.signal(signal.SIGINT, self._signal_handler)

        self.setup_routes()

    def _signal_handler(self, signum, frame):
        logger.info(f'Received signal {signum}, shutting down...')
        self.shutdown()

    def shutdown(self):
        if getattr(self, '_shutting_down', False):
            return
        self._shutting_down = True
        self.shutdown_requested = True
        try:
            if hasattr(self, 'session'):
                self.session.close()
            logger.info('Task Manager shutdown complete')
        except Exception as e:
            logger.error(f'Error during shutdown: {e}')

    # ── Zenoh queries ────────────────────────────────────────────────────

    def _zenoh_get(self, key: str, timeout: float = 2.0) -> Optional[Dict]:
        """Query Zenoh and return parsed JSON response."""
        from zenoh import QueryTarget, ConsolidationMode
        for reply in self.session.get(
            key, target=QueryTarget.BEST_MATCHING,
            consolidation=ConsolidationMode.NONE, timeout=timeout
        ):
            if reply.ok:
                return json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
        return None

    def _zenoh_put(self, key: str, data: Dict):
        """Publish a JSON payload to Zenoh."""
        self.session.put(key, json.dumps(data).encode('utf-8'))

    def _read_concerns_from_resources(self, character: str):
        """Fallback: read concern data from raw named notes via /resources."""
        user_concerns, derived_concerns = [], []
        resources = self._zenoh_get(f"cognitive/{character}/resources") or {}
        if not resources.get('success'):
            return user_concerns, derived_concerns
        for r in resources.get('resources', []):
            props = r.get('properties', {})
            name = props.get('note_name', '') or r.get('name', '')
            if name not in ('_user_concerns', '_derived_concerns'):
                continue
            detail = self._zenoh_get(f"cognitive/{character}/resource/{r['id']}")
            if not detail or not detail.get('success'):
                continue
            res = detail.get('resource', {})
            content = res.get('properties', {}).get('content', '') or detail.get('content', '')
            try:
                parsed = json.loads(content) if isinstance(content, str) else content
            except (json.JSONDecodeError, TypeError):
                continue
            if name == '_user_concerns' and isinstance(parsed, list):
                user_concerns = parsed
            elif name == '_derived_concerns' and isinstance(parsed, list):
                derived_concerns = parsed
        return user_concerns, derived_concerns

    # ── Routes ───────────────────────────────────────────────────────────

    def setup_routes(self):

        @self.app.get("/", response_class=HTMLResponse)
        def index():
            return self.get_html()

        @self.app.get("/api/characters")
        def list_characters():
            """Discover active characters."""
            result = self._zenoh_get("cognitive/*/concerns")
            if result and result.get('success'):
                return {'success': True, 'found': True}
            return {'success': True, 'found': False}

        @self.app.get("/api/data/{character}")
        def get_all_data(character: str):
            """Get all task manager data in one call.

            Tries the dedicated /concerns queryable first; falls back to reading
            the raw named notes (_user_concerns, _derived_concerns) via the
            /resources queryable that already exists on older executive_node builds.
            """
            # Normalize: executive_node capitalizes character names
            character = character.strip().capitalize()
            concerns = self._zenoh_get(f"cognitive/{character}/concerns") or {}
            tasks_resp = self._zenoh_get(f"cognitive/{character}/task_wips") or {}
            triage = self._zenoh_get(f"cognitive/{character}/triage_status") or {}

            user_concerns = concerns.get('user_concerns', [])
            derived_concerns = concerns.get('derived_concerns', [])
            activations = concerns.get('activations', {})

            # Fallback: if the dedicated queryable isn't available yet,
            # read concern data from the raw named-note resources.
            if not user_concerns and not derived_concerns:
                user_concerns, derived_concerns = self._read_concerns_from_resources(character)

            # Fetch learning notes from resources
            resources = self._zenoh_get(f"cognitive/{character}/resources") or {}
            learnings = []
            if resources.get('success'):
                for r in resources.get('resources', []):
                    # Resource name is in properties.note_name, not top-level 'name'
                    props = r.get('properties', {})
                    name = props.get('note_name', '') or r.get('name', '')
                    if name.startswith('_task_learning_'):
                        rid = r.get('id', '')
                        detail = self._zenoh_get(f"cognitive/{character}/resource/{rid}")
                        if detail and detail.get('success'):
                            res = detail.get('resource', {})
                            content = res.get('properties', {}).get('content', '') or detail.get('content', '')
                            try:
                                parsed = json.loads(content) if isinstance(content, str) else content
                                if isinstance(parsed, dict):
                                    learnings.append(parsed)
                            except (json.JSONDecodeError, TypeError):
                                pass
            return {
                'success': True,
                'user_concerns': user_concerns,
                'derived_concerns': derived_concerns,
                'activations': activations,
                'tasks': tasks_resp.get('tasks', []),
                'triage': triage,
                'learnings': learnings,
            }

        @self.app.post("/api/approve/{character}")
        def approve_task(character: str, data: dict = Body(...)):
            character = character.strip().capitalize()
            self._zenoh_put(f"cognitive/{character}/control/task_approve", data)
            return {'success': True}

        @self.app.post("/api/abandon/{character}")
        def abandon_task(character: str, data: dict = Body(...)):
            character = character.strip().capitalize()
            self._zenoh_put(f"cognitive/{character}/control/task_abandon", data)
            return {'success': True}

        @self.app.post("/api/edit/{character}")
        def edit_task(character: str, data: dict = Body(...)):
            character = character.strip().capitalize()
            self._zenoh_put(f"cognitive/{character}/control/task_edit", data)
            return {'success': True}

        @self.app.post("/api/delete/{character}")
        def delete_task(character: str, data: dict = Body(...)):
            character = character.strip().capitalize()
            self._zenoh_put(f"cognitive/{character}/control/task_wip_delete", data)
            return {'success': True}

        @self.app.post("/api/interrupt/{character}")
        def interrupt_task(character: str, data: dict = Body(...)):
            character = character.strip().capitalize()
            self._zenoh_put(f"cognitive/{character}/control/task_wip_interrupt", data)
            return {'success': True}

    # ── HTML UI ──────────────────────────────────────────────────────────

    def get_html(self) -> str:
        return """<!DOCTYPE html>
<html>
<head>
<title>Task & Concern Manager</title>
<style>
* { margin: 0; padding: 0; box-sizing: border-box; }
body {
    font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
    background: #1e1e1e; color: #d4d4d4;
    height: 100vh; display: flex; flex-direction: column;
}
header {
    background: #252526; padding: 10px 20px;
    border-bottom: 1px solid #3e3e42;
    display: flex; justify-content: space-between; align-items: center;
}
h1 { font-size: 16px; font-weight: 500; color: #ccc; }
.header-controls { display: flex; gap: 8px; align-items: center; }
.header-controls select {
    background: #3c3c3c; color: #ccc; border: 1px solid #555;
    padding: 4px 8px; border-radius: 3px; font-size: 12px;
}
button {
    background: #0e639c; color: white; border: none;
    padding: 5px 12px; border-radius: 3px; cursor: pointer; font-size: 12px;
}
button:hover { background: #1177bb; }
button.danger { background: #a1260d; }
button.danger:hover { background: #c1341d; }
button.approve { background: #388a34; }
button.approve:hover { background: #45a340; }
button.secondary { background: #3c3c3c; }
button.secondary:hover { background: #4e4e4e; }

.main { display: flex; flex: 1; overflow: hidden; }

/* Left panel: Concerns */
.panel { overflow-y: auto; padding: 12px; }
.panel-left { width: 340px; border-right: 1px solid #3e3e42; }
.panel-right { flex: 1; }

.section-title {
    font-size: 11px; font-weight: 600; text-transform: uppercase;
    color: #888; margin: 12px 0 6px 0; letter-spacing: 0.5px;
}
.section-title:first-child { margin-top: 0; }

/* Concern cards — accordion */
.concern-card {
    background: #2d2d30; border: 1px solid #3e3e42; border-radius: 4px;
    margin-bottom: 4px; font-size: 12px; overflow: hidden;
}
.concern-summary {
    padding: 6px 10px; cursor: pointer; user-select: none;
}
.concern-summary:hover { background: #353538; }
.concern-header { display: flex; justify-content: space-between; align-items: center; }
.concern-label { font-weight: 600; color: #4ec9b0; }
.concern-chevron {
    font-size: 10px; color: #666; margin-right: 4px; display: inline-block;
    transition: transform 0.15s;
}
.concern-card.open .concern-chevron { transform: rotate(90deg); }
.concern-details {
    display: none; padding: 0 10px 8px 10px;
    border-top: 1px solid #3e3e42;
}
.concern-card.open .concern-details { display: block; }
.concern-status {
    font-size: 10px; padding: 1px 6px; border-radius: 8px;
    background: #3e3e42; color: #aaa;
}
.concern-status.active { background: #2a4d2a; color: #6ddb6d; }
.concern-status.surfaced { background: #4d3e2a; color: #dbb86d; }
.concern-status.resolved { background: #2a3d4d; color: #6db1db; }
.concern-desc { color: #999; margin-top: 4px; line-height: 1.4; }
.activation-bar {
    margin-top: 5px; height: 4px; background: #3e3e42;
    border-radius: 2px; overflow: hidden;
}
.activation-fill { height: 100%; border-radius: 2px; transition: width 0.3s; }
.activation-info {
    display: flex; justify-content: space-between; font-size: 10px;
    color: #888; margin-top: 2px;
}
.trend-rising { color: #f0a050; }
.trend-falling { color: #6090d0; }

/* Task cards */
.task-card {
    background: #2d2d30; border: 1px solid #3e3e42; border-radius: 4px;
    padding: 10px 12px; margin-bottom: 8px;
}
.task-header { display: flex; justify-content: space-between; align-items: flex-start; }
.task-intention { font-size: 13px; color: #dcdcaa; flex: 1; line-height: 1.4; }
.task-status {
    font-size: 10px; padding: 2px 8px; border-radius: 8px;
    margin-left: 8px; white-space: nowrap; font-weight: 600;
}
.task-status.proposed { background: #4d3e2a; color: #dbb86d; }
.task-status.in_progress, .task-status.establishing { background: #2a3d4d; color: #6db1db; }
.task-status.active { background: #2a4d2a; color: #6ddb6d; }
.task-status.completed { background: #2a4d3d; color: #6ddba5; }
.task-status.abandoned { background: #4d2a2a; color: #db6d6d; }
.task-status.archived { background: #3e3e42; color: #888; }
.task-meta { font-size: 11px; color: #888; margin-top: 4px; }
.task-meta span { color: #aaa; }
.task-actions { display: flex; gap: 6px; margin-top: 8px; }
.task-reason { font-size: 11px; color: #999; margin-top: 4px; font-style: italic; }

/* Edit intention */
.edit-field {
    width: 100%; background: #1e1e1e; border: 1px solid #555;
    color: #d4d4d4; padding: 6px 8px; border-radius: 3px;
    font-size: 12px; font-family: inherit; resize: vertical;
    margin-top: 6px;
}
.edit-field:focus { outline: none; border-color: #007acc; }

/* Triage status */
.triage-box {
    background: #2d2d30; border: 1px solid #3e3e42; border-radius: 4px;
    padding: 8px 10px; margin-bottom: 8px; font-size: 12px;
}
.triage-candidate {
    padding: 4px 0; border-bottom: 1px solid #3e3e42;
}
.triage-candidate:last-child { border-bottom: none; }

/* Learnings */
.learning-card {
    background: #2d2d30; border: 1px solid #3e3e42; border-radius: 4px;
    padding: 10px 12px; margin-bottom: 8px; font-size: 12px;
}
.learning-summary { color: #ce9178; margin-bottom: 4px; }
.learning-item { color: #9cdcfe; padding: 2px 0; }
.learning-item::before { content: "• "; color: #555; }

/* Empty state */
.empty { color: #666; font-size: 13px; padding: 20px 0; text-align: center; }

::-webkit-scrollbar { width: 8px; }
::-webkit-scrollbar-track { background: #1e1e1e; }
::-webkit-scrollbar-thumb { background: #424242; border-radius: 4px; }
::-webkit-scrollbar-thumb:hover { background: #4e4e4e; }
</style>
</head>
<body>
<header>
    <h1>Task & Concern Manager</h1>
    <div class="header-controls">
        <label style="font-size:12px;color:#888;">Character:</label>
        <input id="char-input" type="text" value="Jill"
            style="background:#3c3c3c;color:#ccc;border:1px solid #555;padding:4px 8px;
                   border-radius:3px;font-size:12px;width:100px;">
        <button onclick="refresh()">Refresh</button>
        <label style="font-size:12px;color:#888;">
            <input type="checkbox" id="auto-refresh" checked> Auto (10s)
        </label>
    </div>
</header>

<div class="main">
    <!-- Left panel: Concerns -->
    <div class="panel panel-left">
        <div class="section-title">User Concerns (<span id="uc-count">0</span>)</div>
        <div id="user-concerns"></div>

        <div class="section-title">Derived Concerns (<span id="dc-count">0</span>)</div>
        <div id="derived-concerns"></div>

        <div class="section-title">Triage Pipeline</div>
        <div id="triage-status"></div>
    </div>

    <!-- Right panel: Tasks & Learnings -->
    <div class="panel panel-right">
        <div class="section-title">Proposed Tasks (<span id="proposed-count">0</span>)</div>
        <div id="proposed-tasks"></div>

        <div class="section-title">Active Tasks (<span id="active-count">0</span>)</div>
        <div id="active-tasks"></div>

        <div class="section-title">Establishing Tasks (<span id="establishing-count">0</span>)</div>
        <div id="establishing-tasks"></div>

        <div class="section-title">Completed / Archived (<span id="done-count">0</span>)</div>
        <div id="done-tasks"></div>

        <div class="section-title">Learnings (<span id="learnings-count">0</span>)</div>
        <div id="learnings"></div>
    </div>
</div>

<script>
let data = {};
let autoTimer = null;

function char() { return document.getElementById('char-input').value.trim() || 'jill'; }

async function refresh() {
    try {
        const resp = await fetch(`/api/data/${char()}`);
        data = await resp.json();
        if (data.success) render();
    } catch (e) {
        console.error('Refresh failed:', e);
    }
}

function render() {
    renderConcerns('user-concerns', data.user_concerns || [], 'uc-count');
    renderDerivedConcerns('derived-concerns', data.derived_concerns || [], 'dc-count');
    renderTriage(data.triage || {});
    renderTasks(data.tasks || []);
    renderLearnings(data.learnings || []);
}

function activationColor(v) {
    if (v > 0.6) return '#f0a050';
    if (v > 0.3) return '#a0c060';
    return '#608090';
}

function toggleConcern(el) {
    el.closest('.concern-card').classList.toggle('open');
}

function renderConcerns(containerId, concerns, countId) {
    const el = document.getElementById(containerId);
    const visible = concerns.filter(c => c.status !== 'closed');
    document.getElementById(countId).textContent = visible.length;
    if (!visible.length) { el.innerHTML = '<div class="empty">No concerns</div>'; return; }
    el.innerHTML = visible.map(c => {
        const act = (data.activations || {})[c.concern_id] || {};
        const av = act.activation || 0;
        const trend = act.trend || 'stable';
        const arrow = trend === 'rising' ? '↑' : trend === 'falling' ? '↓' : '→';
        const trendClass = trend === 'rising' ? 'trend-rising' : trend === 'falling' ? 'trend-falling' : '';
        return `<div class="concern-card">
            <div class="concern-summary" onclick="toggleConcern(this)">
                <div class="concern-header">
                    <span><span class="concern-chevron">▶</span><span class="concern-label">${esc(c.concern_label)}</span></span>
                    <span class="concern-status ${c.status}">${c.status}</span>
                </div>
                <div class="activation-bar" style="margin-top:4px"><div class="activation-fill" style="width:${av*100}%;background:${activationColor(av)}"></div></div>
                <div class="activation-info">
                    <span>activation: ${av.toFixed(2)}</span>
                    <span class="${trendClass}">${arrow} ${trend}</span>
                    <span>weight: ${(c.weight||0).toFixed(2)}</span>
                </div>
            </div>
            <div class="concern-details">
                ${c.concern_description ? `<div class="concern-desc" style="margin-top:6px">${esc(c.concern_description)}</div>` : ''}
                <div style="font-size:10px;color:#666;margin-top:4px">
                    stance: ${c.stance || '—'} | touches: ${c.touch_count || 0}
                </div>
            </div>
        </div>`;
    }).join('');
}

function renderDerivedConcerns(containerId, concerns, countId) {
    const el = document.getElementById(containerId);
    document.getElementById(countId).textContent = concerns.length;
    if (!concerns.length) { el.innerHTML = '<div class="empty">No derived concerns</div>'; return; }
    el.innerHTML = concerns.map(c => {
        const act = (data.activations || {})[c.concern_id] || {};
        const av = act.activation || 0;
        const trend = act.trend || 'stable';
        const arrow = trend === 'rising' ? '↑' : trend === 'falling' ? '↓' : '→';
        const trendClass = trend === 'rising' ? 'trend-rising' : trend === 'falling' ? 'trend-falling' : '';
        return `<div class="concern-card">
            <div class="concern-summary" onclick="toggleConcern(this)">
                <div class="concern-header">
                    <span><span class="concern-chevron">▶</span><span class="concern-label">${esc(c.concern_label)}</span></span>
                    <span class="concern-status ${c.status}">${c.status}</span>
                </div>
                <div class="activation-bar" style="margin-top:4px"><div class="activation-fill" style="width:${av*100}%;background:${activationColor(av)}"></div></div>
                <div class="activation-info">
                    <span>activation: ${av.toFixed(2)}</span>
                    <span class="${trendClass}">${arrow} ${trend}</span>
                    <span>origin: ${c.origin || '?'}</span>
                </div>
            </div>
            <div class="concern-details">
                ${c.concern_description ? `<div class="concern-desc" style="margin-top:6px">${esc(c.concern_description)}</div>` : ''}
                ${c.parent_user_concern_id ? `<div style="font-size:10px;color:#666;margin-top:4px">parent: ${esc(c.parent_user_concern_id)}</div>` : ''}
                <div style="font-size:10px;color:#666;margin-top:2px">
                    ${c.seeded ? 'seed concern' : ''} ${c.status_rationale ? '| ' + esc(c.status_rationale) : ''}
                </div>
            </div>
        </div>`;
    }).join('');
}

function renderTriage(triage) {
    const el = document.getElementById('triage-status');
    const candidates = triage.candidates || [];
    const deferred = triage.deferred || {};
    const deferredCount = Object.keys(deferred).length;
    const budgetTotal = triage.budget_total_seconds || 0;
    const budgetRemaining = triage.budget_remaining_seconds || 0;

    let html = '<div class="triage-box">';

    // Autonomy budget bar
    if (budgetTotal > 0) {
        const budgetUsedPct = Math.max(0, Math.min(100, ((budgetTotal - budgetRemaining) / budgetTotal) * 100));
        const budgetColor = budgetRemaining > 0 ? '#6ddb6d' : '#db6d6d';
        const windowMin = Math.round((triage.budget_window_seconds || 3600) / 60);
        html += `<div style="margin-bottom:6px">
            <div style="display:flex;justify-content:space-between;font-size:11px;color:#aaa;margin-bottom:2px">
                <span>Autonomy budget</span>
                <span>${Math.round(budgetRemaining/60)}m / ${Math.round(budgetTotal/60)}m per ${windowMin}m</span>
            </div>
            <div class="activation-bar"><div class="activation-fill" style="width:${budgetUsedPct}%;background:${budgetColor}"></div></div>
        </div>`;
    }

    if (candidates.length) {
        html += `<div style="color:#dbb86d;margin-bottom:4px">${candidates.length} candidate(s) queued</div>`;
        candidates.forEach(c => {
            html += `<div class="triage-candidate">
                <span style="color:#4ec9b0">${esc(c.label)}</span>
                <span style="color:#888"> — ${c.source}, act=${(c.activation||0).toFixed(2)}</span>
            </div>`;
        });
    } else if (!deferredCount && !budgetTotal) {
        html += '<span style="color:#888">No candidates in triage pipeline</span>';
    } else if (!candidates.length) {
        html += '<span style="color:#888">No candidates queued</span>';
    }
    if (deferredCount) {
        html += `<div style="color:#888;margin-top:4px">${deferredCount} concern(s) deferred</div>`;
    }
    html += '</div>';
    el.innerHTML = html;
}

function renderTasks(tasks) {
    const proposed = tasks.filter(t => t.status === 'proposed');
    const active = tasks.filter(t => t.status === 'active');
    const establishing = tasks.filter(t => t.status === 'in_progress' || t.status === 'establishing');
    const done = tasks.filter(t => ['completed', 'archived', 'abandoned'].includes(t.status));

    document.getElementById('proposed-count').textContent = proposed.length;
    document.getElementById('active-count').textContent = active.length;
    document.getElementById('establishing-count').textContent = establishing.length;
    document.getElementById('done-count').textContent = done.length;

    document.getElementById('proposed-tasks').innerHTML = proposed.length
        ? proposed.map(renderProposedTask).join('')
        : '<div class="empty">No proposed tasks</div>';

    document.getElementById('active-tasks').innerHTML = active.length
        ? active.map(renderActiveTask).join('')
        : '<div class="empty">No active tasks</div>';

    document.getElementById('establishing-tasks').innerHTML = establishing.length
        ? establishing.map(renderEstablishingTask).join('')
        : '<div class="empty">No tasks establishing</div>';

    document.getElementById('done-tasks').innerHTML = done.length
        ? done.map(renderDoneTask).join('')
        : '<div class="empty">No completed tasks</div>';
}

function renderProposedTask(t) {
    const noteName = t._note_name || '';
    return `<div class="task-card" id="task-${esc(noteName)}">
        <div class="task-header">
            <div class="task-intention">${esc(t.intention)}</div>
            <span class="task-status proposed">proposed</span>
        </div>
        ${t.proposal_reason ? `<div class="task-reason">${esc(t.proposal_reason)}</div>` : ''}
        <div class="task-meta">
            Concern: <span>${esc(t.linked_concern_id || 'none')}</span>
            &nbsp;|&nbsp; Created: <span>${fmtDate(t.created)}</span>
        </div>
        <textarea class="edit-field" id="edit-${esc(noteName)}" rows="2">${esc(t.intention)}</textarea>
        <div class="task-actions">
            <button class="approve" onclick="approveTask('${esc(noteName)}')">Approve</button>
            <button class="danger" onclick="abandonTask('${esc(noteName)}')">Dismiss</button>
        </div>
    </div>`;
}

function renderActiveTask(t) {
    const noteName = t._note_name || '';
    return `<div class="task-card">
        <div class="task-header">
            <div class="task-intention">${esc(t.intention)}</div>
            <span class="task-status active">active</span>
        </div>
        <div class="task-meta">
            Goal: <span>${esc(t._scheduled_goal_name || t.operational_goal_id || '—')}</span>
            &nbsp;|&nbsp; Schedule: <span>${t.schedule_mode || '—'}</span>
            &nbsp;|&nbsp; Executions: <span>${t.execution_count || 0}</span>
        </div>
        <div class="task-actions">
            <button class="danger" onclick="abandonTask('${esc(noteName)}')">Abandon</button>
            <button class="danger" onclick="deleteTask('${esc(noteName)}')">Delete</button>
        </div>
    </div>`;
}

function renderEstablishingTask(t) {
    const noteName = t._note_name || '';
    const milestones = t.milestones_completed || [];
    return `<div class="task-card">
        <div class="task-header">
            <div class="task-intention">${esc(t.intention)}</div>
            <span class="task-status establishing">establishing</span>
        </div>
        <div class="task-meta">
            Phase: <span>${t.phase || '?'}</span>
            &nbsp;|&nbsp; Milestones: <span>${milestones.length}</span>
            ${t.current_milestone ? `&nbsp;|&nbsp; Current: <span>${esc(t.current_milestone)}</span>` : ''}
        </div>
        ${milestones.length ? `<div style="margin-top:6px;font-size:11px;color:#888">
            ${milestones.map(m => `<div style="padding:2px 0;line-height:1.4">
                <span style="color:${m.status==='completed'?'#6ddb6d':'#db6d6d'}">${m.status==='completed'?'✓':'✗'}</span>
                ${esc((m.goal_text||'').substring(0,200))}
            </div>`).join('')}
        </div>` : ''}
        <div class="task-actions">
            <button class="secondary" onclick="interruptTask('${esc(noteName)}')">Interrupt</button>
            <button class="danger" onclick="deleteTask('${esc(noteName)}')">Delete</button>
        </div>
    </div>`;
}

function renderDoneTask(t) {
    const noteName = t._note_name || '';
    return `<div class="task-card">
        <div class="task-header">
            <div class="task-intention">${esc(t.intention)}</div>
            <span class="task-status ${t.status}">${t.status}</span>
        </div>
        <div class="task-meta">
            ${t.completion_summary ? `Summary: <span>${esc(t.completion_summary)}</span>` : ''}
            ${t.abandon_reason ? `Reason: <span>${esc(t.abandon_reason)}</span>` : ''}
            &nbsp;|&nbsp; Updated: <span>${fmtDate(t.updated)}</span>
        </div>
        <div class="task-actions">
            <button class="danger" onclick="deleteTask('${esc(noteName)}')">Delete</button>
        </div>
    </div>`;
}

function renderLearnings(learnings) {
    const el = document.getElementById('learnings');
    document.getElementById('learnings-count').textContent = learnings.length;
    if (!learnings.length) { el.innerHTML = '<div class="empty">No learnings yet</div>'; return; }
    el.innerHTML = learnings.map(l => `<div class="learning-card">
        <div class="learning-summary">${esc(l.summary || '')}</div>
        <div style="font-size:11px;color:#888;margin-bottom:4px">
            Task: ${esc(l.task_wip_id || '?')} | Outcome: ${esc(l.outcome || '?')}
            | Concern: ${esc(l.concern_recommendation || '?')}
        </div>
        ${(l.learnings||[]).map(item => `<div class="learning-item">${esc(item)}</div>`).join('')}
    </div>`).join('');
}

// ── Actions ──────────────────────────────────────────────────────

async function approveTask(noteName) {
    const editEl = document.getElementById('edit-' + noteName);
    const intention = editEl ? editEl.value.trim() : '';
    await fetch(`/api/approve/${char()}`, {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({note_name: noteName, intention: intention}),
    });
    setTimeout(refresh, 500);
}

async function abandonTask(noteName) {
    const reason = prompt('Reason for abandoning (optional):') || '';
    await fetch(`/api/abandon/${char()}`, {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({note_name: noteName, reason: reason}),
    });
    setTimeout(refresh, 500);
}

async function deleteTask(noteName) {
    if (!confirm('Delete this task and all its artifacts? This cannot be undone.')) return;
    await fetch(`/api/delete/${char()}`, {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({note_name: noteName}),
    });
    setTimeout(refresh, 500);
}

async function interruptTask(noteName) {
    await fetch(`/api/interrupt/${char()}`, {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({note_name: noteName}),
    });
    setTimeout(refresh, 500);
}

// ── Helpers ──────────────────────────────────────────────────────

function esc(s) {
    if (s == null) return '';
    const d = document.createElement('div');
    d.textContent = String(s);
    return d.innerHTML;
}

function fmtDate(iso) {
    if (!iso) return '—';
    try {
        const d = new Date(iso);
        return d.toLocaleString(undefined, {month:'short', day:'numeric', hour:'2-digit', minute:'2-digit'});
    } catch { return iso; }
}

// ── Auto-refresh ─────────────────────────────────────────────────

function startAutoRefresh() {
    stopAutoRefresh();
    if (document.getElementById('auto-refresh').checked) {
        autoTimer = setInterval(refresh, 10000);
    }
}

function stopAutoRefresh() {
    if (autoTimer) { clearInterval(autoTimer); autoTimer = null; }
}

document.getElementById('auto-refresh').addEventListener('change', () => {
    if (document.getElementById('auto-refresh').checked) startAutoRefresh();
    else stopAutoRefresh();
});

// Initial load
refresh();
startAutoRefresh();
</script>
</body>
</html>"""

    def run(self):
        """Start the web server."""
        import threading
        if self.open_browser:
            threading.Timer(1.0, lambda: webbrowser.open(f'http://localhost:{self.port}')).start()
        logger.info(f"Starting Task Manager on http://0.0.0.0:{self.port}")
        uvicorn.run(self.app, host="0.0.0.0", port=self.port, log_level="info")


def main():
    parser = argparse.ArgumentParser(description="Task & Concern Manager")
    parser.add_argument('--port', type=int, default=3002, help='Web server port (default: 3002)')
    parser.add_argument('--no-browser', action='store_true', help='Do not open browser automatically')
    args = parser.parse_args()
    manager = TaskManager(port=args.port, open_browser=not args.no_browser)
    manager.run()


if __name__ == "__main__":
    main()
