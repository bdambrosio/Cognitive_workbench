/**
 * inspector.js — Right panel showing detail for the selected node.
 */

import { state } from './state.js';
import * as api from './api.js';

const panel = document.getElementById('inspector');
const titleEl = document.getElementById('inspector-title');
const contentEl = document.getElementById('inspector-content');
const actionsEl = document.getElementById('inspector-actions');
const closeBtn = document.getElementById('inspector-close');

export function initInspector() {
    closeBtn.addEventListener('click', () => close());
    state.on('node-selected', (node) => {
        if (node) show(node);
        else close();
    });
}

function show(node) {
    const renderer = renderers[node.type] || renderers.default;
    renderer(node);
    panel.classList.remove('hidden');
    panel.classList.add('visible');
}

function close() {
    panel.classList.remove('visible');
    panel.classList.add('hidden');
}

// ── Renderers by node type ───────────────────────────────

const renderers = {
    agent(node) {
        titleEl.textContent = node.label || 'Agent';
        contentEl.innerHTML = `
            <h3>OODA State</h3>
            <p class="field-label">Phase: <span class="field-value">${state.oodaPhase || 'idle'}</span></p>
            <p class="field-label">Last tick: <span class="field-value">${state.lastTick ? new Date(state.lastTick).toLocaleTimeString() : 'n/a'}</span></p>
            <h3>Recent Actions</h3>
            ${recentActionsHtml()}
        `;
        actionsEl.innerHTML = '';
    },

    goal(node) {
        const d = node.data || {};
        titleEl.textContent = `Goal: ${node.label || node.id}`;
        contentEl.innerHTML = `
            <h3>Status</h3>
            <p><span class="status-badge status-${node.status || 'ready'}">${node.status || 'unknown'}</span></p>
            ${d.goal_text ? `<h3>Goal Text</h3><pre>${escapeHtml(d.goal_text)}</pre>` : ''}
            ${d.last_result ? `<h3>Last Result</h3><pre>${escapeHtml(d.last_result)}</pre>` : ''}
            ${d.primary_product ? `<h3>Output</h3><p class="field-value">${escapeHtml(d.primary_product)}</p>` : ''}
        `;
        actionsEl.innerHTML = goalActions(node);
        bindGoalActions(node);
    },

    concern_user(node) {
        const d = node.data || {};
        titleEl.textContent = `Concern: ${node.label}`;
        contentEl.innerHTML = `
            <h3>User Concern</h3>
            <p class="field-label">Status: <span class="field-value">${d.status || '?'}</span></p>
            <p class="field-label">Weight: <span class="field-value">${d.weight || 0}</span></p>
            <p class="field-label">Stance: <span class="field-value">${d.stance || 'none'}</span></p>
            ${d.concern_description ? `<h3>Description</h3><pre>${escapeHtml(d.concern_description)}</pre>` : ''}
        `;
        actionsEl.innerHTML = '';
    },

    concern_derived(node) {
        const d = node.data || {};
        titleEl.textContent = `Derived: ${node.label}`;
        contentEl.innerHTML = `
            <h3>Derived Concern</h3>
            <p class="field-label">Status: <span class="field-value">${d.status || '?'}</span></p>
            <p class="field-label">Origin: <span class="field-value">${d.origin || '?'}</span></p>
            ${d.parent_user_concern_id ? `<p class="field-label">Parent: <span class="field-value">${d.parent_user_concern_id}</span></p>` : ''}
            ${d.concern_description ? `<h3>Description</h3><pre>${escapeHtml(d.concern_description)}</pre>` : ''}
        `;
        actionsEl.innerHTML = '';
    },

    note(node) {
        titleEl.textContent = `Note: ${node.label || node.id}`;
        contentEl.innerHTML = `<h3>Loading...</h3>`;
        actionsEl.innerHTML = '';
        // Fetch content
        api.getResource(node.data?.resource_id || node.id).then(res => {
            if (res.success && res.content) {
                contentEl.innerHTML = `
                    <h3>Content</h3>
                    <pre>${escapeHtml(typeof res.content === 'string' ? res.content : JSON.stringify(res.content, null, 2))}</pre>
                `;
            } else {
                contentEl.innerHTML = `<p class="field-label">Could not load content.</p>`;
            }
        }).catch(() => {
            contentEl.innerHTML = `<p class="field-label">Error loading content.</p>`;
        });
    },

    binding(node) {
        const d = node.data || {};
        const resId = d.resource_id || '';
        titleEl.textContent = `${d.variable || node.label || node.id}`;
        contentEl.innerHTML = `
            <h3>Binding</h3>
            <p class="field-label">Variable: <span class="field-value">${d.variable || node.id}</span></p>
            <p class="field-label">Resource: <span class="field-value">${resId || 'unknown'}</span></p>
            <h3>Content</h3>
            <div id="binding-content-area"><p class="field-label">Loading...</p></div>
        `;
        actionsEl.innerHTML = '';

        // Auto-load the referenced Note content
        if (resId) {
            api.getResource(resId).then(res => {
                const area = document.getElementById('binding-content-area');
                if (!area) return;
                if (res.success && res.content != null) {
                    const text = typeof res.content === 'string'
                        ? res.content
                        : JSON.stringify(res.content, null, 2);
                    area.innerHTML = `<pre>${escapeHtml(text)}</pre>`;
                } else {
                    area.innerHTML = `<p class="field-label">Could not load content for ${escapeHtml(resId)}.</p>`;
                }
            }).catch(() => {
                const area = document.getElementById('binding-content-area');
                if (area) area.innerHTML = `<p class="field-label">Error loading content.</p>`;
            });
        }
    },

    default(node) {
        titleEl.textContent = node.label || node.id;
        contentEl.innerHTML = `<pre>${escapeHtml(JSON.stringify(node, null, 2))}</pre>`;
        actionsEl.innerHTML = '';
    },
};

// ── Goal Action Buttons ──────────────────────────────────

function goalActions(node) {
    const status = node.status || '';
    const btns = [];
    if (status === 'ready' || status === 'pending' || status === 'scheduled') {
        btns.push(`<button class="dock-btn dock-btn-primary" data-action="proceed">Proceed</button>`);
    }
    if (status === 'running') {
        btns.push(`<button class="dock-btn" data-action="interrupt">Interrupt</button>`);
    }
    if (status === 'completed' || status === 'failed') {
        btns.push(`<button class="dock-btn" data-action="rerun">Rerun</button>`);
    }
    btns.push(`<button class="dock-btn dock-btn-danger" data-action="remove">Remove</button>`);
    return btns.join('');
}

function bindGoalActions(node) {
    const char = state.character;
    const goalId = node.data?.goal_id || node.id;
    actionsEl.querySelectorAll('[data-action]').forEach(btn => {
        btn.addEventListener('click', async () => {
            const action = btn.dataset.action;
            if (action === 'proceed') {
                await api.sendTextInput(char, `proceed ${goalId}`);
            } else if (action === 'interrupt') {
                await api.goalInterrupt(char, { goal_id: goalId });
            } else if (action === 'rerun') {
                await api.sendTextInput(char, `reuse ${goalId}`);
            } else if (action === 'remove') {
                await api.goalRemove(char, { goal_id: goalId });
            }
        });
    });
}

// ── Helpers ──────────────────────────────────────────────

function recentActionsHtml() {
    const actions = state.recentActions.slice(-8).reverse();
    if (!actions.length) return '<p class="field-label">No recent actions.</p>';
    return actions.map(a => {
        const time = a.timestamp ? new Date(a.timestamp).toLocaleTimeString() : '';
        const text = truncate(a.text || a.action_type, 80);
        return `<p><span class="field-label">${time}</span> <span class="field-value">${escapeHtml(text)}</span></p>`;
    }).join('');
}

function escapeHtml(str) {
    if (!str) return '';
    return str.replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');
}

function truncate(str, len) {
    if (!str) return '';
    return str.length > len ? str.slice(0, len - 1) + '\u2026' : str;
}
