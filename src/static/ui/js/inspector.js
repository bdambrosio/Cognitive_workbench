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
            <div id="health-indicator"></div>
            <h3>OODA Feed</h3>
            <div id="ooda-feed" style="max-height:300px;overflow-y:auto;font-size:11px;font-family:monospace">
                <p class="field-label">Loading...</p>
            </div>
            <h3>Recent Actions</h3>
            ${recentActionsHtml()}
        `;
        actionsEl.innerHTML = '';
        // Load OODA feed and health
        loadAgentDetails();
    },

    goal(node) {
        const d = node.data || {};
        const goalId = d.goal_id || node.id;
        const noteId = d.note_id || '';
        const product = d.primary_product || '';
        titleEl.textContent = `Goal: ${node.label || goalId}`;
        contentEl.innerHTML = `
            <p class="field-label">${escapeHtml(goalId)}${noteId ? ` \u00b7 ${escapeHtml(noteId)}` : ''}</p>
            <h3>Status</h3>
            <p><span class="status-badge status-${node.status || 'ready'}">${node.status || 'unknown'}</span></p>
            ${d.goal_text ? `<h3>Goal Text</h3><pre>${escapeHtml(d.goal_text)}</pre>` : ''}
            ${d.last_result ? `<h3>Last Result</h3><pre>${escapeHtml(d.last_result)}</pre>` : ''}
            ${product ? `<h3>Output</h3><p class="field-value">${escapeHtml(product)}</p><div id="goal-product-content"></div>` : ''}
        `;
        // Auto-load primary_product content if it's a Note ID
        if (product && product.match(/^(Note|Collection)_\d+$/)) {
            const area = document.getElementById('goal-product-content');
            if (area) {
                area.innerHTML = '<p class="field-label">Loading content...</p>';
                api.getResource(product).then(res => {
                    if (res.success && res.content != null) {
                        const text = typeof res.content === 'string'
                            ? res.content : JSON.stringify(res.content, null, 2);
                        area.innerHTML = `<pre>${escapeHtml(text)}</pre>`;
                    } else {
                        area.innerHTML = `<p class="field-label" style="color: var(--text-dim); font-style: italic;">Resource ${escapeHtml(product)} was cleaned up after goal completion.</p>`;
                    }
                }).catch(() => {
                    area.innerHTML = `<p class="field-label" style="color: var(--text-dim); font-style: italic;">Resource ${escapeHtml(product)} is no longer available.</p>`;
                });
            }
        }
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
        const resId = node.data?.resource_id || node.id;
        titleEl.textContent = `Note: ${node.label || resId}`;
        contentEl.innerHTML = `<h3>Loading ${escapeHtml(resId)}...</h3>`;
        actionsEl.innerHTML = '';
        // Fetch content
        console.log(`[inspector] fetching resource: ${resId}`);
        api.getResource(resId).then(res => {
            console.log(`[inspector] resource response:`, res);
            if (res.success && res.content != null) {
                contentEl.innerHTML = `
                    <h3>Content</h3>
                    <pre>${escapeHtml(typeof res.content === 'string' ? res.content : JSON.stringify(res.content, null, 2))}</pre>
                `;
            } else {
                contentEl.innerHTML = `<p class="field-label">Could not load: ${escapeHtml(res.message || 'unknown error')}</p>`;
            }
        }).catch((e) => {
            console.error(`[inspector] resource fetch error:`, e);
            contentEl.innerHTML = `<p class="field-label">Error loading content: ${escapeHtml(String(e))}</p>`;
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
            console.log(`[inspector] binding: fetching resource ${resId}`);
            api.getResource(resId).then(res => {
                console.log(`[inspector] binding resource response:`, res);
                const area = document.getElementById('binding-content-area');
                if (!area) return;
                if (res.success && res.content != null) {
                    const text = typeof res.content === 'string'
                        ? res.content
                        : JSON.stringify(res.content, null, 2);
                    area.innerHTML = `<pre>${escapeHtml(text)}</pre>`;
                } else {
                    area.innerHTML = `<p class="field-label">Could not load: ${escapeHtml(res.message || resId)}</p>`;
                }
            }).catch((e) => {
                console.error(`[inspector] binding resource error:`, e);
                const area = document.getElementById('binding-content-area');
                if (area) area.innerHTML = `<p class="field-label">Error: ${escapeHtml(String(e))}</p>`;
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
        btns.push(`<button class="dock-btn" data-action="run">Run</button>`);
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
            } else if (action === 'run') {
                await api.sendTextInput(char, `proceed ${goalId}`);
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

// ── Agent Detail Loaders ────────────────────────────────

async function loadAgentDetails() {
    if (!state.character) return;

    // Load OODA feed
    try {
        const res = await api.getOodaFeed(state.character);
        const feedEl = document.getElementById('ooda-feed');
        if (feedEl && res.success && res.events) {
            const events = res.events.slice(-20).reverse();
            if (!events.length) {
                feedEl.innerHTML = '<p class="field-label">No OODA events yet.</p>';
            } else {
                feedEl.innerHTML = events.map(e => {
                    const time = e.timestamp ? e.timestamp.split('T')[1] || e.timestamp : '';
                    const bumps = e.concern_bumps || '';
                    const src = (e.source || '').replace('sensor:', 'S:');
                    return `<div style="padding:2px 0;border-bottom:1px solid #2a2a2a">` +
                        `<span style="color:#666">${escapeHtml(time)}</span> ` +
                        `<span style="color:#4ec9b0">${escapeHtml(src)}</span> ` +
                        `<span style="color:#dbb86d">${escapeHtml(e.classification || '')}</span>` +
                        ` \u2192 <span style="color:#6ddb6d">${escapeHtml(e.action_taken || '')}</span>` +
                        (bumps ? `<div style="color:#888;padding-left:8px">${escapeHtml(bumps)}</div>` : '') +
                        `</div>`;
                }).join('');
            }
        }
    } catch (e) {
        const feedEl = document.getElementById('ooda-feed');
        if (feedEl) feedEl.innerHTML = '<p class="field-label">Failed to load feed.</p>';
    }

    // Load health indicator from concerns
    try {
        const res = await api.getConcerns(state.character);
        const healthEl = document.getElementById('health-indicator');
        if (healthEl && res.success) {
            // Find system_health_monitoring concern or health-related task
            const dc = res.derived_concerns || [];
            const healthConcern = dc.find(c =>
                c.concern_label && c.concern_label.includes('health'));
            const activations = res.activations || {};

            // Build health summary
            const parts = [];
            if (healthConcern) {
                const act = activations[healthConcern.concern_id] || {};
                const status = healthConcern.status || '?';
                const av = (act.activation || 0).toFixed(2);
                const statusColor = status === 'active' ? '#dbb86d'
                    : status === 'satisfied' ? '#6ddba5' : '#888';
                parts.push(`<span style="color:${statusColor}">${escapeHtml(status)}</span>`);
                parts.push(`activation: ${av}`);
            }
            // Count concern statuses
            const statuses = {};
            dc.forEach(c => { statuses[c.status] = (statuses[c.status] || 0) + 1; });
            const statusSummary = Object.entries(statuses)
                .map(([s, n]) => `${n} ${s}`).join(', ');
            if (statusSummary) parts.push(statusSummary);

            if (parts.length) {
                healthEl.innerHTML = `<h3>Health</h3><p class="field-label">${parts.join(' \u00b7 ')}</p>`;
            }
        }
    } catch (e) {
        // Silently ignore
    }
}
