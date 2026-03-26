/**
 * main.js — Application entry point.
 *
 * Initializes all modules, connects WebSocket, starts topology polling.
 */

import { state } from './state.js';
import { connect } from './websocket.js';
import { initGraph, flashNode, highlightCompletion } from './graph.js';
import { initInspector } from './inspector.js';
import { initDock } from './dock.js';
import { initPulse, triggerPulse } from './pulse.js';
import * as api from './api.js';

// ── Boot ─────────────────────────────────────────────────

document.addEventListener('DOMContentLoaded', () => {
    initGraph();
    initInspector();
    initDock();
    initPulse();
    connect();

    // Seed the agent node immediately
    state.upsertNode('agent', {
        type: 'agent',
        label: 'Agent',
        activation: 1.0,
    });

    // Once we know the character name, start polling topology
    state.on('character-detected', (char) => {
        state.upsertNode('agent', { type: 'agent', label: char, activation: 1.0 });
        refreshTopology();
    });

    // Also try to detect character on initial connect
    state.on('connection', ({ connected }) => {
        if (connected && !state.character) {
            // Try to discover character from existing endpoints
            api.getCharacters().then(res => {
                if (res.characters && res.characters.length > 0) {
                    state.character = res.characters[0];
                    state.upsertNode('agent', {
                        type: 'agent',
                        label: state.character,
                        activation: 1.0,
                    });
                    refreshTopology();
                }
            }).catch(() => {});
        }
    });

    // Periodic topology refresh
    setInterval(() => {
        if (state.character && state.connected) {
            refreshTopology();
        }
    }, 8000);

    // Refresh bindings more frequently during active execution
    setInterval(() => {
        if (state.character && state.connected) {
            refreshBindings();
        }
    }, 4000);

    // Refresh health indicator in dock
    setInterval(() => {
        if (state.character && state.connected) {
            refreshHealthIndicator();
        }
    }, 15000);
    // Initial health check after character detected
    state.on('character-detected', () => setTimeout(refreshHealthIndicator, 2000));

    // On goal updates from WebSocket, refresh topology
    state.on('goal-update', () => {
        setTimeout(() => refreshTopology(), 500);
    });
    state.on('action', (msg) => {
        // Flash the agent node on any action
        if (msg.action_type && msg.action_type !== 'say') {
            flashNode('agent', 'up');
        }
    });
});

// ── Topology Refresh ─────────────────────────────────────

// Track previous goal statuses to detect completion transitions
const prevGoalStatuses = new Map();

async function refreshTopology() {
    if (!state.character) return;

    try {
        const res = await api.getTopology(state.character);
        if (res.success && res.nodes && res.edges) {
            console.log(`[topology] ${res.nodes.length} nodes, ${res.edges.length} edges`);
            // Detect goals that just completed with a primary_product
            detectGoalCompletions(res.nodes);
            state.setTopology(res.nodes, res.edges);
        } else {
            console.log('[topology] endpoint returned empty, using fallback');
            await buildTopologyFromEndpoints();
        }
    } catch (e) {
        console.warn('[topology] endpoint failed, using fallback:', e);
        await buildTopologyFromEndpoints();
    }
}

function detectGoalCompletions(nodes) {
    for (const n of nodes) {
        if (n.type !== 'goal') continue;
        const prev = prevGoalStatuses.get(n.id);
        const curr = n.status;
        prevGoalStatuses.set(n.id, curr);
        // Transition to completed/failed with a primary_product
        if (prev && prev !== curr && (curr === 'completed' || curr === 'failed')) {
            const product = n.data?.primary_product;
            if (product) {
                highlightGoalCompletion(n.id, product);
            }
        }
    }
}

function highlightGoalCompletion(goalId, primaryProduct) {
    console.log(`[topology] goal ${goalId} completed with product: ${primaryProduct}`);
    highlightCompletion(goalId);
    state.emit('goal-completed-with-product', { goalId, primaryProduct });
}

async function buildTopologyFromEndpoints() {
    const char = state.character;
    if (!char) return;

    const nodes = [];
    const edges = [];

    // Ensure agent node
    nodes.push({ id: 'agent', type: 'agent', label: char, activation: 1.0 });

    // Fetch goals
    try {
        const goalRes = await api.getScheduledGoals(char);
        if (goalRes.success && goalRes.goals) {
            for (const g of goalRes.goals) {
                const goalId = g.goal_id || g.name || `goal_${nodes.length}`;
                const status = g.status || 'ready';
                const label = g.name || (g.goal_text || '').slice(0, 60);

                // Activation: running=1.0, ready=0.5, completed=0.2
                let activation = 0.2;
                if (status === 'running') activation = 1.0;
                else if (status === 'ready' || status === 'pending') activation = 0.5;

                nodes.push({
                    id: goalId,
                    type: 'goal',
                    label,
                    status,
                    activation,
                    data: g,
                });

                // Edge: agent -> running goal (foreground link)
                if (status === 'running') {
                    edges.push({ source: 'agent', target: goalId, type: 'produces' });
                }
            }
        }
    } catch (e) {
        console.warn('Failed to fetch goals:', e);
    }

    // Fetch bindings (skip system-internal _ prefixed only)
    const runningGoal = nodes.find(n => n.type === 'goal' && n.status === 'running');
    try {
        const bindRes = await api.getPlanBindings(char);
        if (bindRes.success && bindRes.bindings) {
            for (const [varName, bindInfo] of Object.entries(bindRes.bindings)) {
                const cleanName = varName.replace(/^\$/, '');
                if (cleanName.startsWith('_')) continue;

                const resourceId = (typeof bindInfo === 'object' && bindInfo?.value)
                    ? bindInfo.value
                    : (typeof bindInfo === 'string' ? bindInfo : '');
                const label = `$${cleanName}`;
                const bindId = `binding_${cleanName}`;
                nodes.push({
                    id: bindId,
                    type: 'binding',
                    label,
                    data: { variable: label, resource_id: resourceId, value_preview: resourceId },
                });
                edges.push({
                    source: runningGoal ? runningGoal.id : 'agent',
                    target: bindId,
                    type: 'binding',
                });
            }
        }
    } catch (e) {
        console.warn('Failed to fetch bindings:', e);
    }

    // Fetch inventory (notes/collections) — only persistent ones
    try {
        const invRes = await api.getInventory(char);
        if (invRes.success && invRes.inventory) {
            for (const item of invRes.inventory) {
                const noteId = item.resource_id || item.name || `note_${nodes.length}`;
                if (noteId.startsWith('_')) continue; // skip system notes
                nodes.push({
                    id: noteId,
                    type: 'note',
                    label: item.name || noteId,
                    activation: 0.3,
                    data: { resource_id: noteId, ...item },
                });
            }
        }
    } catch (e) {
        console.warn('Failed to fetch inventory:', e);
    }

    state.setTopology(nodes, edges);
}

// ── Health Indicator ─────────────────────────────────────

async function refreshHealthIndicator() {
    if (!state.character) return;
    const el = document.getElementById('dock-health');
    if (!el) return;
    try {
        const res = await api.getConcerns(state.character);
        if (!res.success) return;
        const dc = res.derived_concerns || [];
        const activations = res.activations || {};

        // Count concern statuses
        const active = dc.filter(c => c.status === 'active' || c.status === 'surfaced').length;
        const satisfied = dc.filter(c => c.status === 'satisfied').length;
        const total = dc.length;

        // Check if any health-related concern is active with high activation
        const healthConcern = dc.find(c =>
            c.concern_label && c.concern_label.includes('health') && c.status === 'active');
        const healthAct = healthConcern ? (activations[healthConcern.concern_id] || {}).activation || 0 : 0;

        if (healthConcern && healthAct > 0.5) {
            el.textContent = `Health: warning`;
            el.className = 'dock-health health-warning';
        } else if (active > 0) {
            el.textContent = `${active} active, ${satisfied} satisfied`;
            el.className = 'dock-health health-ok';
        } else {
            el.textContent = `${total} concerns`;
            el.className = 'dock-health';
        }
    } catch {
        // silently ignore
    }

    // Resource footprint from topology state
    const fpEl = document.getElementById('dock-footprint');
    if (fpEl) {
        let notes = 0, goals = 0, bindings = 0;
        for (const [, n] of state.nodes) {
            if (n.type === 'note') notes++;
            else if (n.type === 'goal') goals++;
            else if (n.type === 'binding') bindings++;
        }
        fpEl.textContent = `${notes} notes \u00b7 ${goals} goals \u00b7 ${bindings} bindings`;
    }
}

// ── Bindings Refresh ─────────────────────────────────────

async function refreshBindings() {
    if (!state.character) return;
    try {
        const res = await api.getPlanBindings(state.character);
        if (!res.success || !res.bindings) return;

        const newBindings = res.bindings;
        const runningGoal = Array.from(state.nodes.values())
            .find(n => n.type === 'goal' && n.status === 'running');

        // Update all visible bindings (skip system-internal _ prefix only)
        const visibleBindings = new Set();
        for (const [varName, bindInfo] of Object.entries(newBindings)) {
            const cleanName = varName.replace(/^\$/, '');
            if (cleanName.startsWith('_')) continue;

            const bindId = `binding_${cleanName}`;
            visibleBindings.add(bindId);
            const resourceId = (typeof bindInfo === 'object' && bindInfo?.value)
                ? bindInfo.value
                : (typeof bindInfo === 'string' ? bindInfo : '');
            const label = `$${cleanName}`;

            if (!state.nodes.has(bindId)) {
                state.upsertNode(bindId, {
                    type: 'binding',
                    label,
                    data: { variable: label, resource_id: resourceId, value_preview: resourceId },
                });
                state.upsertEdge(runningGoal ? runningGoal.id : 'agent', bindId, 'binding');
                flashNode(bindId, 'up');
            }
        }

        // Remove bindings no longer present
        for (const [id, node] of state.nodes) {
            if (node.type === 'binding' && !visibleBindings.has(id)) {
                state.removeNode(id);
            }
        }

        state.bindings = newBindings;
    } catch {
        // Silently ignore binding refresh failures
    }
}
