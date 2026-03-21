/**
 * main.js — Application entry point.
 *
 * Initializes all modules, connects WebSocket, starts topology polling.
 */

import { state } from './state.js';
import { connect } from './websocket.js';
import { initGraph, flashNode } from './graph.js';
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

async function refreshTopology() {
    if (!state.character) return;

    try {
        const res = await api.getTopology(state.character);
        if (res.success && res.nodes && res.edges) {
            state.setTopology(res.nodes, res.edges);
        } else {
            // Fallback: build topology from individual endpoints
            await buildTopologyFromEndpoints();
        }
    } catch {
        // Topology endpoint may not exist yet; fallback
        await buildTopologyFromEndpoints();
    }
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

    // Fetch bindings
    try {
        const bindRes = await api.getPlanBindings(char);
        if (bindRes.success && bindRes.bindings) {
            const runningGoal = nodes.find(n => n.type === 'goal' && n.status === 'running');
            for (const [varName, value] of Object.entries(bindRes.bindings)) {
                const bindId = `binding_${varName}`;
                const resourceId = typeof value === 'string' ? value : (value?.resource_id || '');
                const preview = typeof value === 'string' ? value : JSON.stringify(value).slice(0, 100);
                nodes.push({
                    id: bindId,
                    type: 'binding',
                    label: varName,
                    activation: 0.6,
                    data: { variable: varName, resource_id: resourceId, value_preview: preview },
                });
                // Link binding to running goal
                if (runningGoal) {
                    edges.push({ source: runningGoal.id, target: bindId, type: 'binding' });
                }
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

// ── Bindings Refresh ─────────────────────────────────────

async function refreshBindings() {
    if (!state.character) return;
    try {
        const res = await api.getPlanBindings(state.character);
        if (!res.success || !res.bindings) return;

        const newBindings = res.bindings;
        const runningGoal = Array.from(state.nodes.values())
            .find(n => n.type === 'goal' && n.status === 'running');

        // Add new bindings
        for (const [varName, value] of Object.entries(newBindings)) {
            const bindId = `binding_${varName}`;
            const resourceId = typeof value === 'string' ? value : (value?.resource_id || '');
            const preview = typeof value === 'string' ? value : JSON.stringify(value).slice(0, 100);

            if (!state.nodes.has(bindId)) {
                state.upsertNode(bindId, {
                    type: 'binding',
                    label: varName,
                    activation: 0.8,
                    data: { variable: varName, resource_id: resourceId, value_preview: preview },
                });
                if (runningGoal) {
                    state.upsertEdge(runningGoal.id, bindId, 'binding');
                }
                flashNode(bindId, 'up');
            }
        }

        // Fade bindings no longer present
        for (const [id, node] of state.nodes) {
            if (node.type === 'binding') {
                const varName = node.data?.variable;
                if (varName && !(varName in newBindings)) {
                    state.upsertNode(id, { activation: 0.1 });
                }
            }
        }

        state.bindings = newBindings;
    } catch {
        // Silently ignore binding refresh failures
    }
}
