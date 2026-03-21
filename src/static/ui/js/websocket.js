/**
 * websocket.js — WebSocket connection manager.
 *
 * Connects to the FastAPI backend, handles reconnection,
 * and dispatches incoming messages to update the state store.
 */

import { state } from './state.js';

let ws = null;
let reconnectDelay = 1000;
let pingInterval = null;

export function connect() {
    const proto = location.protocol === 'https:' ? 'wss' : 'ws';
    const url = `${proto}://${location.host}/ws`;

    ws = new WebSocket(url);

    ws.onopen = () => {
        console.log('[ws] connected');
        state.connected = true;
        reconnectDelay = 1000;
        state.emit('connection', { connected: true });

        // Start keepalive
        if (pingInterval) clearInterval(pingInterval);
        pingInterval = setInterval(() => {
            if (ws && ws.readyState === WebSocket.OPEN) {
                ws.send('ping');
            }
        }, 30000);
    };

    ws.onclose = () => {
        console.log('[ws] disconnected, reconnecting...');
        state.connected = false;
        state.emit('connection', { connected: false });
        if (pingInterval) { clearInterval(pingInterval); pingInterval = null; }
        setTimeout(connect, reconnectDelay);
        reconnectDelay = Math.min(reconnectDelay * 1.5, 10000);
    };

    ws.onerror = (e) => {
        console.warn('[ws] error', e);
    };

    ws.onmessage = (event) => {
        if (event.data === 'pong') return;
        try {
            const msg = JSON.parse(event.data);
            dispatch(msg);
        } catch (e) {
            console.warn('[ws] unparseable message', event.data);
        }
    };
}

export function send(data) {
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(typeof data === 'string' ? data : JSON.stringify(data));
    }
}

// ── Message Dispatch ─────────────────────────────────────

function dispatch(msg) {
    const handler = handlers[msg.type];
    if (handler) {
        handler(msg);
    }
}

const handlers = {
    test(msg) {
        console.log('[ws] server:', msg.message);
    },

    ready_state(msg) {
        state.systemReady = msg.system_ready;
        state.emit('system-ready', msg);
    },

    action(msg) {
        // Detect character name
        if (msg.character && !state.character) {
            state.character = msg.character;
            state.emit('character-detected', msg.character);
        }

        // Track recent actions
        state.recentActions.push({
            action_type: msg.action_type,
            text: msg.text || msg.llm_response || '',
            timestamp: msg.timestamp,
            character: msg.character,
            source: msg.source,
            target: msg.target,
            value: msg.value,
            metadata: msg.metadata,
            raw: msg,
        });
        if (state.recentActions.length > 50) {
            state.recentActions = state.recentActions.slice(-30);
        }

        // Chat messages from say/ask actions
        if (msg.action_type === 'say' || msg.action_type === 'ask') {
            const target = msg.target || '';
            if (target.toLowerCase() === 'user' || !target) {
                state.addChatMessage('agent', msg.text || msg.value || '');
            }
        }

        state.emit('action', msg);
    },

    goal(msg) {
        if (msg.character && !state.character) {
            state.character = msg.character;
        }
        state.emit('goal-update', msg);
    },

    current_plan(msg) {
        state.emit('plan-update', msg);
    },

    current_state(msg) {
        state.emit('state-update', msg);
    },

    world_state_update(msg) {
        state.emit('world-state', msg);
    },

    turn_state_update(msg) {
        if (msg.turn) {
            state.turnState.mode = msg.turn.mode || state.turnState.mode;
            state.turnState.turnNumber = msg.turn.turn_number || state.turnState.turnNumber;
        }
        if (msg.buttons) {
            state.turnState.continuous = msg.buttons.continuous || false;
            if (msg.buttons.llm_toggle) {
                state.turnState.llmLabel = msg.buttons.llm_toggle.label || '';
                state.turnState.llmMode = msg.buttons.llm_toggle.mode || '';
            }
        }
        if (msg.goal_scheduler !== undefined) {
            state.turnState.goalScheduler = msg.goal_scheduler;
        }
        if (msg.character && !state.character) {
            state.character = msg.character;
        }
        state.emit('turn-state', msg);
    },

    turn_state(msg) {
        state.turnState.mode = msg.mode || state.turnState.mode;
        state.turnState.turnNumber = msg.turn_number || state.turnState.turnNumber;
        state.emit('turn-state', msg);
    },

    ooda_tick(msg) {
        state.oodaPhase = msg.phase || '';
        state.lastTick = Date.now();
        state.emit('ooda-tick', msg);
    },

    situation_data(msg) {
        state.emit('situation-data', msg);
    },

    time_update() { /* ignore clock sync */ },
    turn_mode_update() { },
    turn_start() { },
    step_complete() { },
    decided_action() { },
};
