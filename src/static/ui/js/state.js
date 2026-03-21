/**
 * state.js — Client-side state store for the activation field.
 *
 * Single source of truth. WebSocket handler writes, all other modules read.
 * Emits events on change so the graph can react.
 */

class AppState extends EventTarget {
    constructor() {
        super();
        this.character = '';          // active character name
        this.systemReady = false;
        this.connected = false;

        // Graph topology
        this.nodes = new Map();       // id -> {id, type, label, status, activation, weight, data}
        this.edges = new Map();       // "src->tgt" -> {source, target, type}

        // OODA
        this.oodaPhase = '';
        this.lastTick = null;

        // Turn state
        this.turnState = {
            mode: 'step',
            turnNumber: 0,
            continuous: false,
            goalScheduler: false,
        };

        // Bindings (during active plan execution)
        this.bindings = {};           // $var -> {resource_id, value_preview}

        // Chat messages
        this.chatMessages = [];       // [{role, text, timestamp}]

        // Action log (last N actions for the graph)
        this.recentActions = [];      // [{action_type, text, timestamp, character}]
    }

    // ── Node management ──────────────────────────────────────

    upsertNode(id, props) {
        const existing = this.nodes.get(id);
        if (existing) {
            Object.assign(existing, props);
            this.emit('node-update', existing);
        } else {
            const node = { id, ...props };
            this.nodes.set(id, node);
            this.emit('node-add', node);
        }
    }

    removeNode(id) {
        const node = this.nodes.get(id);
        if (node) {
            this.nodes.delete(id);
            // Remove related edges
            for (const [key, edge] of this.edges) {
                if (edge.source === id || edge.target === id) {
                    this.edges.delete(key);
                }
            }
            this.emit('node-remove', node);
        }
    }

    upsertEdge(source, target, type) {
        const key = `${source}->${target}`;
        if (!this.edges.has(key)) {
            const edge = { source, target, type };
            this.edges.set(key, edge);
            this.emit('edge-add', edge);
        }
    }

    // ── Bulk update from topology endpoint ───────────────────

    setTopology(nodes, edges) {
        // Track existing IDs for removal
        const newIds = new Set(nodes.map(n => n.id));
        const newEdgeKeys = new Set(edges.map(e => `${e.source}->${e.target}`));

        // Remove stale nodes (but keep agent)
        for (const [id, node] of this.nodes) {
            if (!newIds.has(id) && node.type !== 'agent') {
                this.nodes.delete(id);
            }
        }
        // Remove stale edges
        for (const key of this.edges.keys()) {
            if (!newEdgeKeys.has(key)) {
                this.edges.delete(key);
            }
        }
        // Upsert
        for (const n of nodes) {
            this.upsertNode(n.id, n);
        }
        for (const e of edges) {
            this.upsertEdge(e.source, e.target, e.type);
        }
        this.emit('topology-refresh');
    }

    // ── Chat ─────────────────────────────────────────────────

    addChatMessage(role, text) {
        const msg = { role, text, timestamp: new Date().toISOString() };
        this.chatMessages.push(msg);
        if (this.chatMessages.length > 200) {
            this.chatMessages = this.chatMessages.slice(-150);
        }
        this.emit('chat-message', msg);
    }

    // ── Events ───────────────────────────────────────────────

    emit(name, detail) {
        this.dispatchEvent(new CustomEvent(name, { detail }));
    }

    on(name, handler) {
        this.addEventListener(name, (e) => handler(e.detail));
    }
}

export const state = new AppState();
