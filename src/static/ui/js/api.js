/**
 * api.js — REST API wrapper for all backend endpoints.
 */

async function post(url, body = {}) {
    const res = await fetch(url, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(body),
    });
    return res.json();
}

async function get(url) {
    const res = await fetch(url);
    return res.json();
}

// ── Chat / Input ──────────────────────────────────────────

export function sendTextInput(character, message) {
    return post('/api/text_input', { character, message });
}

export function interrupt(character) {
    return post('/api/interrupt', { character });
}

export function endDialog(character) {
    return post('/api/end_dialog', { character });
}

// ── Execution Control ─────────────────────────────────────

export function stopTurn() { return post('/api/turn/stop'); }
export function toggleContinuous() { return post('/api/turn/continuous'); }
export function toggleLLM() { return post('/api/llm/toggle'); }

// ── Goals ─────────────────────────────────────────────────

export function getScheduledGoals(character) {
    return get(`/api/scheduled_goals/${encodeURIComponent(character)}`);
}

export function goalScheduler(character, data) {
    return post(`/api/goal_scheduler/${encodeURIComponent(character)}`, data);
}

export function goalScheduleMode(character, data) {
    return post(`/api/goal_schedule_mode/${encodeURIComponent(character)}`, data);
}

export function goalRename(character, data) {
    return post(`/api/goal_rename/${encodeURIComponent(character)}`, data);
}

export function goalExecutionMode(character, data) {
    return post(`/api/goal_execution_mode/${encodeURIComponent(character)}`, data);
}

export function goalTextUpdate(character, data) {
    return post(`/api/goal_text_update/${encodeURIComponent(character)}`, data);
}

export function goalInterrupt(character, data) {
    return post(`/api/goal_interrupt/${encodeURIComponent(character)}`, data);
}

export function goalRemove(character, data) {
    return post(`/api/goal_remove/${encodeURIComponent(character)}`, data);
}

// ── Data Retrieval ────────────────────────────────────────

export function getPlanBindings(character) {
    return get(`/api/plan_bindings/${encodeURIComponent(character)}`);
}

export function getWorldState(character) {
    return get(`/api/world_state/${encodeURIComponent(character)}`);
}

export function getResource(resourceId) {
    return get(`/api/resource/${encodeURIComponent(resourceId)}`);
}

export function getInventory(character) {
    return get(`/api/character/${encodeURIComponent(character)}/inventory`);
}

export function getTopology(character) {
    return get(`/api/topology/${encodeURIComponent(character)}`);
}

export function getCharacters() {
    return get('/api/characters');
}

export function getOodaFeed(character) {
    return get(`/api/ooda_feed/${encodeURIComponent(character)}`);
}

export function getConcerns(character) {
    return get(`/api/concerns/${encodeURIComponent(character)}`);
}

// ── System ────────────────────────────────────────────────

export function save() { return post('/api/save'); }
export function shutdown() { return post('/api/save_and_shutdown'); }
