/**
 * dock.js — Bottom dock bar: chat, goal entry, controls, system actions.
 */

import { state } from './state.js';
import * as api from './api.js';

// Module-level state for ask-pending alerts
let askPending = false;
let titleFlashInterval = null;
const originalTitle = document.title;

export function initDock() {
    // ── Slide panel toggles ──────────────────────────────
    setupToggle('dock-chat-toggle', 'chat-panel');
    setupToggle('dock-goal-toggle', 'goal-panel');

    // Close buttons on slide panels
    document.querySelectorAll('.slide-panel-close').forEach(btn => {
        btn.addEventListener('click', () => {
            const target = btn.dataset.target;
            closePanel(target);
        });
    });

    // ── Chat ─────────────────────────────────────────────
    const chatInput = document.getElementById('chat-input');
    const chatSend = document.getElementById('chat-send');
    const chatMessages = document.getElementById('chat-messages');

    chatSend.addEventListener('click', () => sendChat());
    chatInput.addEventListener('keydown', (e) => {
        if (e.key === 'Enter' && !e.shiftKey) {
            e.preventDefault();
            sendChat();
        }
    });

    function sendChat() {
        const text = chatInput.value.trim();
        if (!text || !state.character) return;
        state.addChatMessage('user', text);
        api.sendTextInput(state.character, text);
        chatInput.value = '';
    }

    // ── End Conversation ────────────────────────────────
    const chatEnd = document.getElementById('chat-end');
    chatEnd.addEventListener('click', () => {
        if (!state.character) return;
        confirm('End conversation?', () => {
            api.endDialog(state.character);
            state.addChatMessage('system', 'Conversation ended.');
        });
    });

    const chatToggleBtn = document.getElementById('dock-chat-toggle');
    let unreadCount = 0;

    // Request notification permission on load
    if ('Notification' in window && Notification.permission === 'default') {
        Notification.requestPermission();
    }

    state.on('chat-message', (msg) => {
        const div = document.createElement('div');
        div.className = `chat-msg chat-msg-${msg.role}`;
        div.textContent = msg.text;
        chatMessages.appendChild(div);
        chatMessages.scrollTop = chatMessages.scrollHeight;

        // Show unread badge if chat panel is closed and message is from agent
        if (activePanel !== 'chat-panel' && msg.role === 'agent') {
            unreadCount++;
            chatToggleBtn.textContent = `Chat (${unreadCount})`;
            chatToggleBtn.classList.add('has-unread');
        }
    });

    // ── Ask Pending Alerts ────────────────────────────────
    state.on('ask-pending', ({ text, character }) => {
        if (askPending) return;  // already alerting
        askPending = true;

        // 1. Pulse the chat button
        chatToggleBtn.classList.add('ask-pending');

        // 2. Flash the document title
        let flash = false;
        titleFlashInterval = setInterval(() => {
            flash = !flash;
            document.title = flash ? `⚠ ${character || 'Agent'} is asking...` : originalTitle;
        }, 1000);

        // 3. Browser notification (if permitted)
        if ('Notification' in window && Notification.permission === 'granted') {
            const preview = (text || '').slice(0, 120);
            const n = new Notification(`${character || 'Agent'} is asking`, {
                body: preview || 'A question needs your response.',
                icon: '/static/favicon.png',
                tag: 'ask-pending',  // dedup
            });
            n.onclick = () => { window.focus(); n.close(); };
        }
    });

    // ── Goal Entry ───────────────────────────────────────
    const goalInput = document.getElementById('goal-input');
    const goalSubmit = document.getElementById('goal-submit');

    goalSubmit.addEventListener('click', () => submitGoal());
    goalInput.addEventListener('keydown', (e) => {
        if (e.key === 'Enter' && e.ctrlKey) {
            e.preventDefault();
            submitGoal();
        }
    });

    function submitGoal() {
        const text = goalInput.value.trim();
        if (!text || !state.character) return;
        const mode = document.getElementById('goal-mode').value;
        // Send as a goal command
        api.sendTextInput(state.character, `goal: ${text}`);
        goalInput.value = '';
        closePanel('goal-panel');
    }

    // ── Control Buttons ──────────────────────────────────
    document.getElementById('btn-stop').addEventListener('click', () => api.stopTurn());
    document.getElementById('btn-continuous').addEventListener('click', () => api.toggleContinuous());
    document.getElementById('btn-llm').addEventListener('click', () => api.toggleLLM());
    document.getElementById('btn-save').addEventListener('click', () => api.save());

    document.getElementById('btn-shutdown').addEventListener('click', () => {
        confirm('Save and shutdown?', () => api.shutdown());
    });

    // ── Status Updates ───────────────────────────────────
    const statusText = document.getElementById('dock-status-text');

    state.on('connection', ({ connected }) => {
        statusText.textContent = connected ? 'Connected' : 'Reconnecting...';
    });

    state.on('turn-state', () => {
        const ts = state.turnState;
        const parts = [];
        if (state.character) parts.push(state.character);
        parts.push(`Turn ${ts.turnNumber}`);
        parts.push(ts.mode);
        if (ts.continuous) parts.push('continuous');
        statusText.textContent = parts.join(' \u00b7 ');

        // Update button active states
        document.getElementById('btn-continuous').classList.toggle('active', ts.continuous);

        // Update LLM button label
        const llmBtn = document.getElementById('btn-llm');
        if (ts.llmLabel) {
            llmBtn.textContent = ts.llmLabel;
            llmBtn.classList.toggle('active', ts.llmMode === 'alt');
        }
    });

    state.on('character-detected', (char) => {
        statusText.textContent = `${char} \u00b7 Ready`;
    });
}

// ── Panel Management ─────────────────────────────────────

let activePanel = null;

function setupToggle(buttonId, panelId) {
    document.getElementById(buttonId).addEventListener('click', () => {
        if (activePanel === panelId) {
            closePanel(panelId);
        } else {
            if (activePanel) closePanel(activePanel);
            openPanel(panelId);
        }
    });
}

function openPanel(id) {
    const el = document.getElementById(id);
    el.classList.remove('hidden');
    el.classList.add('visible');
    activePanel = id;
    // Clear unread badge when chat opens
    if (id === 'chat-panel') {
        clearChatUnread();
    }
    // Shrink inspector to avoid overlap
    const inspector = document.getElementById('inspector');
    if (inspector) inspector.classList.add('panel-open');
}

function clearChatUnread() {
    const btn = document.getElementById('dock-chat-toggle');
    if (btn) {
        btn.textContent = 'Chat';
        btn.classList.remove('has-unread');
        btn.classList.remove('ask-pending');
    }
    // Clear ask-pending state
    askPending = false;
    if (titleFlashInterval) {
        clearInterval(titleFlashInterval);
        titleFlashInterval = null;
    }
    document.title = originalTitle;
}

function closePanel(id) {
    const el = document.getElementById(id);
    el.classList.remove('visible');
    el.classList.add('hidden');
    if (activePanel === id) activePanel = null;
    // Restore inspector height when no panel is open
    if (!activePanel) {
        const inspector = document.getElementById('inspector');
        if (inspector) inspector.classList.remove('panel-open');
    }
}

// ── Confirmation Dialog ──────────────────────────────────

function confirm(message, onYes) {
    const overlay = document.getElementById('confirm-dialog');
    document.getElementById('confirm-message').textContent = message;
    overlay.classList.remove('hidden');

    const yesBtn = document.getElementById('confirm-yes');
    const noBtn = document.getElementById('confirm-no');

    function cleanup() {
        overlay.classList.add('hidden');
        yesBtn.removeEventListener('click', handleYes);
        noBtn.removeEventListener('click', handleNo);
    }
    function handleYes() { cleanup(); onYes(); }
    function handleNo() { cleanup(); }

    yesBtn.addEventListener('click', handleYes);
    noBtn.addEventListener('click', handleNo);
}
