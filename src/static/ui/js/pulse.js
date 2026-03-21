/**
 * pulse.js — OODA heartbeat pulse animation.
 *
 * Renders expanding rings from the agent node on each OODA tick.
 * Uses the overlay canvas for smooth animation without affecting D3.
 */

import { state } from './state.js';

let canvas, ctx;
let pulses = [];  // active pulse animations

const PHASE_COLORS = {
    observe: 'rgba(80, 140, 255, 0.4)',
    orient:  'rgba(255, 200, 60, 0.4)',
    decide:  'rgba(255, 140, 40, 0.4)',
    act:     'rgba(60, 230, 120, 0.4)',
    '':      'rgba(200, 200, 220, 0.2)',
};

const PULSE_DURATION = 800;  // ms
const PULSE_MAX_RADIUS = 60;

export function initPulse() {
    canvas = document.getElementById('pulse-canvas');
    ctx = canvas.getContext('2d');

    resize();
    window.addEventListener('resize', resize);

    state.on('ooda-tick', (msg) => {
        const agentNode = state.nodes.get('agent');
        if (!agentNode || agentNode.x == null) return;
        pulses.push({
            x: agentNode.x,
            y: agentNode.y,
            startTime: performance.now(),
            color: PHASE_COLORS[msg.phase] || PHASE_COLORS[''],
        });
    });

    requestAnimationFrame(animate);
}

function resize() {
    canvas.width = window.innerWidth;
    canvas.height = window.innerHeight;
}

function animate(now) {
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // Get current graph transform from the SVG
    const gEl = document.querySelector('.graph-container');
    let tx = 0, ty = 0, scale = 1;
    if (gEl) {
        const transform = gEl.getAttribute('transform');
        if (transform) {
            const m = transform.match(/translate\(([\d.-]+)[,\s]+([\d.-]+)\)/);
            if (m) { tx = parseFloat(m[1]); ty = parseFloat(m[2]); }
            const s = transform.match(/scale\(([\d.-]+)\)/);
            if (s) { scale = parseFloat(s[1]); }
        }
    }

    pulses = pulses.filter(p => {
        const elapsed = now - p.startTime;
        if (elapsed > PULSE_DURATION) return false;

        const progress = elapsed / PULSE_DURATION;
        const radius = PULSE_MAX_RADIUS * progress * scale;
        const alpha = 1 - progress;

        // Transform node coordinates to screen coordinates
        const sx = p.x * scale + tx;
        const sy = p.y * scale + ty;

        ctx.beginPath();
        ctx.arc(sx, sy, radius, 0, Math.PI * 2);
        ctx.strokeStyle = p.color.replace(/[\d.]+\)$/, `${alpha * 0.6})`);
        ctx.lineWidth = 2 * scale;
        ctx.stroke();

        return true;
    });

    requestAnimationFrame(animate);
}

/**
 * Trigger a synthetic pulse (e.g., when topology refreshes without an ooda_tick).
 */
export function triggerPulse() {
    const agentNode = state.nodes.get('agent');
    if (!agentNode || agentNode.x == null) return;
    pulses.push({
        x: agentNode.x,
        y: agentNode.y,
        startTime: performance.now(),
        color: PHASE_COLORS[''],
    });
}
