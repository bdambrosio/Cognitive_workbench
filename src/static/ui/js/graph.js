/**
 * graph.js — D3 force-directed graph for the activation field.
 *
 * Renders nodes (agent, goals, concerns, notes, bindings) and edges
 * on the SVG canvas. Handles zoom, drag, click, and force simulation.
 */

import { state } from './state.js';

let svg, g, simulation;
let nodeGroup, edgeGroup;
let width, height;
let selectedNodeId = null;

// Node size ranges by type
const SIZE = {
    agent:   { min: 22, max: 22 },
    goal:    { min: 10, max: 18 },
    concern: { min: 8, max: 14 },
    note:    { min: 6, max: 10 },
    binding: { min: 5, max: 8 },
};

// Force parameters
const FORCES = {
    charge: -120,
    linkDistance: 80,
    centerStrength: 0.03,
    collideRadius: 30,
};

export function initGraph() {
    svg = d3.select('#activation-field');
    width = window.innerWidth;
    height = window.innerHeight;

    // Zoom behavior
    const zoom = d3.zoom()
        .scaleExtent([0.2, 4])
        .on('zoom', (event) => {
            g.attr('transform', event.transform);
        });
    svg.call(zoom);

    // Click on background to deselect
    svg.on('click', (event) => {
        if (event.target === svg.node()) {
            selectNode(null);
        }
    });

    // Container group for zoom/pan
    g = svg.append('g').attr('class', 'graph-container');

    // Edge layer (below nodes)
    edgeGroup = g.append('g').attr('class', 'edges');
    // Node layer
    nodeGroup = g.append('g').attr('class', 'nodes');

    // Force simulation
    simulation = d3.forceSimulation()
        .force('charge', d3.forceManyBody().strength(FORCES.charge))
        .force('link', d3.forceLink().id(d => d.id).distance(FORCES.linkDistance))
        .force('center', d3.forceCenter(width / 2, height / 2).strength(FORCES.centerStrength))
        .force('collide', d3.forceCollide().radius(FORCES.collideRadius))
        .alphaDecay(0.02)
        .on('tick', ticked);

    // Listen for state changes
    state.on('topology-refresh', () => updateGraph());
    state.on('node-add', () => updateGraph());
    state.on('node-remove', () => updateGraph());
    state.on('node-update', (node) => updateNodeAppearance(node));

    // Handle resize
    window.addEventListener('resize', () => {
        width = window.innerWidth;
        height = window.innerHeight;
        simulation.force('center', d3.forceCenter(width / 2, height / 2).strength(FORCES.centerStrength));
        simulation.alpha(0.1).restart();
    });
}

// ── Graph Update (enter/update/exit) ─────────────────────

function updateGraph() {
    const nodes = Array.from(state.nodes.values());
    const edges = Array.from(state.edges.values()).map(e => ({
        ...e,
        source: typeof e.source === 'string' ? e.source : e.source.id,
        target: typeof e.target === 'string' ? e.target : e.target.id,
    }));

    // Filter edges to only those with existing nodes
    const nodeIds = new Set(nodes.map(n => n.id));
    const validEdges = edges.filter(e => nodeIds.has(e.source) && nodeIds.has(e.target));

    // ── Edges ────────────────────────────────────────────
    const edgeSel = edgeGroup.selectAll('line.edge')
        .data(validEdges, d => `${d.source}->${d.target}`);

    edgeSel.exit().remove();

    const edgeEnter = edgeSel.enter()
        .append('line')
        .attr('class', d => `edge edge-${d.type || 'default'}`);

    const allEdges = edgeEnter.merge(edgeSel);

    // ── Nodes ────────────────────────────────────────────
    const nodeSel = nodeGroup.selectAll('g.node')
        .data(nodes, d => d.id);

    nodeSel.exit()
        .transition().duration(300)
        .attr('opacity', 0)
        .remove();

    const nodeEnter = nodeSel.enter()
        .append('g')
        .attr('class', d => `node ${nodeClass(d)}`)
        .attr('opacity', 0)
        .call(drag(simulation))
        .on('click', (event, d) => {
            event.stopPropagation();
            selectNode(d.id);
        })
        .on('mouseenter', (event, d) => showTooltip(event, d))
        .on('mouseleave', hideTooltip);

    // Add shape per type
    nodeEnter.each(function(d) {
        const el = d3.select(this);
        if (d.type === 'note') {
            el.append('rect')
                .attr('width', nodeSize(d) * 1.6)
                .attr('height', nodeSize(d) * 1.6)
                .attr('x', -nodeSize(d) * 0.8)
                .attr('y', -nodeSize(d) * 0.8)
                .attr('rx', 2);
        } else {
            el.append('circle').attr('r', nodeSize(d));
        }
        el.append('text')
            .attr('dy', nodeSize(d) + 12)
            .text(truncate(d.label || d.id, 36));
    });

    nodeEnter.transition().duration(400).attr('opacity', 1);

    const allNodes = nodeEnter.merge(nodeSel);

    // Update classes and sizes on existing nodes
    allNodes.attr('class', d => `node ${nodeClass(d)}${d.id === selectedNodeId ? ' selected' : ''}`);
    allNodes.select('circle').attr('r', d => nodeSize(d));
    allNodes.select('rect')
        .attr('width', d => nodeSize(d) * 1.6)
        .attr('height', d => nodeSize(d) * 1.6)
        .attr('x', d => -nodeSize(d) * 0.8)
        .attr('y', d => -nodeSize(d) * 0.8);
    allNodes.select('text')
        .attr('dy', d => nodeSize(d) + 12)
        .text(d => truncate(d.label || d.id, 36));

    // Update simulation
    simulation.nodes(nodes);
    simulation.force('link').links(validEdges);
    simulation.alpha(0.3).restart();
}

function ticked() {
    edgeGroup.selectAll('line.edge')
        .attr('x1', d => d.source.x)
        .attr('y1', d => d.source.y)
        .attr('x2', d => d.target.x)
        .attr('y2', d => d.target.y);

    nodeGroup.selectAll('g.node')
        .attr('transform', d => `translate(${d.x},${d.y})`);
}

// ── Node Appearance ──────────────────────────────────────

function nodeClass(d) {
    const t = d.type || 'unknown';
    if (t === 'agent') return 'node-agent';
    if (t === 'goal') return `node-goal-${d.status || 'ready'} ${glowClass(d)}`;
    if (t === 'concern_user') return `node-concern-user ${glowClass(d)}`;
    if (t === 'concern_derived') return `node-concern-derived ${glowClass(d)}`;
    if (t === 'note') return 'node-note';
    if (t === 'binding') return 'node-binding';
    return '';
}

function glowClass(d) {
    const a = d.activation || 0;
    if (a >= 0.6) return 'node-glow-high';
    if (a >= 0.3) return 'node-glow-medium';
    return '';
}

function nodeSize(d) {
    const range = SIZE[d.type] || SIZE.note;
    const scale = d.activation || d.weight || 0.5;
    return range.min + (range.max - range.min) * Math.min(scale, 1);
}

function updateNodeAppearance(node) {
    const sel = nodeGroup.selectAll('g.node')
        .filter(d => d.id === node.id);
    if (sel.empty()) return;
    sel.attr('class', `node ${nodeClass(node)}${node.id === selectedNodeId ? ' selected' : ''}`);
    sel.select('circle')
        .transition().duration(400)
        .attr('r', nodeSize(node));
}

// ── Flash delta animation ────────────────────────────────

export function flashNode(nodeId, direction) {
    const sel = nodeGroup.selectAll('g.node').filter(d => d.id === nodeId);
    if (sel.empty()) return;
    sel.select('circle, rect')
        .classed('delta-up', direction === 'up')
        .classed('delta-down', direction === 'down');
    setTimeout(() => {
        sel.select('circle, rect')
            .classed('delta-up', false)
            .classed('delta-down', false);
    }, 700);
}

// ── Selection ────────────────────────────────────────────

function selectNode(id) {
    selectedNodeId = id;
    nodeGroup.selectAll('g.node')
        .classed('selected', d => d.id === id);
    state.emit('node-selected', id ? state.nodes.get(id) : null);
}

export function getSelectedNodeId() { return selectedNodeId; }

// ── Drag ─────────────────────────────────────────────────

function drag(sim) {
    return d3.drag()
        .on('start', (event, d) => {
            if (!event.active) sim.alphaTarget(0.1).restart();
            d.fx = d.x;
            d.fy = d.y;
        })
        .on('drag', (event, d) => {
            d.fx = event.x;
            d.fy = event.y;
        })
        .on('end', (event, d) => {
            if (!event.active) sim.alphaTarget(0);
            // Keep pinned if user dragged; release with double-click
            // d.fx = null; d.fy = null;
        });
}

// ── Tooltip ──────────────────────────────────────────────

let tooltipEl = null;

function ensureTooltip() {
    if (!tooltipEl) {
        tooltipEl = document.createElement('div');
        tooltipEl.className = 'graph-tooltip';
        document.body.appendChild(tooltipEl);
    }
    return tooltipEl;
}

function showTooltip(event, d) {
    const tip = ensureTooltip();
    let text = d.label || d.id;
    if (d.status) text += ` [${d.status}]`;
    if (d.activation != null) text += ` — activation: ${d.activation}`;
    if (d.weight != null) text += ` — weight: ${d.weight}`;
    tip.textContent = text;
    tip.classList.add('visible');
    tip.style.left = (event.pageX + 12) + 'px';
    tip.style.top = (event.pageY - 8) + 'px';
}

function hideTooltip() {
    if (tooltipEl) tooltipEl.classList.remove('visible');
}

// ── Helpers ──────────────────────────────────────────────

function truncate(str, len) {
    if (!str) return '';
    return str.length > len ? str.slice(0, len - 1) + '\u2026' : str;
}
