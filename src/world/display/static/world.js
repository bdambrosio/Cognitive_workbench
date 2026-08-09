// world.js — renders the world the Python server owns.
//
// Nothing here generates terrain. The heightmap arrives over the socket and
// is sampled with the same bilinear formula as terrain.height_at(), so what
// Bruce walks on and what Jill reasons about are the same surface.

import * as THREE from './vendor/three.module.js';

const EYE_H = 1.62;          // camera height when first-person
const BODY_H = 1.75;
const WALK = 1.7, RUN = 3.4; // m/s, matching WALK_SPEED_MS server-side
const SEND_HZ = 20;

let terrain = null;          // {heights, gridN, extent, waterY}
let me = null;               // our occupant name
const avatars = new Map();   // name -> {group, target, nameplate}

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x9dc4e0);
// Haze, not concealment — entity visibility is decided server-side and
// filtered out of the state stream. Tuned to the 384 m world so distant
// terrain reads as distance rather than as a wall of sky.
scene.fog = new THREE.Fog(0x9dc4e0, 90, 360);

const camera = new THREE.PerspectiveCamera(
  70, innerWidth / innerHeight, 0.1, 900);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.setSize(innerWidth, innerHeight);
renderer.shadowMap.enabled = false;
document.body.appendChild(renderer.domElement);

addEventListener('resize', () => {
  camera.aspect = innerWidth / innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(innerWidth, innerHeight);
});

// Lambert under a hemisphere alone reads muddy at ground level; the sun
// carries most of the contrast and the ambient keeps shadowed sides legible.
scene.add(new THREE.HemisphereLight(0xcfe4f5, 0x5d6b45, 2.0));
const sun = new THREE.DirectionalLight(0xfff4e0, 2.2);
sun.position.set(60, 90, 30);
scene.add(sun);

// ---------------------------------------------------------------- terrain

function heightAt(x, z) {
  if (!terrain) return 0;
  const { heights, gridN, extent } = terrain;
  const half = extent / 2;
  let gx = ((x + half) / extent) * (gridN - 1);
  let gz = ((z + half) / extent) * (gridN - 1);
  gx = Math.min(Math.max(gx, 0), gridN - 1);
  gz = Math.min(Math.max(gz, 0), gridN - 1);
  const x0 = Math.floor(gx), z0 = Math.floor(gz);
  const x1 = Math.min(x0 + 1, gridN - 1), z1 = Math.min(z0 + 1, gridN - 1);
  const tx = gx - x0, tz = gz - z0;
  const h00 = heights[z0 * gridN + x0], h01 = heights[z0 * gridN + x1];
  const h10 = heights[z1 * gridN + x0], h11 = heights[z1 * gridN + x1];
  return (h00 * (1 - tx) + h01 * tx) * (1 - tz) +
         (h10 * (1 - tx) + h11 * tx) * tz;
}

function buildTerrain(msg) {
  const bin = atob(msg.heights_b64);
  const bytes = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) bytes[i] = bin.charCodeAt(i);
  const heights = new Float32Array(bytes.buffer);
  terrain = { heights, gridN: msg.grid_n, extent: msg.extent_m,
              waterY: msg.water_y };

  const n = msg.grid_n, extent = msg.extent_m;
  const geo = new THREE.PlaneGeometry(extent, extent, n - 1, n - 1);
  geo.rotateX(-Math.PI / 2);
  const pos = geo.attributes.position;
  const colors = new Float32Array(pos.count * 3);

  const grass = new THREE.Color(0x6f8f4a);
  const meadow = new THREE.Color(0x93a959);
  const rock = new THREE.Color(0x8a8578);
  const sand = new THREE.Color(0xbfae82);
  const c = new THREE.Color();

  for (let i = 0; i < pos.count; i++) {
    const h = heights[i];
    pos.setY(i, h);
    // Slope from the grid directly — cheaper than sampling height four times.
    const ix = i % n, iz = (i / n) | 0;
    const hx = heights[iz * n + Math.min(ix + 1, n - 1)] - h;
    const hz = heights[Math.min(iz + 1, n - 1) * n + ix] - h;
    const slope = Math.hypot(hx, hz) / (extent / (n - 1));
    c.copy(h < msg.water_y + 0.6 ? sand : meadow);
    if (h >= msg.water_y + 0.6) c.lerp(grass, Math.min(h / 12, 1));
    if (slope > 0.45) c.lerp(rock, Math.min((slope - 0.45) * 1.8, 1));
    colors[i * 3] = c.r; colors[i * 3 + 1] = c.g; colors[i * 3 + 2] = c.b;
  }
  geo.setAttribute('color', new THREE.BufferAttribute(colors, 3));
  geo.computeVertexNormals();
  scene.add(new THREE.Mesh(
    geo, new THREE.MeshLambertMaterial({ vertexColors: true })));

  // Water
  const water = new THREE.Mesh(
    new THREE.PlaneGeometry(extent, extent),
    new THREE.MeshLambertMaterial({ color: 0x3f6f8f, transparent: true,
                                    opacity: 0.75 }));
  water.rotation.x = -Math.PI / 2;
  water.position.y = msg.water_y;
  scene.add(water);

  scatter(msg.trees, msg.rocks);
}

// One InstancedMesh per part: trunks shared, foliage split by species.
function scatter(trees, rocks) {
  const m = new THREE.Matrix4(), q = new THREE.Quaternion();
  const v = new THREE.Vector3(), s = new THREE.Vector3();
  const place = (mesh, i, t, yOff, scale) => {
    q.setFromAxisAngle(new THREE.Vector3(0, 1, 0), t.r);
    v.set(t.x, t.y + yOff * t.s, t.z);
    s.set(t.s * scale, t.s * scale, t.s * scale);
    mesh.setMatrixAt(i, m.compose(v, q, s));
  };

  const trunkGeo = new THREE.CylinderGeometry(0.16, 0.24, 3.2, 5);
  const trunkMat = new THREE.MeshLambertMaterial({ color: 0x5b432c });
  const trunks = new THREE.InstancedMesh(trunkGeo, trunkMat, trees.length);

  const pines = trees.filter(t => t.k === 0);
  const broad = trees.filter(t => t.k === 1);
  const pineMesh = new THREE.InstancedMesh(
    new THREE.ConeGeometry(1.5, 5.2, 7),
    new THREE.MeshLambertMaterial({ color: 0x2f5434 }), pines.length);
  const broadMesh = new THREE.InstancedMesh(
    new THREE.IcosahedronGeometry(1.9, 0),
    new THREE.MeshLambertMaterial({ color: 0x4a7a3c }), broad.length);

  trees.forEach((t, i) => place(trunks, i, t, 1.6, 1));
  pines.forEach((t, i) => place(pineMesh, i, t, 4.6, 1));
  broad.forEach((t, i) => place(broadMesh, i, t, 4.2, 1));

  const rockMesh = new THREE.InstancedMesh(
    new THREE.DodecahedronGeometry(0.7, 0),
    new THREE.MeshLambertMaterial({ color: 0x7d7a70 }), rocks.length);
  rocks.forEach((t, i) => place(rockMesh, i, t, 0.25, 1));

  for (const mesh of [trunks, pineMesh, broadMesh, rockMesh]) {
    mesh.instanceMatrix.needsUpdate = true;
    mesh.frustumCulled = false;      // instances span the whole world
    scene.add(mesh);
  }
}

// ---------------------------------------------------------------- markers

const markers = new Map();          // id -> Group

function markerMesh(m) {
  const group = new THREE.Group();
  const post = new THREE.Mesh(
    new THREE.CylinderGeometry(0.06, 0.06, 1.9, 6),
    new THREE.MeshLambertMaterial({ color: 0x6b5a33 }));
  post.position.y = 0.95;
  group.add(post);
  const flag = new THREE.Mesh(
    new THREE.ConeGeometry(0.34, 0.7, 4),
    new THREE.MeshLambertMaterial({ color: 0xffd34d }));
  flag.position.y = 2.05;
  group.add(flag);
  group.add(nameplate(m.label, '#ffd34d'));
  group.position.set(m.x, m.y, m.z);
  scene.add(group);
  return group;
}

function applyMarkers(list) {
  const seen = new Set();
  for (const m of list || []) {
    seen.add(m.id);
    if (!markers.has(m.id)) markers.set(m.id, markerMesh(m));
  }
  for (const [id, g] of markers) {
    if (!seen.has(id)) { scene.remove(g); markers.delete(id); }
  }
}

// ---------------------------------------------------------------- avatars

function nameplate(text, color) {
  const cv = document.createElement('canvas');
  cv.width = 256; cv.height = 64;
  const g = cv.getContext('2d');
  g.font = 'bold 34px system-ui, sans-serif';
  g.textAlign = 'center'; g.textBaseline = 'middle';
  g.lineWidth = 6; g.strokeStyle = 'rgba(0,0,0,.75)';
  g.strokeText(text, 128, 32);
  g.fillStyle = color; g.fillText(text, 128, 32);
  const sprite = new THREE.Sprite(new THREE.SpriteMaterial({
    map: new THREE.CanvasTexture(cv), transparent: true }));
  // depthTest stays on: with it off a nameplate reads through a hill, and
  // the human could identify someone the fog has already hidden.
  sprite.scale.set(3.2, 0.8, 1);
  sprite.position.y = BODY_H + 0.75;
  return sprite;
}

function makeAvatar(o) {
  const group = new THREE.Group();
  const body = new THREE.Mesh(
    new THREE.CapsuleGeometry(0.32, BODY_H - 0.64, 6, 12),
    new THREE.MeshLambertMaterial({ color: new THREE.Color(o.color) }));
  body.position.y = BODY_H / 2;
  group.add(body);
  // A small brow marks which way they are facing, readable from a distance.
  const brow = new THREE.Mesh(
    new THREE.BoxGeometry(0.42, 0.12, 0.12),
    new THREE.MeshLambertMaterial({ color: 0x21262c }));
  brow.position.set(0, BODY_H - 0.28, 0.3);
  group.add(brow);
  const plate = nameplate(o.name, o.color);
  group.add(plate);
  // Start where they actually are. Leaving this at the origin makes an
  // occupant you have only just seen glide in across the whole map.
  group.position.set(o.x, o.y, o.z);
  group.rotation.y = o.heading;
  scene.add(group);
  return { group, plate, target: new THREE.Vector3(o.x, o.y, o.z),
           heading: o.heading };
}

function applyState(occupants) {
  const seen = new Set();
  for (const o of occupants) {
    seen.add(o.name);
    let a = avatars.get(o.name);
    if (!a) { a = makeAvatar(o); avatars.set(o.name, a); }
    if (o.name === me) { a.group.visible = thirdPerson; continue; }
    a.target.set(o.x, o.y, o.z);
    a.heading = o.heading;
  }
  for (const [name, a] of avatars) {
    if (!seen.has(name)) { scene.remove(a.group); avatars.delete(name); }
  }
}

// ---------------------------------------------------------------- controls

const player = { x: 0, z: 0, yaw: 0, pitch: 0 };
const keys = new Set();
let thirdPerson = true;
let pointerLocked = false;

const veil = document.getElementById('veil');
const hud = document.getElementById('hud');

veil.addEventListener('click', () => renderer.domElement.requestPointerLock());
document.addEventListener('pointerlockchange', () => {
  pointerLocked = document.pointerLockElement === renderer.domElement;
  veil.classList.toggle('hidden', pointerLocked);
});
addEventListener('mousemove', (e) => {
  if (!pointerLocked) return;
  player.yaw -= e.movementX * 0.0022;
  player.pitch -= e.movementY * 0.0022;
  player.pitch = Math.max(-1.2, Math.min(1.2, player.pitch));
});
let markSeq = 0;
addEventListener('keydown', (e) => {
  keys.add(e.code);
  if (e.code === 'KeyF') thirdPerson = !thirdPerson;
  if (e.code === 'KeyM' && ws && ws.readyState === 1 && me) {
    // Numbered rather than typed: prompting for a label would mean
    // leaving pointer lock mid-stride. The marker carries the place; what
    // it means is for the conversation.
    ws.send(JSON.stringify({ type: 'mark', name: me,
                             label: `${me}'s mark ${++markSeq}` }));
  }
});
addEventListener('keyup', (e) => keys.delete(e.code));

function movePlayer(dt) {
  if (!terrain) return;
  let fwd = 0, strafe = 0;
  if (keys.has('KeyW')) fwd += 1;
  if (keys.has('KeyS')) fwd -= 1;
  if (keys.has('KeyD')) strafe += 1;
  if (keys.has('KeyA')) strafe -= 1;
  if (!fwd && !strafe) return;
  const len = Math.hypot(fwd, strafe);
  const speed = (keys.has('ShiftLeft') || keys.has('ShiftRight') ? RUN : WALK) * dt;
  // yaw 0 looks along +z, matching the server's heading convention.
  // Strafe is negated against the naive form: three.js cameras look down
  // their local -z, so with forward = +z the screen-right direction is
  // world -x. Getting this sign wrong swaps A and D.
  const sin = Math.sin(player.yaw), cos = Math.cos(player.yaw);
  const nx = player.x + ((fwd / len) * sin - (strafe / len) * cos) * speed;
  const nz = player.z + ((fwd / len) * cos + (strafe / len) * sin) * speed;
  const half = terrain.extent / 2 - 1;
  if (nx < -half || nx > half || nz < -half || nz > half) return;
  if (heightAt(nx, nz) < terrain.waterY) return;   // don't wade in
  player.x = nx; player.z = nz;
}

function updateCamera() {
  const groundY = heightAt(player.x, player.z);
  if (thirdPerson) {
    const dist = 5.2, lift = 2.4;
    const cx = player.x - Math.sin(player.yaw) * dist;
    const cz = player.z - Math.cos(player.yaw) * dist;
    camera.position.set(cx, Math.max(heightAt(cx, cz), groundY) + lift, cz);
    camera.lookAt(player.x, groundY + 1.2, player.z);
    camera.rotateX(player.pitch * 0.5);
    const a = avatars.get(me);
    if (a) {
      a.group.position.set(player.x, groundY, player.z);
      a.group.rotation.y = player.yaw;
      a.group.visible = true;
      // Your own nameplate would hang in the middle of your own view.
      a.plate.visible = false;
    }
  } else {
    camera.position.set(player.x, groundY + EYE_H, player.z);
    // +PI because a three.js camera faces its local -z: without it, yaw 0
    // would look along world -z while W walked toward +z, so first-person
    // would run backwards relative to the view.
    camera.rotation.set(player.pitch, player.yaw + Math.PI, 0, 'YXZ');
    const a = avatars.get(me);
    if (a) a.group.visible = false;
  }
}

// ------------------------------------------------------------------ socket

let ws = null;
async function connect() {
  let wsPort = 8790;
  try {
    const h = await (await fetch('/health')).json();
    if (h.ws_port) wsPort = h.ws_port;
  } catch (e) { /* fall back to the default port */ }

  ws = new WebSocket(`ws://${location.hostname || '127.0.0.1'}:${wsPort}`);
  ws.onmessage = (ev) => {
    const msg = JSON.parse(ev.data);
    if (msg.type === 'terrain') {
      me = msg.you;
      buildTerrain(msg);
      return;
    }
    if (msg.occupants) {
      applyState(msg.occupants);
      applyMarkers(msg.markers);
      const self = msg.occupants.find(o => o.name === me);
      if (self && !placed) {
        player.x = self.x; player.z = self.z; placed = true;
      }
    }
  };
  ws.onclose = () => setTimeout(connect, 1500);
}
let placed = false;
connect();

let lastSend = 0;
function sendInput(now) {
  if (!ws || ws.readyState !== 1 || !me) return;
  if (now - lastSend < 1000 / SEND_HZ) return;
  lastSend = now;
  ws.send(JSON.stringify({ type: 'input', name: me,
                           x: player.x, z: player.z, heading: player.yaw }));
}

// -------------------------------------------------------------------- loop

let prev = performance.now();
function frame(now) {
  const dt = Math.min((now - prev) / 1000, 0.1);
  prev = now;

  movePlayer(dt);
  // Others glide toward their last reported position: server state lands at
  // 20 Hz, the display runs faster, and un-smoothed updates read as stutter.
  for (const [name, a] of avatars) {
    if (name === me) continue;
    a.group.position.lerp(a.target, Math.min(dt * 9, 1));
    const d = a.heading - a.group.rotation.y;
    a.group.rotation.y += Math.atan2(Math.sin(d), Math.cos(d)) * Math.min(dt * 9, 1);
  }
  updateCamera();
  sendInput(now);

  if (terrain) {
    const others = [...avatars.keys()].filter(n => n !== me);
    hud.innerHTML =
      `<b>${me ?? '—'}</b> &nbsp;(${player.x.toFixed(0)}, ${player.z.toFixed(0)})` +
      ` &nbsp;·&nbsp; ${thirdPerson ? 'third' : 'first'}-person` +
      (others.length ? ` &nbsp;·&nbsp; with ${others.join(', ')}` : '');
  }
  renderer.render(scene, camera);
  requestAnimationFrame(frame);
}
requestAnimationFrame(frame);
