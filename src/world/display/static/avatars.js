// avatars.js — what an occupant looks like.
//
// Abstract, low-poly creature forms rather than portraits: a handful of
// primitives each, readable at 40 m and through fog, which is the range
// most encounters happen at. Split out of world.js so the shapes can be
// rendered and looked at on their own (tools/world_avatar_preview.html)
// instead of only ever being seen from inside a running session.
//
// All three stand roughly BODY_H tall and face +z. Both matter: the server
// gives every occupant the same walk speed and eye height, and heading 0
// means +z, so a beak, muzzle or pair of eyes pointing +z is what tells
// you which way someone is looking. Every rig reports headTop so the
// nameplate sits just above the head instead of at a fixed height.
//
// Identity: `color` is assigned by join order server-side, so it is not a
// stable per-character hue and cannot carry species. Each rig wears it as
// a collar instead — species says who someone is, the collar keeps the
// colour agreeing with the nameplate.

import * as THREE from './vendor/three.module.js';

export const BODY_H = 1.75;

function part(geo, color, x, y, z) {
  const m = new THREE.Mesh(geo, new THREE.MeshLambertMaterial({ color }));
  m.position.set(x, y, z);
  return m;
}

// Cones and cylinders are built along +y; +PI/2 about x turns +y into +z,
// which is the direction everything facing-related has to point.
const FACE_FORWARD = Math.PI / 2;

function collar(group, color, y, r) {
  const ring = part(new THREE.TorusGeometry(r, 0.035, 5, 14), color, 0, y, 0);
  ring.rotation.x = FACE_FORWARD;
  group.add(ring);
  return ring;
}

function legs(group, color, y, h, spread) {
  for (const s of [-1, 1]) {
    group.add(part(new THREE.CylinderGeometry(0.045, 0.04, h, 5),
                   color, s * spread, y, 0));
  }
}

// ---------------------------------------------------------------- crow

function crow(accent) {
  const g = new THREE.Group();
  // Not near-black: at 0x191c24 the whole bird read as one silhouette with
  // no features at all. Slate with a blue sheen keeps it unmistakably the
  // darkest of the three while letting the head and wings separate, and the
  // beak is pale on purpose — it is the facing cue and has to be visible
  // against the body from any angle.
  const FEATHER = 0x2b3140, SHEEN = 0x3c4a66, HORN = 0xd8c39a;
  legs(g, 0x2a2620, 0.31, 0.62, 0.11);
  const body = part(new THREE.CapsuleGeometry(0.29, 0.54, 5, 10),
                    FEATHER, 0, 0.98, 0);
  body.rotation.x = 0.20;              // corvids carry the chest forward
  g.add(body);
  collar(g, accent, 1.24, 0.33);
  g.add(part(new THREE.SphereGeometry(0.225, 12, 10), FEATHER, 0, 1.46, 0.06));
  const beak = part(new THREE.ConeGeometry(0.09, 0.44, 4), HORN, 0, 1.44, 0.34);
  beak.rotation.x = FACE_FORWARD;
  g.add(beak);
  for (const s of [-1, 1]) {
    g.add(part(new THREE.SphereGeometry(0.05, 8, 6), 0xf4e9c8,
               s * 0.105, 1.53, 0.20));
    const wing = part(new THREE.BoxGeometry(0.08, 0.62, 0.30),
                      SHEEN, s * 0.29, 0.98, -0.03);
    wing.rotation.z = s * 0.15;
    wing.rotation.x = -0.10;
    g.add(wing);
  }
  // Fan tail, angled down — the long counterweight to the beak, and the
  // clearest crow cue from behind, which is how you mostly see someone.
  const tail = part(new THREE.BoxGeometry(0.34, 0.05, 0.68),
                    FEATHER, 0, 0.78, -0.46);
  tail.rotation.x = -0.26;
  g.add(tail);
  return { rig: g, headTop: 1.70, sway: null };
}

// -------------------------------------------------------------- kitten

function kitten(accent) {
  const g = new THREE.Group();
  const FUR = 0xd7a877, DARK = 0x8d6748, NOSE = 0xdd8b98;
  legs(g, DARK, 0.30, 0.60, 0.12);
  // Body narrower than the head so the two read as separate masses. At
  // equal widths the capsule and the sphere merged into one lozenge and the
  // kitten lost its head entirely.
  const body = part(new THREE.CapsuleGeometry(0.26, 0.52, 5, 10),
                    FUR, 0, 0.94, 0);
  g.add(body);
  collar(g, accent, 1.20, 0.30);
  const HEAD_Y = 1.46, HEAD_R = 0.31, HEAD_Z = 0.03;
  g.add(part(new THREE.SphereGeometry(HEAD_R, 12, 10), FUR, 0, HEAD_Y, HEAD_Z));
  // Face features have to sit ON the sphere, not inside it: at z 0.235 with
  // a 0.295 head they were swallowed whole and the kitten had no face.
  const FACE_Z = HEAD_Z + HEAD_R * 0.86;
  for (const s of [-1, 1]) {
    const ear = part(new THREE.ConeGeometry(0.12, 0.32, 4),
                     FUR, s * 0.16, HEAD_Y + 0.28, 0.01);
    ear.rotation.z = s * 0.24;
    g.add(ear);
    const inner = part(new THREE.ConeGeometry(0.062, 0.17, 4),
                       NOSE, s * 0.16, HEAD_Y + 0.27, 0.055);
    inner.rotation.z = s * 0.24;
    g.add(inner);
    g.add(part(new THREE.SphereGeometry(0.055, 8, 6), 0x2f4636,
               s * 0.125, HEAD_Y + 0.07, FACE_Z - 0.02));
  }
  const muzzle = part(new THREE.SphereGeometry(0.14, 10, 8),
                      0xf2e6d4, 0, HEAD_Y - 0.07, FACE_Z - 0.03);
  muzzle.scale.set(1.1, 0.78, 0.85);
  g.add(muzzle);
  g.add(part(new THREE.SphereGeometry(0.048, 8, 6), NOSE,
             0, HEAD_Y - 0.045, FACE_Z + 0.08));
  for (const s of [-1, 1]) {   // whiskers: two flat slivers, cheap and legible
    const w = part(new THREE.BoxGeometry(0.34, 0.012, 0.012),
                   0xf6efe4, s * 0.20, HEAD_Y - 0.07, FACE_Z + 0.02);
    w.rotation.z = s * 0.12;
    g.add(w);
  }
  // Tail as a torus arc: a straight cylinder reads as a stick, and the
  // curl is most of what says cat at a distance. Held in the YZ plane so
  // it rises behind the body, and swayed while walking.
  const tail = part(new THREE.TorusGeometry(0.30, 0.045, 5, 14, 2.2),
                    FUR, 0, 0.98, -0.28);
  tail.rotation.y = FACE_FORWARD;
  tail.rotation.z = -0.75;
  g.add(tail);
  return { rig: g, headTop: 1.80, sway: tail };
}

// ----------------------------------------------------------------- owl

function owl(accent) {
  const g = new THREE.Group();
  const PLUME = 0x8a7862, DISC = 0xc4b39a, HORN = 0x3a3126, IRIS = 0xe8b03a;
  legs(g, HORN, 0.30, 0.60, 0.13);
  // Squat and wide: an owl at rest is mostly head and chest, which also
  // keeps it distinguishable from the crow in silhouette at range.
  const body = part(new THREE.CapsuleGeometry(0.37, 0.44, 5, 10),
                    PLUME, 0, 1.00, 0);
  g.add(body);
  collar(g, accent, 1.22, 0.41);
  const head = part(new THREE.SphereGeometry(0.36, 14, 10), PLUME, 0, 1.46, 0);
  head.scale.set(1, 0.92, 0.95);
  g.add(head);
  for (const s of [-1, 1]) {
    // Facial discs: the unmistakable owl cue, and a facing indicator
    // twice the size of the crow's beak.
    const disc = part(new THREE.CylinderGeometry(0.145, 0.145, 0.05, 14),
                      DISC, s * 0.145, 1.48, 0.26);
    disc.rotation.x = FACE_FORWARD;
    g.add(disc);
    const eye = part(new THREE.CylinderGeometry(0.085, 0.085, 0.04, 12),
                     IRIS, s * 0.145, 1.48, 0.295);
    eye.rotation.x = FACE_FORWARD;
    g.add(eye);
    const pupil = part(new THREE.CylinderGeometry(0.038, 0.038, 0.03, 10),
                       0x14100c, s * 0.145, 1.48, 0.315);
    pupil.rotation.x = FACE_FORWARD;
    g.add(pupil);
    const tuft = part(new THREE.ConeGeometry(0.075, 0.24, 4),
                      PLUME, s * 0.225, 1.76, -0.02);
    tuft.rotation.z = s * 0.30;
    g.add(tuft);
    const wing = part(new THREE.BoxGeometry(0.10, 0.52, 0.42),
                      0x7a6a55, s * 0.345, 0.98, -0.04);
    wing.rotation.z = s * 0.06;
    wing.rotation.y = s * -0.28;          // wraps the chest, not a flat slab
    g.add(wing);
  }
  const beak = part(new THREE.ConeGeometry(0.062, 0.17, 4), HORN, 0, 1.36, 0.30);
  beak.rotation.x = FACE_FORWARD;
  g.add(beak);
  const tail = part(new THREE.BoxGeometry(0.30, 0.05, 0.34), PLUME,
                    0, 0.70, -0.30);
  tail.rotation.x = -0.34;
  g.add(tail);
  return { rig: g, headTop: 1.78, sway: null };
}

// ---------------------------------------------------------------- hare

function hare(accent) {
  const g = new THREE.Group();
  const FUR = 0x9a8f7d, DARK = 0x6b6152, INNER = 0xd9a1a6, SCUT = 0xf0ece1;
  // Low and haunched, with the height spent on ears instead of on the
  // body. That inverts the owl's mass distribution — squat body, tall
  // head — so the two stay apart in silhouette even though both are
  // shorter-bodied than the crow.
  legs(g, DARK, 0.26, 0.52, 0.14);
  const body = part(new THREE.CapsuleGeometry(0.29, 0.40, 5, 10),
                    FUR, 0, 0.78, -0.02);
  body.rotation.x = -0.18;            // haunches down, chest up
  g.add(body);
  collar(g, accent, 1.04, 0.28);
  const HEAD_Y = 1.28, HEAD_R = 0.26, HEAD_Z = 0.06;
  g.add(part(new THREE.SphereGeometry(HEAD_R, 12, 10), FUR, 0, HEAD_Y, HEAD_Z));
  const FACE_Z = HEAD_Z + HEAD_R * 0.86;
  // The ears are the whole point: at 26 m they are what separates a hare
  // from a cat, whose cones are a third this length. Splayed slightly so
  // they read as two from the front rather than merging into one mass.
  for (const s of [-1, 1]) {
    const ear = part(new THREE.ConeGeometry(0.098, 0.54, 5),
                     FUR, s * 0.13, HEAD_Y + 0.42, -0.02);
    ear.rotation.z = s * 0.20;
    ear.rotation.x = -0.10;
    g.add(ear);
    const inner = part(new THREE.ConeGeometry(0.05, 0.34, 5),
                       INNER, s * 0.132, HEAD_Y + 0.40, 0.026);
    inner.rotation.z = s * 0.20;
    inner.rotation.x = -0.10;
    g.add(inner);
    // Eyes sit far around the side — a prey animal's field of view, and
    // the cheapest way to not read as another cat. Facing is carried by
    // the muzzle instead, which is why that stays dead centre.
    g.add(part(new THREE.SphereGeometry(0.052, 8, 6), 0x241f1a,
               s * 0.215, HEAD_Y + 0.05, HEAD_Z + 0.09));
  }
  const muzzle = part(new THREE.SphereGeometry(0.115, 10, 8),
                      SCUT, 0, HEAD_Y - 0.09, FACE_Z - 0.03);
  muzzle.scale.set(0.95, 0.8, 0.9);
  g.add(muzzle);
  g.add(part(new THREE.SphereGeometry(0.04, 8, 6), INNER,
             0, HEAD_Y - 0.065, FACE_Z + 0.06));
  // Scut: small, pale, and high on the rump — visible from behind, which
  // is the view you mostly get of someone walking away from you.
  const scut = part(new THREE.SphereGeometry(0.10, 8, 6),
                    SCUT, 0, 0.86, -0.30);
  scut.scale.set(1.0, 0.9, 0.7);
  g.add(scut);
  // No sway: animateRig forces its sway object to a -0.75 z baseline,
  // which is the kitten tail's rest angle and nonsense for anything else.
  return { rig: g, headTop: 1.86, sway: null };
}

// ------------------------------------------------------------- default

// Anyone not named below keeps the shape this world always had, so an
// unexpected occupant renders as something rather than nothing.
function person(accent) {
  const g = new THREE.Group();
  const body = part(new THREE.CapsuleGeometry(0.32, BODY_H - 0.64, 6, 12),
                    accent, 0, BODY_H / 2, 0);
  g.add(body);
  g.add(part(new THREE.BoxGeometry(0.42, 0.12, 0.12), 0x21262c,
             0, BODY_H - 0.28, 0.30));
  return { rig: g, headTop: BODY_H, sway: null };
}

// Presentation lookup on a stable identity, not an inference about text:
// the same kind of table as the server's colour palette. An unlisted name
// falls through to `person`, so a rename costs a shape, never a crash.
const SPECIES = { Bruce: crow, Jill: kitten, Sentinel: owl, Jack: hare };

export function buildRig(occupant) {
  const make = SPECIES[occupant.name] || person;
  return make(new THREE.Color(occupant.color));
}

// Gait motion, deliberately small: enough that an abstract shape reads as
// alive rather than slid across the ground, not enough to distract. Phase
// is seeded per name so two occupants walking together don't bob in step.
export function animateRig(a, t) {
  const walking = a.gait === 'walk';
  const ph = t * 8.5 + a.phase;
  a.rig.position.y = walking ? Math.abs(Math.sin(ph)) * 0.055 : 0;
  if (a.sway) {
    a.sway.rotation.z = -0.75 + (walking ? Math.sin(ph * 0.5) * 0.22
                                         : Math.sin(t * 1.1) * 0.07);
  }
}
