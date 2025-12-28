

// bot.js (CommonJS) - Mineflayer bot + Lodestone-facing REST API
// Requires: npm i mineflayer express
//
// Run example:
//   BOT_USERNAME=JillBot MC_HOST=127.0.0.1 MC_PORT=25565 API_PORT=3003 node bot.js
//
// Optional auth:
//   LODESTONE_TOKEN=secret123  (then send Authorization: Bearer secret123)

const mineflayer = require('mineflayer')
const express = require('express')

const BOT_USERNAME = process.env.BOT_USERNAME || 'JillBot'
const MC_HOST = process.env.MC_HOST || '127.0.0.1'
const MC_PORT = Number(process.env.MC_PORT || 25565)
const MC_VERSION = process.env.MC_VERSION || false // e.g. "1.20.4" or leave false for auto
const API_HOST = process.env.API_HOST || '127.0.0.1'
const API_PORT = Number(process.env.API_PORT || 3003)
const LODESTONE_TOKEN = process.env.LODESTONE_TOKEN || '' // optional

function nowMs() { return Date.now() }

function requireAuth(req, res, next) {
  if (!LODESTONE_TOKEN) return next()
  const h = req.headers['authorization'] || ''
  const ok = h.startsWith('Bearer ') && h.slice('Bearer '.length) === LODESTONE_TOKEN
  if (!ok) return res.status(401).json({ ok: false, error: 'unauthorized' })
  next()
}

function clamp(x, lo, hi) { return Math.max(lo, Math.min(hi, x)) }

function vecToObj(v) {
  if (!v) return null
  return { x: Number(v.x), y: Number(v.y), z: Number(v.z) }
}

function relToAbs(bot, rel) {
  // rel: {dx,dy,dz} cartesian OR {forward,right,up} egocentric
  const base = bot.entity.position.floored()

  // 1. If explicit Cartesian offset is provided (legacy support + allocentric)
  if (rel.dx !== undefined || rel.dy !== undefined || rel.dz !== undefined) {
    return base.offset(Number(rel.dx || 0), Number(rel.dy || 0), Number(rel.dz || 0))
  }

  // 2. Egocentric transformation
  // Minecraft Yaw: 0=South, -PI/2=East, PI/2=West, PI=North (Mineflayer radians)
  // We want:
  //   Forward (+Z local) -> moves in direction of Yaw
  //   Right   (+X local) -> moves 90 deg relative to Yaw
  //   Up      (+Y local) -> global Y (gravity invariant)
  
  const forward = Number(rel.forward || 0)
  const right   = Number(rel.right || 0)
  const up      = Number(rel.up || 0)

  // Get bot's yaw (rotation around Y axis)
  const yaw = bot.entity.yaw

  // Calculate offsets
  // forward vector = (-sin(yaw), 0, -cos(yaw))
  // right vector   = (-cos(yaw), 0, sin(yaw))
  
  const dx = -Math.sin(yaw) * forward - Math.cos(yaw) * right
  const dz = -Math.cos(yaw) * forward + Math.sin(yaw) * right
  const dy = up

  // Round to nearest block center for clean interaction
  return base.offset(Math.round(dx), Math.round(dy), Math.round(dz))
}

const bot = mineflayer.createBot({
  host: MC_HOST,
  port: MC_PORT,
  username: BOT_USERNAME,
  version: MC_VERSION || undefined,
})

async function ensureDryLand(maxMs = 5000) {
  const start = Date.now()
  console.log('[boot] ensuring dry land...')

  while (Date.now() - start < maxMs) {
    const headBlock = bot.blockAt(bot.entity.position.offset(0, 1.6, 0))
    const feetBlock = bot.blockAt(bot.entity.position)

    const inWater =
      (headBlock && headBlock.name.includes('water')) ||
      (feetBlock && feetBlock.name.includes('water'))

    if (!inWater) {
      stopAllControls()
      console.log('[boot] dry land achieved')
      return true
    }

    // Swim / escape behavior
    bot.setControlState('jump', true)
    bot.setControlState('forward', true)
    bot.setControlState('sprint', true)

    await sleep(250)
  }

  stopAllControls()
  console.log('[boot] WARNING: failed to find dry land in time')
  return false
}

bot.on('spawn', async () => {
  console.log('[bot] spawned')
  await ensureDryLand()
  try { bot.chat('Hello world! Jill is here 👋') } catch {}
})

bot.on('death', () => {
  console.log('[bot] died')
})

bot.on('respawn', async () => {
  console.log('[bot] respawned')
  await ensureDryLand()
})

/**
 * Simple action serialization to avoid overlapping movement timers etc.
 */
let currentAction = {
  id: null,
  type: null,
  startedAt: 0,
  endsAt: 0,
  note: '',
}

function setAction(type, durationMs = 0, note = '') {
  const id = `${type}-${nowMs()}-${Math.random().toString(16).slice(2)}`
  currentAction = {
    id,
    type,
    startedAt: nowMs(),
    endsAt: durationMs > 0 ? (nowMs() + durationMs) : 0,
    note,
  }
  return currentAction
}

function stopAllControls() {
  bot.setControlState('forward', false)
  bot.setControlState('back', false)
  bot.setControlState('left', false)
  bot.setControlState('right', false)
  bot.setControlState('jump', false)
  bot.setControlState('sprint', false)
  bot.setControlState('sneak', false)
}

async function sleep(ms) {
  await new Promise((r) => setTimeout(r, ms))
}

function safeEntitySummary(e) {
  return {
    id: e.id,
    type: e.type,              // 'player' | 'mob' | 'object' ...
    name: e.name || null,      // mob name
    username: e.username || null, // for players
    position: vecToObj(e.position),
  }
}

function listNearbyEntities(radius = 12, filterItems = null, maxTimeMs = 10) {
  // filterItems: null (all), true (items only), false (non-items only)
  // Returns: { entities: [...], complete: true/false, elapsed_ms: number }
  const startTime = nowMs()
  const out = []
  const me = bot.entity.position
  const entityIds = Object.keys(bot.entities)
  
  for (let i = 0; i < entityIds.length; i++) {
    // Check timeout (leave 1ms buffer for items - should be fast)
    if (nowMs() - startTime > maxTimeMs - 1) {
      out.sort((a, b) => a.distance - b.distance)
      return { entities: out, complete: false, elapsed_ms: nowMs() - startTime }
    }
    
    const id = entityIds[i]
    const e = bot.entities[id]
    if (!e || !e.position) continue
    if (e.id === bot.entity.id) continue
    
    // Check if entity is an item
    const isItem = (e.displayName === 'Item' || e.entityType === 'item')
    
    // Apply filter
    if (filterItems === true && !isItem) continue  // Items only
    if (filterItems === false && isItem) continue  // Non-items only
    
    const dist = e.position.distanceTo(me)
    if (dist <= radius) {
      const summary = { ...safeEntitySummary(e), distance: Number(dist) }
      // Add item-specific info if it's an item
      if (isItem) {
        summary.item_name = e.name || e.displayName || e.metadata?.[8]?.itemId || null
        summary.item_count = e.count || e.metadata?.[8]?.itemCount || 1
      }
      out.push(summary)
    }
  }
  out.sort((a, b) => a.distance - b.distance)
  return { entities: out, complete: true, elapsed_ms: nowMs() - startTime }
}

function getVisibilityDistances(radius = 5) {
  // Returns visibility distances in 6 directions: forward, back, left, right, up, down
  // Distance is to nearest opaque block in that direction, or null if none found within radius
  // Returns: { forward, back, left, right, up, down } (each is number or null)
  
  const eyePos = bot.entity.position.offset(0, 1.6, 0)
  const yaw = bot.entity.yaw
  
  // Calculate direction vectors based on yaw
  // Forward: direction bot is facing
  const forwardDx = -Math.sin(yaw)
  const forwardDz = -Math.cos(yaw)
  
  // Back: opposite of forward
  const backDx = -forwardDx
  const backDz = -forwardDz
  
  // Left: 90° left of forward (rotate forward vector 90° counterclockwise)
  const leftDx = -forwardDz  // perpendicular to forward
  const leftDz = forwardDx
  
  // Right: 90° right of forward (rotate forward vector 90° clockwise)
  const rightDx = forwardDz
  const rightDz = -forwardDx
  
  // Helper function to raycast in a direction
  function raycastDirection(dx, dy, dz, maxDist) {
    const step = 0.1  // Check every 0.1 blocks
    const steps = Math.floor(maxDist / step)
    
    for (let i = 1; i <= steps; i++) {
      const dist = i * step
      const checkPos = eyePos.offset(dx * dist, dy * dist, dz * dist)
      const block = bot.blockAt(checkPos)
      
      if (block && block.name !== 'air') {
        // Check if block is opaque (simplified: check if it's not transparent)
        // Common transparent blocks: glass, water, ice, leaves, etc.
        const transparentBlocks = ['glass', 'water', 'ice', 'leaves', 'slab', 'stairs', 'fence', 'fence_gate', 'wall', 'ladder', 'vine', 'cobweb']
        const isTransparent = transparentBlocks.some(name => block.name.includes(name))
        
        if (!isTransparent) {
          // Found opaque block - return distance
          return Number(dist.toFixed(2))
        }
      }
    }
    
    // No opaque block found within radius
    return null
  }
  
  // Raycast in each direction
  const forward = raycastDirection(forwardDx, 0, forwardDz, radius)
  const back = raycastDirection(backDx, 0, backDz, radius)
  const left = raycastDirection(leftDx, 0, leftDz, radius)
  const right = raycastDirection(rightDx, 0, rightDz, radius)
  const up = raycastDirection(0, 1, 0, radius)
  const down = raycastDirection(0, -1, 0, radius)
  
  return { forward, back, left, right, up, down }
}

function exhaustiveVisibleBlocks(radius = 5, maxTimeMs = 10) {
  // Exhaustive enumeration of ALL visible non-air blocks within radius R
  // Visibility = within radius R AND line-of-sight from bot's eye position
  // Returns: { blocks: [...], complete: true/false, elapsed_ms: number }
  
  const startTime = nowMs()
  const center = bot.entity.position.floored()
  const results = []
  
  // Get bot eye position (eye height ~1.6 blocks above feet)
  const eyePos = bot.entity.position.offset(0, 1.6, 0)
  
  // Iterate through all blocks in cube around bot
  for (let dy = -radius; dy <= radius; dy++) {
    for (let dx = -radius; dx <= radius; dx++) {
      for (let dz = -radius; dz <= radius; dz++) {
        // Check timeout (leave 2ms buffer)
        if (nowMs() - startTime > maxTimeMs - 2) {
          return { blocks: results, complete: false, elapsed_ms: nowMs() - startTime }
        }
        
        const p = center.offset(dx, dy, dz)
        const b = bot.blockAt(p)
        if (!b) continue
        
        // Skip air blocks
        if (b.name === 'air') continue
        
        // Check line-of-sight from bot eye to block center
        // Use mineflayer's canSeeBlock if available, otherwise manual raycast
        let isVisible = false
        try {
          if (typeof bot.canSeeBlock === 'function') {
            isVisible = bot.canSeeBlock(b)
          } else {
            // Fallback: manual raycast check
            // Raycast from eye to block center
            const blockCenter = b.position.offset(0.5, 0.5, 0.5)
            const ray = bot.world.raycast(eyePos, blockCenter)
            // Block is visible if raycast hits the target block (or nothing blocks it)
            isVisible = !ray || ray.block === b || ray.position.distanceTo(blockCenter) < 0.1
          }
        } catch (e) {
          // If LOS check fails, assume visible (fail open for performance)
          isVisible = true
        }
        
        if (isVisible) {
          results.push({
            name: b.name,
            position: vecToObj(b.position),
            dx, dy, dz,
          })
        }
      }
    }
  }
  
  return { blocks: results, complete: true, elapsed_ms: nowMs() - startTime }
}

function statusPayload() {
  const ent = bot.entity
  const yaw = ent ? ent.yaw : 0
  const pitch = ent ? ent.pitch : 0
  const t = bot.time || {}
  return {
    ok: true,
    bot: {
      username: bot.username,
      uuid: bot.uuid || null,
      entityId: bot.entity ? bot.entity.id : null,
    },
    connected: !!bot.player,
    position: bot.entity ? vecToObj(bot.entity.position) : null,
    yaw: Number(yaw),
    pitch: Number(pitch),
    health: typeof bot.health === 'number' ? bot.health : null,
    food: typeof bot.food === 'number' ? bot.food : null,
    onGround: bot.entity ? !!bot.entity.onGround : null,
    dimension: bot.game ? bot.game.dimension : null,
    time: {
      timeOfDay: typeof t.timeOfDay === 'number' ? t.timeOfDay : null,
      day: typeof t.day === 'number' ? t.day : null,
    },
    action: currentAction,
  }
}

// --- Bot lifecycle logs ---

bot.on('login', () => {
  console.log(`[bot] logged in as ${BOT_USERNAME} -> ${MC_HOST}:${MC_PORT} (version=${bot.version})`)
})

bot.on('spawn', () => {
  console.log('[bot] spawned')
  try { bot.chat('Hello world! Jill is here 👋') } catch {}
})

bot.on('end', () => {
  console.log('[bot] disconnected')
})

bot.on('kicked', (reason) => {
  console.log('[bot] kicked:', reason)
})

bot.on('error', (err) => {
  console.log('[bot] error:', err)
})

bot.on('death', () => {
  console.log('[bot] died')
})

// --- REST API ---

const app = express()
app.use(express.json({ limit: '1mb' }))
app.use(requireAuth)

// Request logging middleware
app.use((req, res, next) => {
  const method = req.method
  const path = req.path
  const bodySummary = req.method === 'POST' && req.body 
    ? Object.keys(req.body).slice(0, 3).map(k => `${k}=${typeof req.body[k] === 'object' ? '...' : req.body[k]}`).join(',')
    : ''
  console.log(`[api] ${method} ${path}${bodySummary ? ` (${bodySummary})` : ''}`)
  next()
})

app.get('/healthz', (req, res) => {
  res.json({ ok: true })
})

app.get('/status', (req, res) => {
  res.json(statusPayload())
})

app.get('/observe', (req, res) => {
  const radiusBlocks = clamp(Number(req.query.blocks_radius ?? 5), 1, 6)
  const radiusEntities = clamp(Number(req.query.entities_radius ?? 5), 1, 12)
  
  // Entity filter: null (all), true (items only), false (non-items only)
  let entityFilter = null
  if (req.query.entity_filter === 'items') entityFilter = true
  else if (req.query.entity_filter === 'non-items') entityFilter = false

  // Exhaustive visible blocks enumeration (with timeout protection)
  const blocksResult = exhaustiveVisibleBlocks(radiusBlocks, 10)
  
  // Exhaustive entities enumeration (with timeout protection)
  const entitiesResult = listNearbyEntities(radiusEntities, entityFilter, 10)
  
  // Visibility distances (use entities radius for consistency)
  const visibilityRadius = radiusEntities
  const visibilityDistances = getVisibilityDistances(visibilityRadius)
  
  res.json({
    ok: true,
    status: statusPayload(),
    perception: {
      nearby_blocks: blocksResult.blocks,
      blocks_complete: blocksResult.complete,
      blocks_elapsed_ms: blocksResult.elapsed_ms,
      nearby_entities: entitiesResult.entities,
      entities_complete: entitiesResult.complete,
      entities_elapsed_ms: entitiesResult.elapsed_ms,
      visibility_distances: visibilityDistances,
    },
  })
})

app.post('/say', (req, res) => {
  const msg = String(req.body?.message || '').slice(0, 200)
  if (!msg) {
    console.log(`[api] /say FAIL: message required`)
    return res.status(400).json({ ok: false, error: 'message required' })
  }
  try {
    bot.chat(msg)
    console.log(`[api] /say OK`)
    res.json({ ok: true })
  } catch (e) {
    console.log(`[api] /say ERROR: ${e}`)
    res.status(500).json({ ok: false, error: String(e) })
  }
})

app.post('/act/stop', (req, res) => {
  try {
    stopAllControls()
    setAction('stop', 0, 'stopped all controls')
    console.log(`[api] /act/stop OK`)
    res.json({ ok: true })
  } catch (e) {
    console.log(`[api] /act/stop ERROR: ${e}`)
    res.status(500).json({ ok: false, error: String(e) })
  }
})

app.post('/act/look', (req, res) => {
  const yaw = Number(req.body?.yaw)
  const pitch = Number(req.body?.pitch)
  if (!Number.isFinite(yaw) || !Number.isFinite(pitch)) {
    console.log(`[api] /act/look FAIL: invalid yaw/pitch`)
    return res.status(400).json({ ok: false, error: 'yaw and pitch must be numbers (radians)' })
  }
  try {
    // Mineflayer expects radians; pitch typically [-pi/2, +pi/2]
    const p = clamp(pitch, -Math.PI / 2, Math.PI / 2)
    bot.look(yaw, p, true)
    setAction('look', 0, `yaw=${yaw} pitch=${p}`)
    console.log(`[api] /act/look OK`)
    res.json({ ok: true })
  } catch (e) {
    console.log(`[api] /act/look ERROR: ${e}`)
    res.status(500).json({ ok: false, error: String(e) })
  }
})

app.post('/act/move', async (req, res) => {
  // Collision-Aware Movement Endpoint
  // Body: { forward: bool, ..., duration_ms: 500, check_collision: bool }
  try {
    const durationMs = clamp(Number(req.body?.duration_ms ?? 400), 50, 60_000)
    const checkCollision = req.body?.check_collision !== false // default true

    const controls = ['forward','back','left','right','jump','sprint','sneak']
    stopAllControls()
    for (const c of controls) {
      if (typeof req.body?.[c] === 'boolean') bot.setControlState(c, req.body[c])
    }

    const actionId = `move-${nowMs()}-${Math.random().toString(16).slice(2)}`
    const action = setAction('move', durationMs, JSON.stringify(req.body).slice(0, 200))
    action.id = actionId

    // Start physics loop
    const TICK_MS = 50
    const startPos = bot.entity.position.clone()
    let elapsed = 0
    let collision = null
    
    // We send response AFTER movement completes or fails (Synchronous behavior simulation)
    // To avoid timeout on client, we cap max synchronous wait, but for now we assume client handles 60s timeout
    
    // NOTE: Express doesn't like holding connections forever, but 2-5s is fine.
    // For very long moves, this should be async, but for "manual" moves (usually < 2s), sync is better.
    
    const movePromise = new Promise((resolve) => {
      const interval = setInterval(() => {
        elapsed += TICK_MS
        
        // 1. Check Duration
        if (elapsed >= durationMs) {
          clearInterval(interval)
          stopAllControls()
          resolve({ status: 'completed', elapsed })
          return
        }

        // 2. Check Collision (Velocity near zero despite control state)
        if (checkCollision && elapsed > 250) { // Give 250ms for acceleration
           const vel = bot.entity.velocity
           const isMoving = req.body.forward || req.body.back || req.body.left || req.body.right
           const speed = Math.sqrt(vel.x*vel.x + vel.z*vel.z)
           
           if (isMoving && speed < 0.05) { // Stuck?
             // Double check: are we just blocked?
             const blockInFront = bot.blockAt(bot.entity.position.offset(0,1,0).offset(bot.entity.velocity.x*5, 0, bot.entity.velocity.z*5))
             // Simple heuristic: if we wanted to move but aren't, assume collision
             clearInterval(interval)
             stopAllControls()
             resolve({ status: 'collision', elapsed, pos: bot.entity.position })
             return
           }
        }
      }, TICK_MS)
    })

    // Wait for completion
    const result = await movePromise

    if (result.status === 'collision') {
        setAction('idle', 0, 'collision stopped move')
        console.log(`[api] /act/move COLLISION after ${result.elapsed}ms`)
        return res.json({ 
            ok: true, 
            status: 'collision',
            reason: 'velocity dropped near zero (stuck/blocked)',
            intended_duration_ms: durationMs,
            actual_duration_ms: result.elapsed,
            final_position: vecToObj(result.pos)
        })
    }

    setAction('idle', 0, 'move completed')
    console.log(`[api] /act/move OK (finished ${result.elapsed}ms)`)
    res.json({ 
        ok: true, 
        status: 'success',
        intended_duration_ms: durationMs,
        actual_duration_ms: result.elapsed,
        final_position: vecToObj(bot.entity.position)
    })

  } catch (e) {
    console.log(`[api] /act/move ERROR: ${e}`)
    res.status(500).json({ ok: false, error: String(e) })
  }
})

app.post('/act/dig', async (req, res) => {
  // Dig a block either by absolute position or relative offset.
  // Body:
  //  { "pos": {"x":1,"y":64,"z":2} }  OR  { "rel": {"dx":1,"dy":0,"dz":0} }
  try {
    let pos = null
    if (req.body?.pos && Number.isFinite(req.body.pos.x) && Number.isFinite(req.body.pos.y) && Number.isFinite(req.body.pos.z)) {
      pos = req.body.pos
      pos = bot.entity.position.constructor(pos.x, pos.y, pos.z) // not reliable across mineflayer builds
    }
  } catch {}

  // Safer: use blockAt with Vec3 from mineflayer
  const Vec3 = require('vec3').Vec3
  let targetPos = null
  if (req.body?.pos && Number.isFinite(req.body.pos.x) && Number.isFinite(req.body.pos.y) && Number.isFinite(req.body.pos.z)) {
    targetPos = new Vec3(req.body.pos.x, req.body.pos.y, req.body.pos.z)
  } else if (req.body?.rel) {
    targetPos = relToAbs(bot, req.body.rel)
  } else {
    console.log(`[api] /act/dig FAIL: missing pos/rel`)
    return res.status(400).json({ ok: false, error: 'provide pos {x,y,z} or rel {dx,dy,dz}' })
  }

  const block = bot.blockAt(targetPos)
  if (!block) {
    console.log(`[api] /act/dig FAIL: no block at target`)
    return res.status(404).json({ ok: false, error: 'no block at target' })
  }
  if (block.name === 'air') {
    console.log(`[api] /act/dig FAIL: target is air`)
    return res.status(400).json({ ok: false, error: 'target block is air' })
  }

  try {
    setAction('dig', 0, `dig ${block.name} @ ${block.position}`)
    console.log(`[api] /act/dig OK (${block.name})`)
    // Start async dig, don't wait
    bot.dig(block).then(() => {
      // Dig completed (async)
    }).catch(e => console.log(`[api] /act/dig async ERROR: ${e}`))
    res.json({ ok: true, dug: { name: block.name, position: vecToObj(block.position) } })
  } catch (e) {
    console.log(`[api] /act/dig ERROR: ${e}`)
    res.status(500).json({ ok: false, error: String(e) })
  }
})

app.post('/act/place', async (req, res) => {
  // Place an item onto a reference block face.
  // Body:
  // {
  //   "item": "oak_planks",
  //   "ref": { "rel": {"dx":0,"dy":-1,"dz":0} },   // reference block
  //   "face": "up"                                 // up|down|north|south|east|west
  // }
  const itemName = String(req.body?.item || '')
  if (!itemName) {
    console.log(`[api] /act/place FAIL: item required`)
    return res.status(400).json({ ok: false, error: 'item required' })
  }

  const faceName = String(req.body?.face || 'up')
  const faceMap = {
    down: { x: 0, y: -1, z: 0 },
    up: { x: 0, y: 1, z: 0 },
    north: { x: 0, y: 0, z: -1 },
    south: { x: 0, y: 0, z: 1 },
    west: { x: -1, y: 0, z: 0 },
    east: { x: 1, y: 0, z: 0 },
  }
  const face = faceMap[faceName] || faceMap.up

  const Vec3 = require('vec3').Vec3
  let refPos = null
  if (req.body?.ref?.pos && Number.isFinite(req.body.ref.pos.x) && Number.isFinite(req.body.ref.pos.y) && Number.isFinite(req.body.ref.pos.z)) {
    refPos = new Vec3(req.body.ref.pos.x, req.body.ref.pos.y, req.body.ref.pos.z)
  } else if (req.body?.ref?.rel) {
    refPos = relToAbs(bot, req.body.ref.rel)
  } else {
    // default: place on the block underfoot
    refPos = bot.entity.position.floored().offset(0, -1, 0)
  }

  const refBlock = bot.blockAt(refPos)
  if (!refBlock || refBlock.name === 'air') {
    console.log(`[api] /act/place FAIL: reference block missing/air`)
    return res.status(400).json({ ok: false, error: 'reference block missing/air' })
  }

  // Find item in inventory
  const invItem = bot.inventory.items().find(it => it.name === itemName)
  if (!invItem) {
    console.log(`[api] /act/place FAIL: item not in inventory: ${itemName}`)
    return res.status(400).json({ ok: false, error: `item not in inventory: ${itemName}` })
  }

  try {
    setAction('place', 0, `place ${itemName} on ${refBlock.name} face=${faceName}`)
    console.log(`[api] /act/place OK (${itemName})`)
    // Start async place, don't wait
    bot.equip(invItem, 'hand').then(() => {
      return bot.placeBlock(refBlock, new Vec3(face.x, face.y, face.z))
    }).catch(e => console.log(`[api] /act/place async ERROR: ${e}`))
    res.json({ ok: true })
  } catch (e) {
    console.log(`[api] /act/place ERROR: ${e}`)
    res.status(500).json({ ok: false, error: String(e) })
  }
})

app.post('/act/respawn', (req, res) => {
  try {
    bot.respawn()
    setAction('respawn', 0, 'respawn requested')
    console.log(`[api] /act/respawn OK`)
    res.json({ ok: true })
  } catch (e) {
    console.log(`[api] /act/respawn ERROR: ${e}`)
    res.status(500).json({ ok: false, error: String(e) })
  }
})

app.get('/inventory', (req, res) => {
  try {
    const slots = []
    const inv = bot.inventory
    if (inv && inv.slots) {
      for (let i = 0; i < inv.slots.length; i++) {
        const slot = inv.slots[i]
        if (slot && slot.type !== -1) {
          slots.push({
            slot: i,
            item: slot.name || 'unknown',
            count: slot.count || 0
          })
        }
      }
    }
    
    const equipped = {
      hand: null,
      offhand: null
    }
    
    if (inv && inv.slots) {
      const handSlot = inv.slots[36 + inv.heldItemSlot]
      if (handSlot && handSlot.type !== -1) {
        equipped.hand = handSlot.name || null
      }
      
      const offhandSlot = inv.slots[45]
      if (offhandSlot && offhandSlot.type !== -1) {
        equipped.offhand = offhandSlot.name || null
      }
    }
    
    res.json({ ok: true, slots, equipped })
  } catch (e) {
    console.log(`[api] /inventory ERROR: ${e}`)
    res.status(500).json({ ok: false, error: String(e) })
  }
})

app.post('/act/equip', async (req, res) => {
  const itemName = String(req.body?.item || '')
  const slotName = String(req.body?.slot || 'hand')
  
  if (!itemName) {
    console.log(`[api] /act/equip FAIL: item required`)
    return res.status(400).json({ ok: false, error: 'item required', error_code: 'item_required' })
  }
  
  if (slotName !== 'hand' && slotName !== 'offhand') {
    console.log(`[api] /act/equip FAIL: invalid slot`)
    return res.status(400).json({ ok: false, error: "slot must be 'hand' or 'offhand'", error_code: 'invalid_slot' })
  }
  
  try {
    const invItem = bot.inventory.items().find(it => it.name === itemName)
    if (!invItem) {
      console.log(`[api] /act/equip FAIL: item not in inventory`)
      return res.status(400).json({ ok: false, error: `item not in inventory: ${itemName}`, error_code: 'item_not_in_inventory' })
    }
    
    const equipSlot = slotName === 'hand' ? 'hand' : 'off-hand'
    await bot.equip(invItem, equipSlot)
    
    setAction('equip', 0, `equip ${itemName} to ${slotName}`)
    console.log(`[api] /act/equip OK (${itemName} -> ${slotName})`)
    res.json({ ok: true, equipped: itemName })
  } catch (e) {
    console.log(`[api] /act/equip ERROR: ${e}`)
    res.status(500).json({ ok: false, error: String(e), error_code: 'equip_failed' })
  }
})

app.post('/act/unequip', async (req, res) => {
  const slotName = String(req.body?.slot || 'hand')
  
  if (slotName !== 'hand' && slotName !== 'offhand') {
    console.log(`[api] /act/unequip FAIL: invalid slot`)
    return res.status(400).json({ ok: false, error: "slot must be 'hand' or 'offhand'", error_code: 'invalid_slot' })
  }
  
  try {
    const inv = bot.inventory
    let slotIndex = null
    if (slotName === 'hand') {
      slotIndex = 36 + inv.heldItemSlot
    } else {
      slotIndex = 45
    }
    
    const slot = inv.slots[slotIndex]
    if (!slot || slot.type === -1) {
      console.log(`[api] /act/unequip FAIL: nothing equipped`)
      return res.status(400).json({ ok: false, error: 'nothing equipped', error_code: 'nothing_equipped' })
    }
    
    await bot.unequip(slotName === 'hand' ? 'hand' : 'off-hand')
    
    setAction('unequip', 0, `unequip ${slotName}`)
    console.log(`[api] /act/unequip OK (${slotName})`)
    res.json({ ok: true })
  } catch (e) {
    console.log(`[api] /act/unequip ERROR: ${e}`)
    res.status(500).json({ ok: false, error: String(e), error_code: 'unequip_failed' })
  }
})

app.post('/act/use', async (req, res) => {
  const Vec3 = require('vec3').Vec3
  let targetPos = null
  
  if (req.body?.pos && Number.isFinite(req.body.pos.x) && Number.isFinite(req.body.pos.y) && Number.isFinite(req.body.pos.z)) {
    targetPos = new Vec3(req.body.pos.x, req.body.pos.y, req.body.pos.z)
  } else if (req.body?.rel) {
    targetPos = relToAbs(bot, req.body.rel)
  } else {
    console.log(`[api] /act/use FAIL: missing pos/rel`)
    return res.status(400).json({ ok: false, error: 'provide pos {x,y,z} or rel {dx,dy,dz}', error_code: 'position_required' })
  }
  
  const block = bot.blockAt(targetPos)
  if (!block) {
    console.log(`[api] /act/use FAIL: no block at target`)
    return res.status(404).json({ ok: false, error: 'no block at target', error_code: 'no_block_at_target' })
  }
  
  try {
    setAction('use', 0, `use ${block.name} @ ${block.position}`)
    await bot.activateBlock(block)
    console.log(`[api] /act/use OK (${block.name})`)
    res.json({ ok: true })
  } catch (e) {
    console.log(`[api] /act/use ERROR: ${e}`)
    const errorMsg = String(e)
    let errorCode = 'use_failed'
    if (errorMsg.includes('out of range')) errorCode = 'out_of_range'
    else if (errorMsg.includes('not interactable')) errorCode = 'target_not_interactable'
    res.status(500).json({ ok: false, error: errorMsg, error_code: errorCode })
  }
})

app.post('/act/attack', async (req, res) => {
  try {
    if (req.body?.entity_id) {
      const entityId = Number(req.body.entity_id)
      const entity = bot.entities[entityId]
      if (!entity) {
        console.log(`[api] /act/attack FAIL: entity not found`)
        return res.status(404).json({ ok: false, error: `entity ${entityId} not found`, error_code: 'entity_not_found' })
      }
      
      setAction('attack', 0, `attack entity ${entityId}`)
      await bot.attack(entity)
      console.log(`[api] /act/attack OK (entity ${entityId})`)
      res.json({ ok: true })
    } else if (req.body?.rel) {
      const Vec3 = require('vec3').Vec3
      const targetPos = relToAbs(bot, req.body.rel)
      const block = bot.blockAt(targetPos)
      
      if (!block || block.name === 'air') {
        console.log(`[api] /act/attack FAIL: no block at target`)
        return res.status(404).json({ ok: false, error: 'no block at target', error_code: 'no_block_at_target' })
      }
      
      setAction('attack', 0, `attack block ${block.name}`)
      bot.swingArm()
      await bot.dig(block)
      console.log(`[api] /act/attack OK (block ${block.name})`)
      res.json({ ok: true })
    } else {
      console.log(`[api] /act/attack FAIL: missing entity_id or rel`)
      return res.status(400).json({ ok: false, error: 'provide entity_id or rel {dx,dy,dz}', error_code: 'target_required' })
    }
  } catch (e) {
    console.log(`[api] /act/attack ERROR: ${e}`)
    res.status(500).json({ ok: false, error: String(e), error_code: 'attack_failed' })
  }
})

app.post('/act/open', async (req, res) => {
  const Vec3 = require('vec3').Vec3
  let targetPos = null
  
  if (req.body?.pos && Number.isFinite(req.body.pos.x) && Number.isFinite(req.body.pos.y) && Number.isFinite(req.body.pos.z)) {
    targetPos = new Vec3(req.body.pos.x, req.body.pos.y, req.body.pos.z)
  } else if (req.body?.rel) {
    targetPos = relToAbs(bot, req.body.rel)
  } else {
    console.log(`[api] /act/open FAIL: missing pos/rel`)
    return res.status(400).json({ ok: false, error: 'provide pos {x,y,z} or rel {dx,dy,dz}', error_code: 'position_required' })
  }
  
  const block = bot.blockAt(targetPos)
  if (!block) {
    console.log(`[api] /act/open FAIL: no block at target`)
    return res.status(404).json({ ok: false, error: 'no block at target', error_code: 'no_block_at_target' })
  }
  
  try {
    let window = null
    let uiType = 'unknown'
    
    if (block.name.includes('chest') || block.name.includes('barrel')) {
      window = await bot.openChest(block)
      uiType = 'chest'
    } else if (block.name.includes('furnace')) {
      window = await bot.openFurnace(block)
      uiType = 'furnace'
    } else if (block.name.includes('crafting_table')) {
      window = await bot.openCraftingTable(block)
      uiType = 'crafting_table'
    } else {
      console.log(`[api] /act/open FAIL: block not openable`)
      return res.status(400).json({ ok: false, error: `block ${block.name} cannot be opened`, error_code: 'block_not_openable' })
    }
    
    setAction('open', 0, `open ${uiType}`)
    console.log(`[api] /act/open OK (${uiType})`)
    res.json({ ok: true, ui_type: uiType })
  } catch (e) {
    console.log(`[api] /act/open ERROR: ${e}`)
    res.status(500).json({ ok: false, error: String(e), error_code: 'open_failed' })
  }
})

app.post('/act/close', (req, res) => {
  try {
    if (!bot.currentWindow) {
      console.log(`[api] /act/close FAIL: no window open`)
      return res.status(400).json({ ok: false, error: 'no window open', error_code: 'no_window_open' })
    }
    
    bot.closeWindow(bot.currentWindow)
    setAction('close', 0, 'close window')
    console.log(`[api] /act/close OK`)
    res.json({ ok: true })
  } catch (e) {
    console.log(`[api] /act/close ERROR: ${e}`)
    res.status(500).json({ ok: false, error: String(e), error_code: 'close_failed' })
  }
})

app.post('/act/craft', async (req, res) => {
  const recipeName = String(req.body?.recipe || '')
  const count = Number(req.body?.count || 1)
  
  if (!recipeName) {
    console.log(`[api] /act/craft FAIL: recipe required`)
    return res.status(400).json({ ok: false, error: 'recipe required', error_code: 'recipe_required' })
  }
  
  try {
    const recipes = bot.recipesFor(bot.mcData.itemsByName[recipeName]?.id || null)
    if (!recipes || recipes.length === 0) {
      console.log(`[api] /act/craft FAIL: recipe not found`)
      return res.status(400).json({ ok: false, error: `recipe not found: ${recipeName}`, error_code: 'recipe_not_known' })
    }
    
    if (!bot.currentWindow || bot.currentWindow.type !== 'minecraft:crafting_table') {
      const craftingTable = bot.findBlock({
        matching: block => block.name.includes('crafting_table'),
        maxDistance: 5
      })
      
      if (!craftingTable) {
        console.log(`[api] /act/craft FAIL: no crafting interface`)
        return res.status(400).json({ ok: false, error: 'no crafting interface open and no crafting table nearby', error_code: 'no_crafting_interface' })
      }
      
      await bot.openCraftingTable(craftingTable)
    }
    
    const recipe = recipes[0]
    const craftCount = Math.max(1, Math.floor(count))
    
    await bot.craft(recipe, craftCount, bot.currentWindow)
    
    setAction('craft', 0, `craft ${craftCount}x ${recipeName}`)
    console.log(`[api] /act/craft OK (${craftCount}x ${recipeName})`)
    res.json({ ok: true, item: recipeName, count: craftCount })
  } catch (e) {
    console.log(`[api] /act/craft ERROR: ${e}`)
    const errorMsg = String(e)
    let errorCode = 'craft_failed'
    if (errorMsg.includes('ingredients')) errorCode = 'missing_ingredients'
    else if (errorMsg.includes('crafting')) errorCode = 'no_crafting_interface'
    res.status(500).json({ ok: false, error: errorMsg, error_code: errorCode })
  }
})

app.post('/act/pickup', async (req, res) => {
  const radius = clamp(Number(req.body?.radius ?? 2.5), 0.5, 10)
  const filter = Array.isArray(req.body?.filter) ? req.body.filter.map(String) : null
  
  try {
    const pickedUp = []
    const failed = []
    
    const myPos = bot.entity.position
    const itemEntities = []
    
    for (const id of Object.keys(bot.entities)) {
      const e = bot.entities[id]
      if (!e || !e.position) continue
      if (e.displayName !== 'Item' && e.entityType !== 'item') continue
      
      const dist = e.position.distanceTo(myPos)
      if (dist > radius) continue
      
      const itemName = e.name || e.displayName || e.metadata?.[8]?.itemId || null
      if (filter && filter.length > 0) {
        if (!itemName || !filter.includes(itemName)) continue
      }
      
      itemEntities.push({ entity: e, distance: dist, itemName })
    }
    
    if (itemEntities.length === 0) {
      console.log(`[api] /act/pickup FAIL: no items in range`)
      return res.status(400).json({ ok: false, error: 'no items in range', error_code: 'no_items_in_range' })
    }
    
    itemEntities.sort((a, b) => a.distance - b.distance)
    
    const invBefore = new Set(bot.inventory.items().map(it => `${it.name}:${it.count}`))
    
    for (const { entity, itemName } of itemEntities) {
      try {
        const itemCount = entity.count || entity.metadata?.[8]?.itemCount || 1
        
        if (bot.collectBlock && typeof bot.collectBlock.collect === 'function') {
          await bot.collectBlock.collect(entity)
        } else {
          const targetPos = entity.position
          const dist = myPos.distanceTo(targetPos)
          if (dist > 1.5) {
            bot.lookAt(targetPos)
            await sleep(100)
          }
          await sleep(300)
        }
        
        const invAfter = bot.inventory.items()
        const invAfterSet = new Set(invAfter.map(it => `${it.name}:${it.count}`))
        const wasCollected = invAfterSet.size > invBefore.size || 
          invAfter.some(it => {
            const key = `${it.name}:${it.count}`
            return !invBefore.has(key) && (itemName ? it.name === itemName : true)
          })
        
        if (wasCollected) {
          pickedUp.push({ item: itemName || 'unknown', count: itemCount })
        } else {
          failed.push({ item: itemName || 'unknown', reason: 'not collected' })
        }
      } catch (e) {
        failed.push({ item: itemName || 'unknown', reason: String(e) })
      }
    }
    
    if (pickedUp.length === 0 && failed.length > 0) {
      setAction('pickup', 0, 'pickup failed')
      console.log(`[api] /act/pickup FAIL: all items failed`)
      return res.json({ ok: true, picked_up: [], failed, error_code: 'all_failed' })
    }
    
    setAction('pickup', 0, `picked up ${pickedUp.length} item(s)`)
    console.log(`[api] /act/pickup OK (${pickedUp.length} picked up, ${failed.length} failed)`)
    res.json({ ok: true, picked_up: pickedUp, failed })
  } catch (e) {
    console.log(`[api] /act/pickup ERROR: ${e}`)
    const errorMsg = String(e)
    let errorCode = 'pickup_failed'
    if (errorMsg.includes('full')) errorCode = 'inventory_full'
    else if (errorMsg.includes('path')) errorCode = 'path_blocked'
    else if (errorMsg.includes('unreachable')) errorCode = 'entity_unreachable'
    res.status(500).json({ ok: false, error: errorMsg, error_code: errorCode })
  }
})

app.post('/act/drop', async (req, res) => {
  const itemName = String(req.body?.item || '')
  const count = req.body?.count !== undefined ? Number(req.body.count) : null
  const scatter = Boolean(req.body?.scatter)
  
  if (!itemName) {
    console.log(`[api] /act/drop FAIL: item required`)
    return res.status(400).json({ ok: false, error: 'item required', error_code: 'item_required' })
  }
  
  try {
    const invItems = bot.inventory.items().filter(it => it.name === itemName)
    if (invItems.length === 0) {
      console.log(`[api] /act/drop FAIL: item not in inventory`)
      return res.status(400).json({ ok: false, error: `item not in inventory: ${itemName}`, error_code: 'item_not_in_inventory' })
    }
    
    let totalCount = 0
    for (const item of invItems) {
      totalCount += item.count
    }
    
    const dropCount = count !== null ? Math.min(count, totalCount) : totalCount
    if (dropCount <= 0) {
      console.log(`[api] /act/drop FAIL: insufficient count`)
      return res.status(400).json({ ok: false, error: `insufficient count: have ${totalCount}, requested ${count}`, error_code: 'insufficient_count' })
    }
    
    let remaining = dropCount
    const dropped = []
    
    for (const item of invItems) {
      if (remaining <= 0) break
      
      const toDrop = Math.min(remaining, item.count)
      if (toDrop === item.count) {
        await bot.tossStack(item)
        dropped.push({ item: itemName, count: toDrop })
      } else {
        await bot.toss(item.type, item.metadata, toDrop)
        dropped.push({ item: itemName, count: toDrop })
      }
      remaining -= toDrop
    }
    
    setAction('drop', 0, `dropped ${dropCount}x ${itemName}`)
    console.log(`[api] /act/drop OK (${dropCount}x ${itemName})`)
    res.json({ ok: true, dropped })
  } catch (e) {
    console.log(`[api] /act/drop ERROR: ${e}`)
    const errorMsg = String(e)
    let errorCode = 'drop_failed'
    if (errorMsg.includes('not in inventory')) errorCode = 'item_not_in_inventory'
    else if (errorMsg.includes('insufficient')) errorCode = 'insufficient_count'
    res.status(500).json({ ok: false, error: errorMsg, error_code: errorCode })
  }
})

app.listen(API_PORT, API_HOST, () => {
  console.log(`[api] Lodestone-facing API listening on http://${API_HOST}:${API_PORT}`)
  console.log(`[api] auth: ${LODESTONE_TOKEN ? 'enabled (Bearer token)' : 'disabled'}`)
  console.log(`[api] endpoints: GET /status, GET /observe, GET /inventory, POST /act/move, /act/look, /act/dig, /act/place, /act/equip, /act/unequip, /act/use, /act/attack, /act/open, /act/close, /act/craft, /act/pickup, /act/drop, /say, /act/stop, /act/respawn`)
})
