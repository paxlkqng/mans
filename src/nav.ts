export const SIZE = 65
export const CELL = 12 // metres; demonstration grid covers 768 x 768 m
export type Point = { x: number; z: number }
export type Scenario = 'normal' | 'link' | 'visual' | 'uncertainty' | 'dust' | 'obstruction'
export type Mode = 'plan' | 'navigate'
export type RouteKind = 'safest' | 'shortest'

export type Terrain = { size: number; cellSize: number; heights: number[]; source: string; provenance: 'simulated' | 'observed' }
type WasmPlanner = (heights: number[], size: number, cell: number, sx: number, sz: number, gx: number, gz: number, safest: boolean, blocked: boolean) => Point[]
let rustPlanner: WasmPlanner | null = null
export async function loadRustPlanner(): Promise<boolean> {
  try { const moduleUrl = new URL('wasm/mans_nav.js', window.location.origin).href; const wasm = await import(/* @vite-ignore */ moduleUrl); await wasm.default(); rustPlanner = wasm.plan_route as WasmPlanner; return true } catch { return false }
}
export function route(t: Terrain, start: Point, goal: Point, kind: RouteKind, obstruction = false): Point[] {
  if (rustPlanner) return rustPlanner(t.heights, t.size, t.cellSize, start.x, start.z, goal.x, goal.z, kind === 'safest', obstruction)
  return planRoute(t, start, goal, kind, obstruction)
}

export function makeTerrain(): Terrain {
  const heights: number[] = []
  for (let z = 0; z < SIZE; z++) for (let x = 0; x < SIZE; x++) {
    const dx = x - 34, dz = z - 30
    const mound = 35 * Math.exp(-((dx - 6) ** 2 / 160 + (dz + 4) ** 2 / 90))
    const ridge = 21 * Math.exp(-((dx + 10) ** 2 / 40 + (dz - 3) ** 2 / 400))
    const bowl = -15 * Math.exp(-((dx + 11) ** 2 + (dz + 15) ** 2) / 180)
    heights.push(-2550 + mound + ridge + bowl + 3.2 * Math.sin(x * .37) * Math.cos(z * .29) + 1.4 * Math.sin(x * .93 + z * .61))
  }
  return { size: SIZE, cellSize: CELL, heights, source: 'Procedural Jezero-inspired test terrain', provenance: 'simulated' }
}

export const index = (x: number, z: number, size = SIZE) => z * size + x
export const height = (terrain: Terrain, p: Point) => terrain.heights[index(Math.round(p.x), Math.round(p.z), terrain.size)] ?? 0
export function slopeDegrees(t: Terrain, a: Point, b: Point) {
  const horizontal = Math.hypot((a.x - b.x) * t.cellSize, (a.z - b.z) * t.cellSize)
  return Math.atan2(Math.abs(height(t, a) - height(t, b)), horizontal) * 180 / Math.PI
}

export function planRoute(t: Terrain, start: Point, goal: Point, kind: RouteKind, obstruction = false): Point[] {
  const n = t.size * t.size, g = new Float64Array(n).fill(Infinity), prev = new Int32Array(n).fill(-1)
  const closed = new Uint8Array(n), open: { id: number; f: number }[] = []
  const sid = index(start.x, start.z, t.size), gid = index(goal.x, goal.z, t.size)
  g[sid] = 0; open.push({ id: sid, f: 0 })
  const heuristic = (x: number, z: number) => Math.hypot(goal.x - x, goal.z - z) * t.cellSize
  while (open.length) {
    let best = 0
    for (let i = 1; i < open.length; i++) if (open[i].f < open[best].f) best = i
    const { id } = open.splice(best, 1)[0]
    if (closed[id]) continue
    if (id === gid) break
    closed[id] = 1
    const x = id % t.size, z = Math.floor(id / t.size)
    for (let dz = -1; dz <= 1; dz++) for (let dx = -1; dx <= 1; dx++) {
      if (!dx && !dz) continue
      const nx = x + dx, nz = z + dz
      if (nx < 0 || nz < 0 || nx >= t.size || nz >= t.size) continue
      if (obstruction && nx >= 30 && nx <= 39 && nz >= 43 && nz <= 51) continue
      const ni = index(nx, nz, t.size)
      if (closed[ni]) continue
      const slope = slopeDegrees(t, { x, z }, { x: nx, z: nz })
      if (kind === 'safest' && slope > 23) continue
      const dist = Math.hypot(dx, dz) * t.cellSize
      const cost = dist * (kind === 'safest' ? 1 + (slope / 12) ** 2 * 2.5 : 1 + slope / 150)
      const tentative = g[id] + cost
      if (tentative < g[ni]) { g[ni] = tentative; prev[ni] = id; open.push({ id: ni, f: tentative + heuristic(nx, nz) }) }
    }
  }
  if (sid !== gid && prev[gid] < 0) return []
  const route: Point[] = []
  for (let at = gid; at >= 0; at = prev[at]) { route.push({ x: at % t.size, z: Math.floor(at / t.size) }); if (at === sid) break }
  return route.reverse()
}

export function routeLength(route: Point[], t: Terrain) { return route.slice(1).reduce((v, p, i) => v + Math.hypot(p.x - route[i].x, p.z - route[i].z) * t.cellSize, 0) }
export function nearestRouteIndex(route: Point[], p: Point) { let best = 0, d = Infinity; route.forEach((q, i) => { const a = Math.hypot(q.x - p.x, q.z - p.z); if (a < d) { d = a; best = i } }); return best }

// Constant-velocity 2D filter. Covariance is diagonal, deliberately educational rather than flight-ready.
export type Estimate = { x: number; z: number; vx: number; vz: number; variance: number; heading: number }
export function predict(e: Estimate, ax: number, az: number, dt: number, processNoise: number, angularVelocity = 0): Estimate {
  return { ...e, x: e.x + e.vx * dt + .5 * ax * dt * dt, z: e.z + e.vz * dt + .5 * az * dt * dt,
    vx: e.vx + ax * dt, vz: e.vz + az * dt, heading: (e.heading + angularVelocity * dt + 360) % 360, variance: e.variance + processNoise * dt * dt }
}
export function observe(e: Estimate, x: number, z: number, measurementVariance: number): Estimate {
  const k = e.variance / (e.variance + measurementVariance)
  return { ...e, x: e.x + k * (x - e.x), z: e.z + k * (z - e.z), variance: (1 - k) * e.variance }
}
export const metresToLatLon = (p: Point) => ({ latitude: 18.444 + (p.z - 32) * CELL / 59274, longitude: 77.450 + (p.x - 32) * CELL / (59274 * Math.cos(18.444 * Math.PI / 180)) })
