import { describe, expect, it } from 'vitest'
import { makeTerrain, observe, planRoute, predict, routeLength, slopeDegrees } from './nav'
const t = makeTerrain()
describe('navigation calculations', () => {
  it('routes from start to destination', () => { const r = planRoute(t, { x: 4, z: 8 }, { x: 58, z: 54 }, 'safest'); expect(r[0]).toEqual({ x: 4, z: 8 }); expect(r.at(-1)).toEqual({ x: 58, z: 54 }); expect(routeLength(r, t)).toBeGreaterThan(0) })
  it('avoids an obstruction and replans the default safe route', () => { const a = { x: 8, z: 11 }, b = { x: 56, z: 51 }; const original = planRoute(t, a, b, 'safest'); const rerouted = planRoute(t, a, b, 'safest', true); expect(rerouted.some(p => p.x >= 30 && p.x <= 39 && p.z >= 43 && p.z <= 51)).toBe(false); expect(rerouted).not.toEqual(original) })
  it('computes symmetric slope', () => { const a = { x: 20, z: 20 }, b = { x: 21, z: 20 }; expect(slopeDegrees(t, a, b)).toBeCloseTo(slopeDegrees(t, b, a)) })
  it('uncertainty grows during prediction and falls with observation', () => { const e = { x: 0, z: 0, vx: 1, vz: 0, variance: 4, heading: 0 }; const p = predict(e, 0, 0, 2, 1); expect(p.x).toBe(2); expect(p.variance).toBeGreaterThan(4); expect(observe(p, 2, 0, 1).variance).toBeLessThan(p.variance) })
  it('integrates angular velocity into orientation', () => { const e = { x: 0, z: 0, vx: 0, vz: 0, variance: 1, heading: 350 }; expect(predict(e, 0, 0, 1, 0, 20).heading).toBe(10) })
})
