import { create } from 'zustand'
import type { Mode, Point, RouteKind, Scenario } from './nav'
type State = { mode: Mode; scenario: Scenario; routeKind: RouteKind; start: Point; goal: Point; playing: boolean; progress: number; selecting: 'start' | 'goal' | null;
  setMode: (v: Mode) => void; setScenario: (v: Scenario) => void; setRouteKind: (v: RouteKind) => void; setPoint: (v: Point) => void; setStart: (v: Point) => void; setSelecting: (v: 'start' | 'goal' | null) => void; toggle: () => void; reset: () => void; tick: (v: number) => void }
export const useMission = create<State>((set) => ({ mode: 'plan', scenario: 'normal', routeKind: 'safest', start: { x: 8, z: 11 }, goal: { x: 56, z: 51 }, playing: false, progress: 0, selecting: null,
  setMode: mode => set({ mode, playing: false }), setScenario: scenario => set({ scenario, progress: 0, playing: false }), setRouteKind: routeKind => set({ routeKind, progress: 0 }),
  setPoint: point => set(s => ({ [s.selecting || 'goal']: point, selecting: null, progress: 0 })), setStart: start => set({ start, progress: 0, playing: false }), setSelecting: selecting => set({ selecting }), toggle: () => set(s => ({ playing: !s.playing })), reset: () => set({ progress: 0, playing: false }), tick: progress => set({ progress }) }))
