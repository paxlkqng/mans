import { Canvas, useFrame, useThree, type ThreeEvent } from '@react-three/fiber'
import { OrbitControls, Grid, Line } from '@react-three/drei'
import { useEffect, useMemo } from 'react'
import * as THREE from 'three'
import { CELL, height, nearestRouteIndex, type Point, type Terrain } from './nav'
import { useMission } from './store'

function world(t: Terrain, p: Point, raise = 2): [number, number, number] { return [(p.x - 32) * CELL, (height(t, p) - Math.min(...t.heights)) * 1.8 + raise, (p.z - 32) * CELL] }
function Ground({ terrain }: { terrain: Terrain }) {
  const selecting = useMission(s => s.selecting), setPoint = useMission(s => s.setPoint)
  const geometry = useMemo(() => { const g = new THREE.PlaneGeometry(64 * CELL, 64 * CELL, 64, 64); g.rotateX(-Math.PI / 2); const a = g.attributes.position; const colors = new Float32Array(a.count * 3)
    const low = Math.min(...terrain.heights), high = Math.max(...terrain.heights)
    for (let i = 0; i < a.count; i++) { const x = i % 65, z = Math.floor(i / 65), h = terrain.heights[z * 65 + x]; a.setY(i, (h - low) * 1.8)
      const east = terrain.heights[z * 65 + Math.min(64, x + 1)], south = terrain.heights[Math.min(64, z + 1) * 65 + x]; const grade = Math.min(1, Math.hypot(east - h, south - h) / 12); const elevation = (h - low) / (high - low)
      const color = new THREE.Color().setRGB(.36 + elevation * .25 + grade * .23, .19 + elevation * .17 - grade * .08, .11 + elevation * .09 - grade * .05); colors.set([color.r, color.g, color.b], i * 3) }
    g.setAttribute('color', new THREE.BufferAttribute(colors, 3)); g.computeVertexNormals(); return g }, [terrain])
  const onClick = (e: ThreeEvent<MouseEvent>) => { if (!selecting) return; e.stopPropagation(); setPoint({ x: Math.max(0, Math.min(64, Math.round(e.point.x / CELL + 32))), z: Math.max(0, Math.min(64, Math.round(e.point.z / CELL + 32))) }) }
  return <mesh geometry={geometry} onClick={onClick} receiveShadow><meshStandardMaterial vertexColors roughness={.92} metalness={0} side={THREE.DoubleSide} /></mesh>
}
function Marker({ terrain, point, color }: { terrain: Terrain; point: Point; color: string }) { const pos = world(terrain, point, 7)
  return <group position={pos}><mesh><sphereGeometry args={[6, 16, 16]} /><meshBasicMaterial color={color} /></mesh><mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, -4, 0]}><ringGeometry args={[11, 13, 32]} /><meshBasicMaterial color={color} side={THREE.DoubleSide} /></mesh></group> }
function CameraRig({ terrain, route, position }: { terrain: Terrain; route: Point[]; position: Point }) { const mode = useMission(s => s.mode); const target = route[Math.min(route.length - 1, nearestRouteIndex(route, position) + 5)] || position
  const camera = useThree(s => s.camera)
  useEffect(() => { if (mode === 'plan') { camera.position.set(620, 530, 620); camera.lookAt(0, 0, 0) } }, [camera, mode])
  useFrame(({ camera }) => { if (mode !== 'navigate') return; const p = world(terrain, position, 18), q = world(terrain, target, 18); camera.position.lerp(new THREE.Vector3(...p), .06); camera.lookAt(new THREE.Vector3(...q)) })
  return mode === 'plan' ? <OrbitControls makeDefault enablePan enableZoom minDistance={140} maxDistance={1500} maxPolarAngle={Math.PI / 2.1} /> : null }
function Objects({ terrain, route, alternative, position }: { terrain: Terrain; route: Point[]; alternative: Point[]; position: Point }) {
  const { start, goal, scenario, mode } = useMission()
  const routePts = route.filter((_, i) => i % 2 === 0 || i === route.length - 1).map(p => world(terrain, p, 5))
  const altPts = alternative.filter((_, i) => i % 3 === 0 || i === alternative.length - 1).map(p => world(terrain, p, 3))
  return <>
    {altPts.length > 1 && <Line points={altPts} color="#89a0a6" opacity={.55} transparent lineWidth={1.4} dashed dashSize={8} gapSize={6} />}
    {routePts.length > 1 && <Line points={routePts} color="#68e0e8" lineWidth={4} />}
    {mode === 'navigate' && routePts.slice(0, 30).map((p, i) => <mesh key={i} position={p} rotation={[-Math.PI / 2, 0, 0]}><ringGeometry args={[7, 8.5, 24]} /><meshBasicMaterial color="#6fe9ee" side={THREE.DoubleSide} transparent opacity={.5} /></mesh>)}
    <Marker terrain={terrain} point={start} color="#5ce3d9" /><Marker terrain={terrain} point={goal} color="#ffae78" />
    <Marker terrain={terrain} point={{ x: Math.round(position.x), z: Math.round(position.z) }} color="#ffffff" />
    {scenario === 'obstruction' && <mesh position={[(34 - 32) * CELL, 30, (47 - 32) * CELL]}><boxGeometry args={[10 * CELL, 60, 9 * CELL]} /><meshBasicMaterial color="#f06c53" transparent opacity={.18} /></mesh>}
  </> }
export function TerrainScene(props: { terrain: Terrain; route: Point[]; alternative: Point[]; position: Point }) { return <Canvas shadows camera={{ position: [620, 530, 620], fov: 48, near: .1, far: 4000 }} gl={{ antialias: true }}>
  <color attach="background" args={['#111615']} /><fog attach="fog" args={['#111615', 950, 2400]} /><hemisphereLight args={['#d8ddd0', '#32251c', 1.25]} /><directionalLight position={[300, 600, 150]} intensity={2.5} color="#f3d5a8" castShadow />
  <Ground terrain={props.terrain} /><Grid position={[0, -55, 0]} args={[1300, 1300]} cellSize={48} cellColor="#394039" sectionColor="#4c5145" fadeDistance={1300} fadeStrength={2} />
  <Objects {...props} /><CameraRig terrain={props.terrain} route={props.route} position={props.position} />
  </Canvas> }
