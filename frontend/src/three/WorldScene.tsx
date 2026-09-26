import { OrbitControls, Line, Grid, Html } from '@react-three/drei'
import { Canvas } from '@react-three/fiber'
import { useMemo } from 'react'
import * as THREE from 'three'
import type { SimulationState, Vec3 } from '../types/simulation'

function Drone({ position, color, label, gimbalPan, gimbalTilt }: { position: Vec3; color: string; label: string; gimbalPan?: number; gimbalTilt?: number }) {
  const pan = THREE.MathUtils.degToRad(gimbalPan ?? 0)
  const tilt = THREE.MathUtils.degToRad(gimbalTilt ?? 0)
  return (
    <group position={position}>
      <mesh castShadow><boxGeometry args={[5, 1.3, 3.2]} /><meshStandardMaterial color={color} metalness={0.72} roughness={0.25} /></mesh>
      {[-1, 1].flatMap((x) => [-1, 1].map((z) => (
        <group key={`${x}-${z}`} position={[x * 3.2, 0, z * 2.5]}>
          <mesh rotation={[0, 0, z * 0.28]}><cylinderGeometry args={[0.14, 0.14, 3.8, 8]} /><meshStandardMaterial color="#8fa5b9" /></mesh>
          <mesh position={[0, 0.1, 0]} rotation={[Math.PI / 2, 0, 0]}><cylinderGeometry args={[1.35, 1.35, 0.12, 16]} /><meshStandardMaterial color="#26354a" /></mesh>
        </group>
      )))}
      {gimbalPan !== undefined && (
        <group position={[1.5, -0.9, 0]} rotation={[0, -pan + Math.PI / 2, tilt]}>
          <mesh><sphereGeometry args={[0.72, 16, 16]} /><meshStandardMaterial color="#d7e4ef" metalness={0.8} /></mesh>
          <mesh position={[0.75, 0, 0]} rotation={[0, 0, Math.PI / 2]}><cylinderGeometry args={[0.28, 0.38, 1.1, 12]} /><meshStandardMaterial color="#111827" /></mesh>
        </group>
      )}
      <Html position={[0, 4.7, 0]} center distanceFactor={13}><div className="scene-label">{label}</div></Html>
    </group>
  )
}

function FovCone({ state }: { state: SimulationState }) {
  const [x, y, z] = state.uav1.position
  const pan = THREE.MathUtils.degToRad(state.camera.pan)
  const tilt = THREE.MathUtils.degToRad(state.camera.tilt)
  const radius = Math.tan(THREE.MathUtils.degToRad(state.camera.fov_horizontal / 2)) * 34
  return (
    <group position={[x, y, z]} rotation={[0, -pan + Math.PI / 2, tilt]}>
      <mesh position={[17, 0, 0]} rotation={[0, 0, -Math.PI / 2]}>
        <coneGeometry args={[radius, 34, 32, 1, true]} />
        <meshBasicMaterial color="#42d7ff" transparent opacity={0.07} side={THREE.DoubleSide} depthWrite={false} />
      </mesh>
      <Line points={[[0, 0, 0], [34, 0, 0]]} color="#63e6ff" transparent opacity={0.75} lineWidth={1} />
    </group>
  )
}

function World({ state }: { state: SimulationState }) {
  const beamColor = state.fsoc.link ? '#3af4a3' : state.environment.los ? '#f6ad55' : '#ff5d70'
  const targetPath = useMemo(() => state.target_path ?? [], [state.target_path])
  const obstacle = state.environment.obstacle
  return (
    <>
      <color attach="background" args={['#06101d']} />
      <fog attach="fog" args={['#06101d', 110, 310]} />
      <ambientLight intensity={0.45} />
      <directionalLight position={[70, 120, 30]} intensity={1.8} castShadow />
      <pointLight position={state.uav2.position} intensity={state.beacon.visible ? 2.4 : 0.3} color="#9ee8ff" distance={40} />
      <Grid args={[420, 420]} cellSize={10} cellThickness={0.5} cellColor="#19344d" sectionSize={50} sectionColor="#2e6888" fadeDistance={330} infiniteGrid />
      <mesh position={[0, -1.1, 0]} receiveShadow><planeGeometry args={[420, 420]} /><meshStandardMaterial color="#071824" roughness={0.95} /></mesh>
      {obstacle.enabled && <mesh position={obstacle.center} castShadow receiveShadow><boxGeometry args={obstacle.size} /><meshStandardMaterial color="#344a61" transparent opacity={0.9} /></mesh>}
      <Drone position={state.uav1.position} color="#1f91cc" label="UAV 1 · TRACKER" gimbalPan={state.camera.pan} gimbalTilt={state.camera.tilt} />
      <Drone position={state.uav2.position} color="#d78148" label="UAV 2 · BEACON" />
      <mesh position={state.uav2.position}><sphereGeometry args={[0.85, 16, 16]} /><meshBasicMaterial color={state.beacon.visible ? '#e6faff' : '#6e8193'} /></mesh>
      <Line points={[state.uav1.position, state.uav2.position]} color={beamColor} lineWidth={2.5} />
      {targetPath.length > 1 && <Line points={targetPath} color="#eea86c" transparent opacity={0.5} lineWidth={1} />}
      <FovCone state={state} />
      <OrbitControls makeDefault target={[75, 22, 10]} minDistance={35} maxDistance={310} maxPolarAngle={Math.PI / 2.05} />
    </>
  )
}

export function WorldScene({ state }: { state: SimulationState }) {
  return <div className="world-canvas"><Canvas shadows camera={{ position: [-58, 70, 144], fov: 47 }} dpr={[1, 1.7]}><World state={state} /></Canvas></div>
}
