import { useState } from 'react'
import { Analytics } from './components/Analytics'
import { CameraView } from './components/CameraView'
import { Controls } from './components/Controls'
import { DisturbanceLab } from './components/DisturbanceLab'
import { Settings } from './components/Settings'
import { Telemetry } from './components/Telemetry'
import { useSimulation } from './hooks/useSimulation'
import { WorldScene } from './three/WorldScene'

type Tab = 'mission' | 'live' | 'camera' | 'disturbances' | 'analytics' | 'settings'
const tabs: [Tab, string][] = [['mission', 'Mission Control'], ['live', 'Live 3D Simulation'], ['camera', 'Camera / Tracking'], ['disturbances', 'Disturbance Lab'], ['analytics', 'Analytics'], ['settings', 'Settings']]

export default function App() {
  const [tab, setTab] = useState<Tab>('mission')
  const { state, connection, send } = useSimulation()
  if (!state) return <main className="loading"><div className="loader" /><h1>FSOC Mission Control</h1><p>{connection === 'offline' ? 'Backend unavailable — start FastAPI on port 8000.' : 'Connecting to the Python simulation…'}</p></main>
  const showWorld = tab === 'mission' || tab === 'live'
  return <main className="app-shell">
    <header className="topbar"><div className="brand"><span className="brand-mark">◈</span><div><h1>FSOC <em>MISSION CONTROL</em></h1><p>AI-assisted virtual camera tracking · coarse alignment</p></div></div><div className="connection"><span className={`status-dot ${connection}`} />{connection} · {state.running ? 'simulation running' : 'simulation paused'}</div></header>
    <nav className="tabs" aria-label="Simulation pages">{tabs.map(([id, label]) => <button key={id} className={tab === id ? 'selected' : ''} onClick={() => setTab(id)}>{label}</button>)}</nav>
    <Controls state={state} send={send} />
    {showWorld && <section className="mission-layout"><section className="world-panel panel"><header className="panel-heading"><div><p className="eyebrow">External world view</p><h2>UAV-to-UAV optical tracking</h2></div><span className={`pill ${state.environment.los ? '' : 'warning'}`}>{state.environment.los ? 'LOS CLEAR' : 'LOS BLOCKED'}</span></header><WorldScene state={state} /></section><Telemetry state={state} /></section>}
    {tab === 'camera' && <CameraView state={state} />}
    {tab === 'disturbances' && <DisturbanceLab state={state} send={send} />}
    {tab === 'analytics' && <Analytics state={state} />}
    {tab === 'settings' && <Settings state={state} send={send} />}
    {tab === 'mission' && <section className="mission-bottom"><CameraView state={state} /><Analytics state={state} /></section>}
  </main>
}
