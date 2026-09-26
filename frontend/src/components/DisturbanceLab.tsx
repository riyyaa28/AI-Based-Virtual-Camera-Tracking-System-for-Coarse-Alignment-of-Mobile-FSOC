import type { ControlMessage, DeepPartial, SimulationConfig, SimulationState } from '../types/simulation'

function Slider({ label, value, onChange, note }: { label: string; value: number; onChange: (value: number) => void; note: string }) {
  return <div className="slider-row"><div><strong>{label}</strong><small>{note}</small></div><input aria-label={label} type="range" min="0" max="1" step="0.01" value={value} onChange={(e) => onChange(Number(e.target.value))} /><output>{Math.round(value * 100)}%</output></div>
}

export function DisturbanceLab({ state, send }: { state: SimulationState; send: (message: ControlMessage) => void }) {
  const d = state.config.disturbances
  const configure = (config: DeepPartial<SimulationConfig>) => send({ action: 'configure', config })
  return <section className="disturbance-layout">
    <div className="panel lab-controls"><header className="panel-heading"><div><p className="eyebrow">Image and platform effects</p><h2>Disturbance Lab</h2></div></header>
      <label className="select-row"><span>Platform vibration</span><select value={d.vibration.level} onChange={(e) => configure({ disturbances: { vibration: { level: e.target.value } } })}><option>OFF</option><option>LOW</option><option>MEDIUM</option><option>HIGH</option></select></label>
      <Slider label="Camera noise" value={d.camera_noise} note="Gaussian and photon-count variation in the image" onChange={(value) => configure({ disturbances: { camera_noise: value } })} />
      <Slider label="Atmospheric turbulence" value={d.turbulence} note="Apparent jitter, scintillation, blur and detection degradation" onChange={(value) => configure({ disturbances: { turbulence: value } })} />
      <Slider label="Camera motion" value={d.camera_motion} note="Mechanical response delay, overshoot and settling" onChange={(value) => configure({ disturbances: { camera_motion: value } })} />
    </div>
    <ErrorChart state={state} />
  </section>
}

export function ErrorChart({ state }: { state: SimulationState }) {
  const values = state.history.error.slice(-100)
  const max = Math.max(1, ...values)
  const points = values.map((value, index) => `${index / Math.max(1, values.length - 1) * 100},${100 - value / max * 88}`).join(' ')
  return <section className="panel chart-panel"><header className="panel-heading"><div><p className="eyebrow">Live response</p><h2>Angular tracking error</h2></div><strong>{state.tracking.angular_error.toFixed(2)}°</strong></header><svg viewBox="0 0 100 100" preserveAspectRatio="none" role="img" aria-label="Angular tracking error history"><polyline points={points} fill="none" stroke="#5de3ff" strokeWidth="2" vectorEffect="non-scaling-stroke" /></svg><div className="chart-footer"><span>0 s</span><span>current</span><span>peak {state.metrics.maximum_angular_error.toFixed(2)}°</span></div></section>
}
