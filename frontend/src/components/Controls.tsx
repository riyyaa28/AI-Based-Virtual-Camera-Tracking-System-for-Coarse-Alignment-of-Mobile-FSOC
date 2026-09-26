import type { ControlMessage, DeepPartial, SimulationConfig, SimulationState } from '../types/simulation'

export function Controls({ state, send }: { state: SimulationState; send: (message: ControlMessage) => void }) {
  const config = state.config
  const update = (configPatch: DeepPartial<SimulationConfig>) => send({ action: 'configure', config: configPatch })
  return <section className="controls-bar panel">
    <div className="run-controls">
      <button className="button primary" onClick={() => send({ action: 'start' })}>Start</button>
      <button className="button" onClick={() => send({ action: 'pause' })}>Pause</button>
      <button className="button ghost" onClick={() => send({ action: 'reset' })}>Reset</button>
    </div>
    <label className="compact-control">Speed <select value={config.simulation.speed} onChange={(e) => update({ simulation: { speed: Number(e.target.value) } })}><option value={0.5}>0.5×</option><option value={1}>1×</option><option value={2}>2×</option><option value={3}>3×</option></select></label>
    <label className="compact-control">Trajectory <select value={config.target.trajectory} onChange={(e) => update({ target: { trajectory: e.target.value } })}><option value="figure8">Figure 8</option><option value="circle">Circle</option><option value="straight">Straight</option><option value="random">Erratic</option></select></label>
    <label className="compact-control">Detector <select value={config.tracking.detector} onChange={(e) => send({ action: 'set_detector', detector: e.target.value })}><option value="simulated">Simulated vision</option><option value="classical">Classical ring</option><option value="yolo">YOLO</option></select></label>
  </section>
}
