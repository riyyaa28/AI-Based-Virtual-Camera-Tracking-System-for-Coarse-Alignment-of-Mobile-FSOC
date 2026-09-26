import type { SimulationState } from '../types/simulation'

const fmt = (value: number | null | undefined, suffix = '') => value == null ? '—' : `${value.toFixed(2)}${suffix}`

export function Telemetry({ state }: { state: SimulationState }) {
  const rows = [
    ['Target', [['Distance', fmt(state.target.distance, ' m')], ['Azimuth', fmt(state.target.azimuth, '°')], ['Elevation', fmt(state.target.elevation, '°')]]],
    ['Camera', [['Pan', fmt(state.camera.pan, '°')], ['Tilt', fmt(state.camera.tilt, '°')], ['FOV', `${state.camera.fov_horizontal}° × ${state.camera.fov_vertical}°`]]],
    ['Tracking', [['Error', fmt(state.tracking.angular_error, '°')], ['Confidence', fmt(state.beacon.confidence)], ['Detector', state.beacon.source]]]
  ]
  return <aside className="telemetry panel">
    <div className={`link-state ${state.fsoc.link ? 'active' : ''}`}><span className="status-dot" />{state.fsoc.link ? 'LINK ACTIVE' : 'LINK BLOCKED'}</div>
    <p className="link-reason">{state.fsoc.reason}</p>
    {rows.map(([title, entries]) => <div className="telemetry-group" key={title as string}><h3>{title}</h3>{(entries as string[][]).map(([key, value]) => <div className="readout" key={key}><span>{key}</span><strong>{value}</strong></div>)}</div>)}
    <div className="state-flow"><span>SEE</span><span>ACQUIRE</span><span>TRACK</span><span>PREDICT</span><span>CONTROL</span><span>ALIGN</span><span className={state.fsoc.link ? 'done' : ''}>LOCK</span></div>
  </aside>
}
