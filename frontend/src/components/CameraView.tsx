import type { SimulationState } from '../types/simulation'

function Marker({ point, className, state }: { point: [number, number] | null; className: string; state: SimulationState }) {
  if (!point) return null
  return <span className={`camera-marker ${className}`} style={{ left: `${point[0] / state.camera_view.width * 100}%`, top: `${point[1] / state.camera_view.height * 100}%` }} />
}

export function CameraView({ state }: { state: SimulationState }) {
  const measured = state.beacon.x == null || state.beacon.y == null ? null : [state.beacon.x, state.beacon.y] as [number, number]
  const predicted = state.tracking.predicted_x == null || state.tracking.predicted_y == null ? null : [state.tracking.predicted_x, state.tracking.predicted_y] as [number, number]
  return <section className="camera-view panel">
    <header className="panel-heading"><div><p className="eyebrow">UAV 1 optical sensor</p><h2>Virtual Camera View</h2></div><span className="pill">{state.tracking.state.replace(/_/g, ' ')}</span></header>
    <div className="camera-image">
      {state.camera_view.image ? <img src={state.camera_view.image} alt="Python-generated virtual camera view" /> : <div className="camera-empty">Camera stream initializing</div>}
      <div className="crosshair" />
      <Marker point={state.beacon.actual} className="actual" state={state} />
      <Marker point={measured} className="measured" state={state} />
      <Marker point={predicted} className="predicted" state={state} />
      {measured && <span className="detector-box" style={{ left: `${measured[0] / state.camera_view.width * 100}%`, top: `${measured[1] / state.camera_view.height * 100}%` }} />}
    </div>
    <div className="legend"><span><i className="actual" />Actual</span><span><i className="measured" />Detected centroid</span><span><i className="predicted" />Kalman prediction</span></div>
  </section>
}
