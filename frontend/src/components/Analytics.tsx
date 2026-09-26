import { ErrorChart } from './DisturbanceLab'
import type { SimulationState } from '../types/simulation'

export function Analytics({ state }: { state: SimulationState }) {
  const metrics = [
    ['Acquisition time', state.metrics.acquisition_time == null ? 'Awaiting' : `${state.metrics.acquisition_time.toFixed(2)} s`],
    ['Average angular error', `${state.metrics.average_angular_error.toFixed(2)}°`],
    ['Lock retention', `${state.metrics.lock_retention.toFixed(1)}%`],
    ['Detection confidence', state.beacon.confidence.toFixed(2)],
    ['Stream rate', `${state.metrics.fps.toFixed(0)} Hz`],
    ['Pipeline latency', `${state.metrics.processing_latency.toFixed(1)} ms`],
    ['Target losses', `${state.metrics.target_losses}`]
  ]
  return <section className="analytics-layout"><ErrorChart state={state} /><div className="metric-grid">{metrics.map(([label, value]) => <article className="metric-card panel" key={label}><span>{label}</span><strong>{value}</strong></article>)}</div></section>
}
