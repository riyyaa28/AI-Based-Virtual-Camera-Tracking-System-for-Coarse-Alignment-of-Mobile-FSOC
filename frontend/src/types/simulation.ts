export type Vec3 = [number, number, number]

export interface SimulationState {
  timestamp: number
  running: boolean
  uav1: { position: Vec3; rotation: Vec3 }
  uav2: { position: Vec3; velocity: Vec3; rotation: Vec3 }
  camera: { pan: number; tilt: number; commanded_pan: number; commanded_tilt: number; fov_horizontal: number; fov_vertical: number }
  beacon: { visible: boolean; actual: [number, number] | null; x: number | null; y: number | null; confidence: number; source: string }
  tracking: { error_x: number; error_y: number; angular_error: number; predicted_x: number | null; predicted_y: number | null; state: string }
  disturbances: { platform_vibration: number; vibration_level: string; camera_noise: number; atmospheric_turbulence: number; camera_motion: number }
  target: { distance: number; azimuth: number; elevation: number }
  environment: { los: boolean; atmospheric_loss: number; obstacle: { enabled: boolean; center: Vec3; size: Vec3 } }
  fsoc: { fov_ok: boolean; los_ok: boolean; alignment_ok: boolean; link: boolean; reason: string }
  metrics: { acquisition_time: number | null; average_angular_error: number; maximum_angular_error: number; lock_retention: number; fps: number; processing_latency: number; target_losses: number }
  history: { error: number[]; confidence: number[]; distance: number[]; time: number[] }
  camera_view: { width: number; height: number; image: string }
  config: SimulationConfig
  target_path?: Vec3[]
}

export interface SimulationConfig {
  simulation: { dt: number; speed: number; duration: number }
  target: { trajectory: string; velocity: number; amplitude: number }
  camera: { fov_horizontal: number; fov_vertical: number; pan_limit: number; tilt_limit: number; max_rate: number }
  tracking: { detector: string; prediction: boolean; alignment_threshold: number; stable_duration: number; detector_hz: number }
  disturbances: { vibration: { level: string; amplitude: number; frequency: number }; camera_noise: number; turbulence: number; camera_motion: number }
  environment: { obstacles: boolean; obstacle: { enabled: boolean; center: Vec3; size: Vec3 }; atmospheric_attenuation: number }
}

export type DeepPartial<T> = {
  [Key in keyof T]?: T[Key] extends object ? DeepPartial<T[Key]> : T[Key]
}

export type ControlMessage =
  | { action: 'start' | 'pause' | 'reset' }
  | { action: 'configure'; config: DeepPartial<SimulationConfig> }
  | { action: 'set_detector'; detector: string }
