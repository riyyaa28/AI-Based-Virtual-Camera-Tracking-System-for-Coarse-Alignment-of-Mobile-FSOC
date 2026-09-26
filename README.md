# FSOC Virtual Camera Tracking System

A browser-based, closed-loop simulation of AI-assisted virtual-camera tracking for coarse alignment of mobile Free Space Optical Communication (FSOC) terminals. Two simulated UAVs move in a shared 3D world: UAV 1 carries a pan/tilt optical camera, and UAV 2 carries an optical beacon. Python is the source of truth for motion, camera observations, disturbances, detection, Kalman prediction, PID control, link conditions, and performance metrics.

This extends the original Python/PyQt beacon-tracking project. Its classical ring-signature detector, optional YOLO detector and weights, OpenCV image effects, Kalman tracker, and proportional PTZ helper are retained. The legacy desktop client still starts with `python main.py`; the browser application is the new primary interface.

## What it demonstrates

`SEE → ACQUIRE → DETECT → TRACK → PREDICT → CONTROL → ALIGN → LOCK`

- Two moving UAV/terminal models with configurable figure-eight, circle, straight, and erratic paths.
- A 3D Three.js scene driven only by WebSocket telemetry from the Python simulator.
- A virtual optical camera whose generated image is processed before its centroid is displayed.
- Detector choices: image-derived simulated highlight detector, the original classical ring detector, or the original `beacon_yolo.pt` YOLO detector.
- Existing OpenCV Kalman tracker plus a PID pan/tilt controller and a limited-rate, second-order gimbal response.
- Physically motivated platform vibration, camera sensor noise, atmospheric observation degradation, and gimbal motion effects.
- Segment-versus-building LOS check, visible green/orange/red optical beam, explicit alignment state machine, and non-trivial link decision.

## Architecture

```text
  Python simulator (authoritative)
  target motion / camera image / disturbances / detector / Kalman / PID / FSOC state
                                │
                             FastAPI
                                │
                     WebSocket state at 30 Hz
                                │
               React dashboard ─── Three.js renderer
              controls/config        3D UAVs, FOV, beam, terrain
                                │
                         WebSocket commands
                                └────────────── back to Python
```

React never invents target positions, camera angles, detector points, or link status. It renders the state sent by `backend/simulation.py`.

## Existing detector reference images

The repository's original OpenCV/PyQt outputs remain available as regression references for the reused vision stack. They are not substitutes for the live browser dashboard, which renders live WebSocket state after you start the application.

| Classical beacon tracking | YOLO beacon tracking |
|---|---|
| ![Classical circular tracking](01_classical_circular.png) | ![YOLO straight tracking](02_yolo_straight.png) |

## Technology stack

| Layer | Technology | Responsibility |
|---|---|---|
| Simulation, vision, control | Python, NumPy, OpenCV | Target motion, image formation, disturbances, detection, Kalman/PID, metrics, FSOC states |
| API | FastAPI, WebSocket | 30 Hz state streaming and command/configuration input |
| Interface | React, TypeScript, Vite | Pages, controls, telemetry, virtual camera overlay, analytics |
| 3D view | Three.js, React Three Fiber, Drei | UAVs, gimbal/FOV cone, beacon, beam, grid, obstacle, trajectory |
| Optional AI | Ultralytics YOLO | Existing trained beacon detector (`beacon_yolo.pt`) |

## Project layout

```text
fsoc-tracker-reference/
├── backend/                    # FastAPI, WebSocket hub, central config, FSOC engine
│   ├── main.py
│   ├── simulation.py
│   ├── detectors.py
│   ├── schemas.py
│   └── requirements.txt
├── frontend/                   # React + TypeScript + Three.js application
│   └── src/{components,hooks,services,three,types}/
├── control/ptz_controller.py   # Original proportional helper + new bounded PID
├── disturbance/                # Original OpenCV frame effects (reused by backend)
├── sim/                        # Original virtual-scene utilities (sky renderer reused)
├── vision/                     # Existing classical, YOLO, Kalman, fusion modules
├── tests/                      # Simulation, detector, controller and schema tests
├── beacon_yolo.pt              # Existing beacon detector model
└── main.py                     # Existing PyQt desktop simulator entry point
```

## Installation

Prerequisites: Python 3.10+ and Node.js 20+ (includes npm). On Windows, install both, then open PowerShell at this repository root.

```powershell
py -3 -m venv .venv
.\.venv\Scripts\Activate.ps1
py -3 -m pip install --upgrade pip
py -3 -m pip install -r backend\requirements.txt
```

Install optional YOLO support only if you intend to select **YOLO** in the dashboard:

```powershell
py -3 -m pip install ultralytics
```

For the original PyQt desktop client, additionally install `PyQt5` and `pyqtgraph`. The old pinned `requirements.txt` remains for historical reproduction of that desktop environment; it is not required by the browser application.

Install the browser client dependencies:

```powershell
cd frontend
npm install
cd ..
```

No environment variable is required for local development: Vite proxies `/ws` to FastAPI. For a separately deployed frontend, copy `frontend/.env.example` to `frontend/.env` and set `VITE_BACKEND_WS_URL` to the FastAPI WebSocket URL.

## Run the application

Terminal 1, from the project root:

```powershell
.\.venv\Scripts\Activate.ps1
py -3 -m uvicorn backend.main:app --host 127.0.0.1 --port 8000 --reload
```

Terminal 2:

```powershell
cd frontend
npm run dev
```

Open the URL printed by Vite (normally `http://127.0.0.1:5173`), then select **Start**. Use **Pause** and **Reset** for simulation control. The initial gimbal begins scanning, acquires the moving beacon when it enters its FOV, and progresses toward coarse alignment and link readiness if LOS and stability conditions are met.

Run backend tests:

```powershell
py -3 -m unittest discover -s tests -v
```

Build the production frontend:

```powershell
cd frontend
npm run build
```

## Simulation pipeline

1. Python updates UAV 2's configured 3D trajectory and calculates its range, azimuth, and elevation from UAV 1.
2. The gimbal's real pan/tilt evolves toward PID commands subject to rate limits, response delay, overshoot, settling, and platform vibration.
3. The virtual camera projects the beacon into the current FOV and forms a 640×480 OpenCV image. A blocked LOS or out-of-FOV beacon is not rendered.
4. Noise and turbulence are applied to that image, then the chosen detector processes it. The reported centroid is therefore an observation, not ground-truth world coordinates.
5. The original four-state OpenCV Kalman tracker estimates centroid and velocity. Its estimate is drawn as the predicted marker.
6. A PID controller converts optical-angle error into bounded pan/tilt commands. Stable small error, FOV, LOS, and valid tracking are required before `LINK_READY`.
7. FastAPI sends this state and camera JPEG to the browser. Three.js updates the external view and React updates the dashboard and charts.

### Tracking / FSOC states

`TARGET_NOT_VISIBLE`, `SEARCHING`, `ACQUIRED`, `TRACKING`, `COARSE_ALIGNED`, `LINK_READY`, and `TARGET_LOST` are computed from observation results, time since detection, angular error, stable duration, FOV, and LOS. The link does not activate on a timer: it requires a visible beacon, clear LOS, tracking stability, and error below the configured alignment threshold. The dashboard gives the current blocking reason.

## Disturbance models

| Model | What is simulated | Visible effect |
|---|---|---|
| Platform vibration | Multi-frequency high-frequency pan/tilt displacement with a small stochastic component | Gimbal/FOV jitter, image motion and worse angular error; presets OFF/LOW/MEDIUM/HIGH |
| Camera noise | Gaussian sensor noise using the original OpenCV effect plus a photon-count approximation | Noisy formed image and less reliable image-derived centroid |
| Atmospheric turbulence | Time-varying apparent beacon displacement, scintillation, blur, and stochastic detector dropout | Beacon wanders/degrades in the camera feed and confidence drops |
| Camera motion | Second-order gimbal response with limited velocity, damping, optional greater mechanical imperfection | Pan/tilt visibly lag, overshoot and settle rather than teleport |

Atmospheric turbulence is intentionally a practical image-domain approximation for a tracking demonstrator; it is not a physically exact wave-optics or atmospheric propagation model.

## Configuration and scenarios

The browser settings and controls update the central `DEFAULT_CONFIG` in `backend/simulation.py` through WebSocket. It covers simulation time scale, target path/velocity/amplitude, FOV and gimbal limits, detector frequency and alignment requirements, every disturbance, the obstacle, and atmospheric attenuation. Configuration remains in Python so state sent to every browser client is consistent.

Enable **Obstacle** in Settings to place the configured building on the direct segment between terminals. The backend AABB intersection causes LOS to fail; the beam changes colour and the link reports `LOS obstruction`.

## Metrics

Analytics are accumulated by the backend: acquisition time, average and maximum angular error, alignment/lock retention, latest detection confidence, stream rate, per-tick processing latency, and target-loss count. The chart is the backend's bounded recent angular-error history.

## Troubleshooting

- `No installed Python found`: install Python 3.10+ from python.org, select **Add Python to PATH**, then reopen PowerShell. `py -3 --version` should report a version.
- `npm is not recognized`: install Node.js 20 LTS, reopen PowerShell, and check `node --version` and `npm --version`.
- Frontend says backend unavailable: start the FastAPI command first and confirm `http://127.0.0.1:8000/api/health` returns JSON.
- YOLO selection falls back to simulated vision: install the original requirements, ensure `beacon_yolo.pt` remains at the repository root, and use a Python/PyTorch build supported by your system.
- The link remains blocked: start the simulation, wait for acquisition, ensure the obstacle is disabled, reduce disturbances, or increase FOV/alignment tolerance in Settings.

## Known limitations and future work

- UAV geometry and terrain are portable low-poly procedural Three.js models; they are not proprietary Unreal assets.
- LOS uses a simplified box-obstacle test. The link condition is a coarse operational model rather than a full optical link-budget calculation.
- The camera feed is image-domain synthetic. Atmospheric behavior approximates optical observation effects, not a validated propagation solver.
- YOLO inference is optional and selected explicitly because it is substantially more expensive than the default image-derived detector. A production deployment can place it in a dedicated worker process.
- Future extensions: physical link-budget/received-power model, richer obstacle meshes, recorded scenario replay, a background YOLO worker, and hardware-in-the-loop telemetry adapters.
