"""FastAPI application and 30 Hz WebSocket state broadcaster."""

from __future__ import annotations

import asyncio
import time
from contextlib import asynccontextmanager

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware

from .schemas import ControlMessage, HealthResponse
from .simulation import FSOCSimulation


class SimulationHub:
    def __init__(self) -> None:
        self.simulation = FSOCSimulation()
        self.clients: set[WebSocket] = set()
        self.task: asyncio.Task[None] | None = None
        self.last_tick = time.perf_counter()

    async def start(self) -> None:
        self.task = asyncio.create_task(self._broadcast_loop())

    async def stop(self) -> None:
        if self.task:
            self.task.cancel()
            try:
                await self.task
            except asyncio.CancelledError:
                pass

    async def _broadcast_loop(self) -> None:
        while True:
            now = time.perf_counter()
            state = self.simulation.step(now - self.last_tick)
            self.last_tick = now
            stale: list[WebSocket] = []
            for socket in self.clients.copy():
                try:
                    await socket.send_json(state)
                except Exception:
                    stale.append(socket)
            for socket in stale:
                self.clients.discard(socket)
            await asyncio.sleep(1 / 30)

    def handle(self, message: ControlMessage) -> None:
        if message.action == "start":
            self.simulation.set_running(True)
        elif message.action == "pause":
            self.simulation.set_running(False)
        elif message.action == "reset":
            self.simulation.reset()
        elif message.action == "set_detector" and message.detector:
            self.simulation.configure({"tracking": {"detector": message.detector}})
        elif message.action == "configure":
            self.simulation.configure(message.config)


hub = SimulationHub()


@asynccontextmanager
async def lifespan(_: FastAPI):
    await hub.start()
    yield
    await hub.stop()


app = FastAPI(title="FSOC Virtual Camera Tracker", version="1.0.0", lifespan=lifespan)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:5173", "http://127.0.0.1:5173"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.get("/api/health", response_model=HealthResponse)
async def health() -> HealthResponse:
    return HealthResponse(status="ok", clients=len(hub.clients))


@app.get("/api/state")
async def state() -> dict:
    return hub.simulation.step(0.001)


@app.websocket("/ws/simulation")
async def simulation_socket(websocket: WebSocket) -> None:
    await websocket.accept()
    hub.clients.add(websocket)
    try:
        while True:
            payload = await websocket.receive_json()
            hub.handle(ControlMessage.model_validate(payload))
    except WebSocketDisconnect:
        pass
    finally:
        hub.clients.discard(websocket)
