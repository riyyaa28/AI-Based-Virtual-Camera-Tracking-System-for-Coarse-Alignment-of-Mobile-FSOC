"""Pydantic schemas used at the HTTP and WebSocket boundary."""

from __future__ import annotations

from typing import Any, Literal

from pydantic import BaseModel, Field


class ControlMessage(BaseModel):
    action: Literal["start", "pause", "reset", "configure", "set_detector"]
    config: dict[str, Any] = Field(default_factory=dict)
    detector: str | None = None


class HealthResponse(BaseModel):
    status: str
    clients: int

