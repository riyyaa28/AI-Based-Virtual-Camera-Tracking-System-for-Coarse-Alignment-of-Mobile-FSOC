import type { ControlMessage, SimulationState } from '../types/simulation'

export type ConnectionState = 'connecting' | 'connected' | 'offline'

export class SimulationSocket {
  private socket: WebSocket | null = null
  private retryId: number | null = null
  private closedByUser = false

  constructor(
    private readonly onState: (state: SimulationState) => void,
    private readonly onConnection: (state: ConnectionState) => void
  ) {}

  connect() {
    this.closedByUser = false
    this.onConnection('connecting')
    const protocol = window.location.protocol === 'https:' ? 'wss' : 'ws'
    const configuredUrl = import.meta.env.VITE_BACKEND_WS_URL as string | undefined
    const url = configuredUrl || `${protocol}://${window.location.host}/ws/simulation`
    this.socket = new WebSocket(url)
    this.socket.onopen = () => this.onConnection('connected')
    this.socket.onmessage = (event) => {
      try { this.onState(JSON.parse(event.data) as SimulationState) } catch { /* ignore malformed state */ }
    }
    this.socket.onclose = () => {
      this.onConnection('offline')
      if (!this.closedByUser) this.retryId = window.setTimeout(() => this.connect(), 1500)
    }
    this.socket.onerror = () => this.socket?.close()
  }

  send(message: ControlMessage) {
    if (this.socket?.readyState === WebSocket.OPEN) this.socket.send(JSON.stringify(message))
  }

  close() {
    this.closedByUser = true
    if (this.retryId !== null) window.clearTimeout(this.retryId)
    this.socket?.close()
  }
}
