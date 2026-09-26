import { useCallback, useEffect, useRef, useState } from 'react'
import { SimulationSocket, type ConnectionState } from '../services/simulationSocket'
import type { ControlMessage, SimulationState } from '../types/simulation'

export function useSimulation() {
  const [state, setState] = useState<SimulationState | null>(null)
  const [connection, setConnection] = useState<ConnectionState>('connecting')
  const socket = useRef<SimulationSocket | null>(null)

  useEffect(() => {
    const client = new SimulationSocket(setState, setConnection)
    socket.current = client
    client.connect()
    return () => client.close()
  }, [])

  const send = useCallback((message: ControlMessage) => socket.current?.send(message), [])
  return { state, connection, send }
}
