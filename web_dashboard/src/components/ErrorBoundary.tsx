import { Component, type ReactNode } from 'react'

interface Props {
  children: ReactNode
}
interface State {
  error: Error | null
}

// Tanpa ini, error render di mana pun (mis. bug di MapPanel) bikin React
// unmount SELURUH tree tanpa fallback — termasuk <Canvas> three.js, yang
// kadang tidak sempat cleanup rAF loop-nya sendiri (jadi "nyangkut" jalan
// terus meski komponennya sudah dianggap hilang oleh React).
export class ErrorBoundary extends Component<Props, State> {
  state: State = { error: null }

  static getDerivedStateFromError(error: Error): State {
    return { error }
  }

  render() {
    if (this.state.error) {
      return (
        <div className="error-boundary">
          <h2>Terjadi error di dashboard</h2>
          <p>{this.state.error.message}</p>
          <button onClick={() => this.setState({ error: null })}>Coba lagi</button>
        </div>
      )
    }
    return this.props.children
  }
}
