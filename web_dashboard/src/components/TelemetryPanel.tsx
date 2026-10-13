import type { Pose2D } from '../hooks/useRosConnection'
import type { ViewerObstacle } from './RobotViewer'
import { batteryStatus, estimateSecondsRemaining, formatDuration } from '../lib/batteryEstimate'

function Stat({ label, value, unit }: { label: string; value: string; unit?: string }) {
  return (
    <div className="float-stat">
      <div className="float-stat-label">{label}</div>
      <div className="float-stat-value">
        {value}
        {unit && <span className="float-stat-unit">{unit}</span>}
      </div>
    </div>
  )
}

const KIND_LABEL: Record<ViewerObstacle['kind'], string> = { person: 'Orang', robot: 'Robot lab' }

function ObstacleList({ obstacles, pose }: { obstacles: ViewerObstacle[]; pose: Pose2D | null }) {
  if (obstacles.length === 0) return null
  return (
    <div className="float-obstacles">
      <div className="float-stat-label">Obstacle terdeteksi · {obstacles.length}</div>
      <div className="obstacle-chip-row">
        {obstacles.map((o) => {
          const dist = pose ? Math.hypot(o.x - pose.x, o.y - pose.y) : null
          return (
            <div key={o.id} className={`obstacle-chip obstacle-chip-${o.kind} ${o.moving ? 'is-moving' : ''}`}>
              <span className="obstacle-chip-dot" />
              <span className="obstacle-chip-label">{o.kind === 'person' ? KIND_LABEL.person : o.label}</span>
              {dist != null && <span className="obstacle-chip-dist">{dist.toFixed(1)}m</span>}
              <span className="obstacle-chip-state">{o.moving ? 'bergerak' : 'diam'}</span>
            </div>
          )
        })}
      </div>
    </div>
  )
}

export function TelemetryPanel({
  statusLabel,
  statusKind,
  pose,
  batteryPercentage,
  batteryVoltage,
  batteryHistory,
  speed,
  obstacles = [],
}: {
  statusLabel: string
  statusKind: 'connected' | 'connecting' | 'disconnected' | 'error'
  pose: Pose2D | null
  batteryPercentage: number
  batteryVoltage: number
  batteryHistory: { t: number; pct: number }[]
  speed: number
  obstacles?: ViewerObstacle[]
}) {
  const batteryPct = Math.round(batteryPercentage * 100)
  const status = batteryStatus(batteryPct)
  const remaining = estimateSecondsRemaining(batteryHistory)

  return (
    <>
      <div className="float-header">
        <h1>HUSKY A200</h1>
        <div className={`float-status status-${statusKind}`}>
          <span className="status-dot" />
          {statusLabel}
        </div>
      </div>

      <div className="float-bottom">
        <div className="float-battery">
          <div className={`float-battery-value status-text-${status}`}>{batteryPct}%</div>
          <div className="float-stat-label">
            Baterai · {batteryVoltage.toFixed(1)}V · bertahan {formatDuration(remaining)}
          </div>
        </div>

        <div className="float-stat-row">
          <Stat label="Posisi X" value={pose ? pose.x.toFixed(2) : '—'} unit="m" />
          <Stat label="Posisi Y" value={pose ? pose.y.toFixed(2) : '—'} unit="m" />
          <Stat
            label="Heading"
            value={pose ? ((pose.yaw * 180) / Math.PI).toFixed(0) : '—'}
            unit="°"
          />
          <Stat label="Kecepatan" value={speed.toFixed(2)} unit="m/s" />
        </div>

        <ObstacleList obstacles={obstacles} pose={pose} />
      </div>
    </>
  )
}
