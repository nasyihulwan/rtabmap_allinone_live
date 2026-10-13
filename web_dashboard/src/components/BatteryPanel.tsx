import { useMemo } from 'react'
import type { SimBattery } from '../hooks/useSimEngine'
import { batteryStatus, estimateSecondsRemaining, formatDuration } from '../lib/batteryEstimate'

const STATUS_LABEL: Record<string, string> = {
  good: 'Baik',
  warning: 'Perhatian',
  serious: 'Rendah',
  critical: 'Kritis',
}

function Sparkline({ history }: { history: { t: number; pct: number }[] }) {
  const width = 560
  const height = 160
  const padL = 34
  const padR = 12
  const padT = 16
  const padB = 24

  const { pathD, areaD, points, yForPct, minT } = useMemo(() => {
    if (history.length < 2) {
      return { pathD: '', areaD: '', points: [] as { x: number; y: number; pct: number }[], yForPct: () => 0, minT: 0 }
    }
    const t0 = history[0].t
    const t1 = history[history.length - 1].t
    const xForT = (t: number) => padL + ((t - t0) / Math.max(1, t1 - t0)) * (width - padL - padR)
    const yForPct = (pct: number) => padT + (1 - pct / 100) * (height - padT - padB)
    const pts = history.map((s) => ({ x: xForT(s.t), y: yForPct(s.pct), pct: s.pct }))
    const line = pts.map((p, i) => `${i === 0 ? 'M' : 'L'}${p.x.toFixed(1)},${p.y.toFixed(1)}`).join(' ')
    const baseline = yForPct(0)
    const area = `${line} L${pts[pts.length - 1].x.toFixed(1)},${baseline} L${pts[0].x.toFixed(1)},${baseline} Z`
    return { pathD: line, areaD: area, points: pts, yForPct, minT: t0 }
  }, [history])

  if (points.length < 2) {
    return <div className="battery-chart-empty">Mengumpulkan data riwayat…</div>
  }

  const last = points[points.length - 1]
  const status = batteryStatus(last.pct)
  const gridPcts = [25, 50, 75]

  return (
    <svg className="battery-chart" viewBox={`0 0 ${width} ${height}`} role="img" aria-label="Riwayat persentase baterai">
      {gridPcts.map((g) => (
        <g key={g}>
          <line x1={padL} x2={width - padR} y1={yForPct(g)} y2={yForPct(g)} className="chart-gridline" />
          <text x={padL - 8} y={yForPct(g)} className="chart-axis-label" textAnchor="end" dominantBaseline="middle">
            {g}%
          </text>
        </g>
      ))}
      <path d={areaD} className={`chart-area status-fill-${status}`} />
      <path d={pathD} className={`chart-line status-stroke-${status}`} />
      <circle cx={last.x} cy={last.y} r={5} className={`chart-end-dot status-fill-${status}`} />
      <text x={last.x} y={last.y - 12} className="chart-end-label" textAnchor="end">
        {last.pct.toFixed(0)}%
      </text>
      <text x={padL} y={height - 6} className="chart-axis-label">
        {new Date(minT).toLocaleTimeString('id-ID', { hour: '2-digit', minute: '2-digit', second: '2-digit' })}
      </text>
      <text x={width - padR} y={height - 6} className="chart-axis-label" textAnchor="end">
        sekarang
      </text>
    </svg>
  )
}

export function BatteryPanel({ battery, moving }: { battery: SimBattery; moving: boolean }) {
  const pct = battery.percentage * 100
  const status = batteryStatus(pct)
  const historyForEstimate = battery.history.map((h) => ({ t: h.t, pct: h.pct }))
  const remainingSeconds = estimateSecondsRemaining(historyForEstimate)

  return (
    <div className="panel-detail battery-panel">
      <header className="panel-detail-header">
        <h2>Baterai</h2>
        <span className={`status-chip status-chip-${status}`}>{STATUS_LABEL[status]}</span>
      </header>

      <div className="battery-hero-row">
        <div className="battery-hero">
          <div className="battery-hero-value">{pct.toFixed(0)}<span>%</span></div>
          <div className="battery-hero-sub">
            {moving ? 'Bergerak — konsumsi daya meningkat' : 'Idle'}
          </div>
        </div>
        <div className="battery-gauge" aria-hidden="true">
          <div className="battery-gauge-shell">
            <div
              className={`battery-gauge-fill status-fill-${status}`}
              style={{ height: `${Math.max(4, pct)}%` }}
            />
          </div>
          <div className="battery-gauge-cap" />
        </div>
        <div className="battery-eta">
          <div className="battery-eta-label">Estimasi bertahan</div>
          <div className="battery-eta-value">{formatDuration(remainingSeconds)}</div>
          <div className="battery-eta-note">berdasar laju pemakaian saat ini</div>
        </div>
      </div>

      <div className="battery-stat-grid">
        <div className="battery-stat">
          <div className="battery-stat-label">Voltase</div>
          <div className="battery-stat-value">{battery.voltage.toFixed(2)} V</div>
        </div>
        <div className="battery-stat">
          <div className="battery-stat-label">Arus</div>
          <div className="battery-stat-value">{battery.current.toFixed(1)} A</div>
        </div>
        <div className="battery-stat">
          <div className="battery-stat-label">Daya</div>
          <div className="battery-stat-value">{(battery.voltage * Math.abs(battery.current)).toFixed(0)} W</div>
        </div>
      </div>

      <div className="battery-chart-card">
        <div className="battery-chart-title">Riwayat baterai</div>
        <Sparkline history={battery.history} />
      </div>

      <div className="sim-note">
        <strong>Data riil</strong> dari topic <code>/a200_1060/platform/bms/state</code>. Estimasi
        waktu tersisa masih dihitung dari slope persentase terkini (regresi sederhana atas beberapa
        sample terakhir), belum dari kapasitas baterai (Wh) dibagi daya terpakai riil (V×A) BMS.
      </div>
    </div>
  )
}
