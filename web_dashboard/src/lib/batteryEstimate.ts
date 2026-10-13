export interface BatterySample {
  t: number // ms epoch
  pct: number // 0..100
}

/** Estimasi waktu tersisa dari slope persentase terkini (regresi linear
 * sederhana atas beberapa sample terakhir) — BUKAN dari kapasitas Wh riil.
 *
 * Catatan robot riil: BMS Husky A200 (/a200_1060/platform/bms/state)
 * biasanya juga expose `current` (A). Estimasi paling akurat di lapangan
 * adalah energi_tersisa_Wh / daya_terpakai_W (voltage * |current|), bukan
 * slope persentase — slope di sini dipakai sebagai proxy generik karena di
 * simulasi malam ini current cuma angka konstan per state (diam/gerak).
 */
export function estimateSecondsRemaining(history: BatterySample[]): number | null {
  if (history.length < 2) return null
  const recent = history.slice(-8)
  const first = recent[0]
  const last = recent[recent.length - 1]
  const dtSec = (last.t - first.t) / 1000
  if (dtSec <= 0) return null
  const dPct = last.pct - first.pct
  if (dPct >= -0.01) return null // diam/charging, tidak ada estimasi habis yg masuk akal
  const ratePctPerSec = dPct / dtSec
  return last.pct / -ratePctPerSec
}

export function formatDuration(totalSeconds: number | null): string {
  if (totalSeconds == null || !Number.isFinite(totalSeconds)) return '—'
  if (totalSeconds > 60 * 60 * 99) return '99h+'
  const totalMinutes = Math.round(totalSeconds / 60)
  const h = Math.floor(totalMinutes / 60)
  const m = totalMinutes % 60
  if (h === 0) return `${m}m`
  return `${h}j ${m}m`
}

export type BatteryStatus = 'good' | 'warning' | 'serious' | 'critical'

export function batteryStatus(pct: number): BatteryStatus {
  if (pct <= 15) return 'critical'
  if (pct <= 30) return 'serious'
  if (pct <= 50) return 'warning'
  return 'good'
}
