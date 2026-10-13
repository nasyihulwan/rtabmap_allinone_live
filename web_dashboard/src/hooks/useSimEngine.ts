import { useCallback, useEffect, useRef, useState } from 'react'
import type { MapMeta, SemanticObject, WalkableGrid } from '../lib/mapData'
import { gridCellToWorld, worldToGridCell } from '../lib/mapData'
import { findPath, nearestWalkable, pathLength, pointAtDistance } from '../lib/pathfinding'

// ═══════════════════════════════════════════════════════════════════════
// MODE SIMULASI — TIDAK ada koneksi ke robot Husky A200 riil.
//
// Hook ini adalah "kembaran digital" murni di browser: pose, baterai, dan
// gerakan robot semuanya diperagakan lewat interpolasi lokal, BUKAN data
// sensor. Dipakai malam ini karena belum ada akses ke robot fisik.
//
// Yang HARUS diganti saat sudah terhubung ke robot riil (lihat juga
// useRosConnection.ts untuk pola langganan topic yang sudah ada):
//   1. Pose robot     -> subscribe /amcl_pose (sudah ada di useRosConnection),
//                        JANGAN diinterpolasi lokal lagi.
//   2. Baterai        -> subscribe /a200_1060/platform/bms/state (sudah ada),
//                        dan pakai field `current`/kapasitas Wh riil BMS utk
//                        estimasi waktu tersisa (bukan slope % lokal).
//   3. Navigasi ke titik objek -> kirim action goal Nav2 `NavigateToPose`
//                        (lewat rosbridge actionlib, topic action
//                        /navigate_to_pose) memakai (x,y) target dari
//                        semantic_map.yaml — BUKAN animasi lokal + A* di
//                        browser. Progress ambil dari feedback action,
//                        bukan dihitung dari jarak tempuh lokal.
//   4. Path yang digambar di peta -> ambil dari topic Nav2 plan
//                        (/plan atau feedback action), bukan hasil A* lokal.
//   5. Obstacle (person/robot lab) -> subscribe /semantic/person_tracks +
//                        /semantic/lab_robot_tracks (sudah ada di
//                        useRosConnection, lihat person_detector.py /
//                        lab_robot_detector.py) — JSON [{track_id,x,y,
//                        speed,moving,(label)}], sudah di frame `map`,
//                        BUKAN diorbit-orbit sintetis spt di bawah ini.
// ═══════════════════════════════════════════════════════════════════════

export type NavStatus = 'idle' | 'planning' | 'moving' | 'arrived' | 'failed'

export type ObstacleKind = 'person' | 'robot'

export interface SimObstacle {
  id: string
  label: string
  kind: ObstacleKind
  x: number
  y: number
  speed: number
  moving: boolean
}

// Simulasi dua obstacle dinamis mengorbit di sekitar posisi robot SAAT ITU
// (bukan titik map tetap) — supaya selalu kelihatan di robot-viewer (radius
// dekat) berapapun posisi robot di peta. Ini pengganti sementara utk
// /semantic/person_tracks + /semantic/lab_robot_tracks (lihat
// person_detector.py/lab_robot_detector.py) sampai terhubung ke robot riil —
// pola sama dgn pose/baterai di atas.
const OBSTACLE_DEFS: {
  id: string
  label: string
  kind: ObstacleKind
  radius: number
  angularSpeed: number // rad/s, tanda menentukan arah orbit
  phase: number
}[] = [
  { id: 'person_1', label: 'person', kind: 'person', radius: 0.75, angularSpeed: 0.5, phase: 0 },
  { id: 'robot_1', label: 'o-seater', kind: 'robot', radius: 1.0, angularSpeed: 0.12, phase: 2.4 },
]
const OBSTACLE_MOVING_THRESHOLD_MPS = 0.15

export interface SimPose {
  x: number
  y: number
  z: number
  yaw: number
}

export interface SimBattery {
  percentage: number // 0..1, sama unit dgn BatteryState.percentage ROS
  voltage: number
  current: number // Ampere, negatif = discharge
  history: { t: number; pct: number }[] // t = ms epoch
}

export interface NavState {
  status: NavStatus
  targetId: string | null
  targetName: string | null
  path: { x: number; y: number }[]
  traveledDistance: number
  totalDistance: number
  error: string | null
}

const SPEED_MPS = 0.55 // kira-kira max_vel_x Husky A200 di nav2_params.yaml (dikonservatifkan utk demo)
const IDLE_DRAIN_PCT_PER_SEC = 3 / 100 / 60 // ~3%/menit diam (dipercepat drastis dari real robot demi demo)
const MOVE_DRAIN_PCT_PER_SEC = 9 / 100 / 60 // ~9%/menit bergerak
const HISTORY_INTERVAL_MS = 4000
const HISTORY_MAX_SAMPLES = 120
// React state (dan seluruh re-render pohon komponen di bawahnya) di-flush
// sekitar 5x/detik, BUKAN tiap frame rAF (60x/detik) — batteryPctRef sendiri
// tetap dihitung tiap frame (halus), cuma publish ke React di-throttle
// supaya <Canvas> three.js gak ikut re-render 60x/detik tanpa perlu (drain
// idle ga butuh presisi sehalus itu secara visual).
const BATTERY_FLUSH_INTERVAL_MS = 200

function computeVoltage(pct: number): number {
  // Model linear kasar 24V nominal (2x12V seri, khas Husky A200 BMS) —
  // BUKAN kurva discharge riil (yg biasanya non-linear / ada "knee").
  return 20 + 4 * Math.max(0, Math.min(1, pct))
}

export function useSimEngine(mapData: { meta: MapMeta; objects: SemanticObject[]; grid: WalkableGrid } | null) {
  const [pose, setPose] = useState<SimPose>({ x: 0, y: 6, z: 0, yaw: 0 })
  const [speed, setSpeed] = useState(0)
  const [battery, setBattery] = useState<SimBattery>(() => ({
    percentage: 0.87,
    voltage: computeVoltage(0.87),
    current: -1.2,
    history: [{ t: Date.now(), pct: 87 }],
  }))
  const [nav, setNav] = useState<NavState>({
    status: 'idle',
    targetId: null,
    targetName: null,
    path: [],
    traveledDistance: 0,
    totalDistance: 0,
    error: null,
  })
  const [obstacles, setObstacles] = useState<SimObstacle[]>([])

  const poseRef = useRef(pose)
  const navRef = useRef(nav)
  const speedRef = useRef(0)
  const rafRef = useRef<number | null>(null)
  const lastTsRef = useRef<number | null>(null)
  const lastHistoryPushRef = useRef(0)
  const lastBatteryFlushRef = useRef(0)
  const lastObstacleFlushRef = useRef(0)
  const batteryPctRef = useRef(0.87)

  poseRef.current = pose
  navRef.current = nav

  // Snap posisi awal ke sel walkable terdekat begitu grid data siap, supaya
  // robot tidak "start" di dalam tembok kalau tebakan koordinat default meleset.
  const snappedRef = useRef(false)
  useEffect(() => {
    if (!mapData || snappedRef.current) return
    const cell = worldToGridCell(mapData.meta, mapData.grid, poseRef.current.x, poseRef.current.y)
    const walkable = nearestWalkable(mapData.grid, cell.gx, cell.gy)
    if (walkable) {
      snappedRef.current = true
      const world = gridCellToWorld(mapData.meta, mapData.grid, walkable.gx, walkable.gy)
      setPose((p) => ({ ...p, x: world.x, y: world.y }))
    }
  }, [mapData])

  useEffect(() => {
    function tick(ts: number) {
      const last = lastTsRef.current
      lastTsRef.current = ts
      const dt = last == null ? 0 : Math.min(0.25, (ts - last) / 1000)

      const currentNav = navRef.current
      const isMoving = currentNav.status === 'moving'
      const nextSpeed = isMoving ? SPEED_MPS : 0
      if (speedRef.current !== nextSpeed) {
        speedRef.current = nextSpeed
        setSpeed(nextSpeed)
      }

      if (isMoving && dt > 0) {
        const traveled = currentNav.traveledDistance + SPEED_MPS * dt
        const { point, heading } = pointAtDistance(currentNav.path, traveled)
        if (traveled >= currentNav.totalDistance) {
          const finalPoint = currentNav.path[currentNav.path.length - 1] ?? point
          setPose({ x: finalPoint.x, y: finalPoint.y, z: 0, yaw: heading })
          setNav((n) => ({ ...n, status: 'arrived', traveledDistance: n.totalDistance }))
        } else {
          setPose({ x: point.x, y: point.y, z: 0, yaw: heading })
          setNav((n) => ({ ...n, traveledDistance: traveled }))
        }
      }

      {
        const now = Date.now()
        if (now - lastObstacleFlushRef.current > BATTERY_FLUSH_INTERVAL_MS) {
          lastObstacleFlushRef.current = now
          const t = ts / 1000
          const { x: rx, y: ry } = poseRef.current
          setObstacles(
            OBSTACLE_DEFS.map((def) => {
              const angle = def.phase + t * def.angularSpeed
              const linearSpeed = Math.abs(def.angularSpeed) * def.radius
              return {
                id: def.id,
                label: def.label,
                kind: def.kind,
                x: rx + def.radius * Math.cos(angle),
                y: ry + def.radius * Math.sin(angle),
                speed: linearSpeed,
                moving: linearSpeed >= OBSTACLE_MOVING_THRESHOLD_MPS,
              }
            }),
          )
        }
      }

      if (dt > 0) {
        const drainRate = isMoving ? MOVE_DRAIN_PCT_PER_SEC : IDLE_DRAIN_PCT_PER_SEC
        batteryPctRef.current = Math.max(0, batteryPctRef.current - drainRate * dt)

        const now = Date.now()
        if (now - lastBatteryFlushRef.current > BATTERY_FLUSH_INTERVAL_MS) {
          lastBatteryFlushRef.current = now
          const pct = batteryPctRef.current
          const current = isMoving ? -3.4 : -1.2
          setBattery((b) => {
            let history = b.history
            if (now - lastHistoryPushRef.current > HISTORY_INTERVAL_MS) {
              lastHistoryPushRef.current = now
              history = [...history, { t: now, pct: pct * 100 }].slice(-HISTORY_MAX_SAMPLES)
            }
            return { percentage: pct, voltage: computeVoltage(pct), current, history }
          })
        }
      }

      rafRef.current = requestAnimationFrame(tick)
    }

    rafRef.current = requestAnimationFrame(tick)
    return () => {
      if (rafRef.current != null) cancelAnimationFrame(rafRef.current)
    }
  }, [])

  const navigateTo = useCallback(
    (objectId: string) => {
      if (!mapData) return
      const target = mapData.objects.find((o) => o.id === objectId)
      if (!target) return

      setNav({
        status: 'planning',
        targetId: objectId,
        targetName: target.name,
        path: [],
        traveledDistance: 0,
        totalDistance: 0,
        error: null,
      })

      // findPath cukup cepat (grid ~264x158) utk dipanggil sinkron, tapi
      // status 'planning' tetap di-set duluan supaya UI kasih feedback
      // instan kalau nanti diganti pemanggilan async (mis. request Nav2).
      const path = findPath(mapData.meta, mapData.grid, poseRef.current, { x: target.x, y: target.y })
      if (!path || path.length < 2) {
        setNav((n) => ({ ...n, status: 'failed', error: 'Tidak ditemukan jalur bebas obstacle ke objek ini.' }))
        return
      }
      const total = pathLength(path)
      setNav({
        status: 'moving',
        targetId: objectId,
        targetName: target.name,
        path,
        traveledDistance: 0,
        totalDistance: total,
        error: null,
      })
    },
    [mapData],
  )

  const cancelNavigation = useCallback(() => {
    setNav((n) => ({ ...n, status: 'idle', path: [], targetId: null, targetName: null }))
  }, [])

  const distanceRemaining = Math.max(0, nav.totalDistance - nav.traveledDistance)
  const etaSeconds = speed > 0 ? distanceRemaining / SPEED_MPS : null

  return { pose, speed, battery, nav, obstacles, distanceRemaining, etaSeconds, navigateTo, cancelNavigation }
}
