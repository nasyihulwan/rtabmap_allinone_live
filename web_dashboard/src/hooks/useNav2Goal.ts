import { useCallback, useEffect, useRef, useState } from 'react'
import * as ROSLIB from 'roslib'
import type { MapMeta, SemanticObject, WalkableGrid } from '../lib/mapData'
import { gridCellToWorld, worldToGridCell } from '../lib/mapData'
import { nearestWalkable, pathLength } from '../lib/pathfinding'
import type { NavStatus, NavState } from './useSimEngine'

// Pengganti useSimEngine's fake pathfinder: goal dikirim sebagai action
// NavigateToPose asli via rosbridge (ROSLIB.Action, dukungan native ROS2
// action di roslib >=2.x) ke stack Nav2 yang benar-benar jalan (real robot
// atau ./run.sh sim) — path yang digambar juga dari topic /plan asli
// (SmacPlanner2D di nav2_params.yaml), bukan A* lokal di browser.

const INITIAL_NAV: NavState = {
  status: 'idle',
  targetId: null,
  targetName: null,
  path: [],
  traveledDistance: 0,
  totalDistance: 0,
  error: null,
}

function yawToQuaternion(yaw: number) {
  return { x: 0, y: 0, z: Math.sin(yaw / 2), w: Math.cos(yaw / 2) }
}

export function useNav2Goal(
  ros: ROSLIB.Ros | null,
  objects: SemanticObject[],
  speed: number,
  mapGrid: { meta: MapMeta; grid: WalkableGrid } | null,
) {
  const [nav, setNav] = useState<NavState>(INITIAL_NAV)
  const actionRef = useRef<ROSLIB.Action | null>(null)
  const goalIdRef = useRef<string | null>(null)
  const totalDistanceRef = useRef(0)

  // /plan: path GLOBAL asli dari planner_server (SmacPlanner2D) — update tiap
  // kali bt_navigator replan (lihat memory path_preview_test: replan 1Hz).
  useEffect(() => {
    if (!ros) return
    const planTopic = new ROSLIB.Topic({
      ros,
      name: '/plan',
      messageType: 'nav_msgs/msg/Path',
    })
    const handler = (msg: unknown) => {
      const poses = (msg as { poses: { pose: { position: { x: number; y: number } } }[] }).poses
      const path = poses.map((p) => ({ x: p.pose.position.x, y: p.pose.position.y }))
      totalDistanceRef.current = pathLength(path)
      setNav((n) => (n.status === 'idle' ? n : { ...n, path, totalDistance: totalDistanceRef.current }))
    }
    planTopic.subscribe(handler)
    return () => planTopic.unsubscribe()
  }, [ros])

  const navigateTo = useCallback(
    (objectId: string) => {
      if (!ros) {
        setNav({ ...INITIAL_NAV, status: 'failed', error: 'rosbridge tidak terhubung.' })
        return
      }
      const target = objects.find((o) => o.id === objectId)
      if (!target) return

      // Posisi objek semantic itu sendiri SELALU jatuh di sel occupied (objek
      // = obstacle fisik, mis. shelf/tembok) — kirim itu langsung sbg goal
      // Nav2 bikin planner/controller ABORT (error_code 208, dikonfirmasi
      // 2026-07-14: object dot di peta ada TEPAT di piksel shelf). Snap ke
      // sel walkable terdekat dulu, pola sama yg sudah dipakai lib/
      // pathfinding.ts (nearestWalkable) buat A* lokal sebelumnya.
      let goalX = target.x
      let goalY = target.y
      if (mapGrid) {
        const cell = worldToGridCell(mapGrid.meta, mapGrid.grid, target.x, target.y)
        const walkable = nearestWalkable(mapGrid.grid, cell.gx, cell.gy)
        if (walkable) {
          const world = gridCellToWorld(mapGrid.meta, mapGrid.grid, walkable.gx, walkable.gy)
          goalX = world.x
          goalY = world.y
        }
      }

      const action = new ROSLIB.Action({
        ros,
        name: '/navigate_to_pose',
        actionType: 'nav2_msgs/action/NavigateToPose',
      })
      actionRef.current = action
      totalDistanceRef.current = 0

      setNav({
        status: 'planning',
        targetId: objectId,
        targetName: target.name,
        path: [],
        traveledDistance: 0,
        totalDistance: 0,
        error: null,
      })

      const goalMessage = {
        pose: {
          header: { frame_id: 'map' },
          pose: {
            position: { x: goalX, y: goalY, z: 0 },
            orientation: yawToQuaternion(0),
          },
        },
      }

      const id = action.sendGoal(
        goalMessage,
        () => {
          setNav((n) => ({ ...n, status: 'arrived' as NavStatus, traveledDistance: n.totalDistance }))
        },
        (feedback: unknown) => {
          const distanceRemaining = (feedback as { distance_remaining: number }).distance_remaining
          const traveled = Math.max(0, totalDistanceRef.current - distanceRemaining)
          setNav((n) => ({ ...n, status: 'moving' as NavStatus, traveledDistance: traveled }))
        },
        (error: string) => {
          setNav((n) => ({ ...n, status: 'failed' as NavStatus, error }))
        },
      )
      goalIdRef.current = id ?? null
    },
    [ros, objects, mapGrid],
  )

  const cancelNavigation = useCallback(() => {
    if (actionRef.current && goalIdRef.current) {
      actionRef.current.cancelGoal(goalIdRef.current)
    }
    setNav({ ...INITIAL_NAV })
  }, [])

  const distanceRemaining = Math.max(0, nav.totalDistance - nav.traveledDistance)
  const etaSeconds = speed > 0.01 && nav.status === 'moving' ? distanceRemaining / speed : null

  return { nav, distanceRemaining, etaSeconds, navigateTo, cancelNavigation }
}
