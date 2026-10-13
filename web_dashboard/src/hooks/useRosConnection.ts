import { useEffect, useRef, useState, useCallback } from 'react'
import * as ROSLIB from 'roslib'

export type ConnectionStatus = 'disconnected' | 'connecting' | 'connected' | 'error'

export interface Pose2D {
  x: number
  y: number
  z: number
  yaw: number
}

export interface BatteryState {
  percentage: number | null
  voltage: number | null
  current: number | null
  history: { t: number; pct: number }[]
}

export interface RosObstacle {
  id: string
  label: string
  kind: 'person' | 'robot'
  x: number
  y: number
  speed: number
  moving: boolean
}

interface RosData {
  status: ConnectionStatus
  pose: Pose2D | null
  speed: number
  battery: BatteryState
  obstacles: RosObstacle[]
  // Instance ROSLIB.Ros mentah, dipakai hook lain (mis. useNav2Goal) yang
  // butuh action client / topic tambahan lewat koneksi yang SAMA — jangan
  // buka websocket kedua ke rosbridge cuma buat navigasi.
  rosInstance: ROSLIB.Ros | null
}

function quaternionToYaw(z: number, w: number): number {
  return 2 * Math.atan2(z, w)
}

const BATTERY_HISTORY_INTERVAL_MS = 4000
const BATTERY_HISTORY_MAX_SAMPLES = 120

export function useRosConnection(url: string) {
  const [data, setData] = useState<RosData>({
    status: 'disconnected',
    pose: null,
    speed: 0,
    battery: { percentage: null, voltage: null, current: null, history: [] },
    obstacles: [],
    rosInstance: null,
  })
  const rosRef = useRef<ROSLIB.Ros | null>(null)
  const lastPoseSampleRef = useRef<{ x: number; y: number; t: number } | null>(null)
  const lastBatteryHistoryPushRef = useRef(0)
  // person_tracks & lab_robot_tracks dua topic terpisah (lihat
  // person_detector.py / lab_robot_detector.py) — disimpan terpisah lalu
  // digabung tiap kali salah satunya update, supaya update satu topic gak
  // menghapus data topic lainnya yg belum publish ulang.
  const personTracksRef = useRef<RosObstacle[]>([])
  const robotTracksRef = useRef<RosObstacle[]>([])

  const publishObstacles = useCallback(() => {
    setData((d) => ({ ...d, obstacles: [...personTracksRef.current, ...robotTracksRef.current] }))
  }, [])

  const connect = useCallback(() => {
    if (rosRef.current) {
      rosRef.current.close()
    }

    setData((d) => ({ ...d, status: 'connecting' }))

    const ros = new ROSLIB.Ros({ url })
    rosRef.current = ros

    ros.on('connection', () => {
      setData((d) => ({ ...d, status: 'connected', rosInstance: ros }))
    })
    ros.on('error', () => {
      setData((d) => ({ ...d, status: 'error', rosInstance: null }))
    })
    ros.on('close', () => {
      setData((d) => ({ ...d, status: 'disconnected', rosInstance: null }))
    })

    const poseTopic = new ROSLIB.Topic({
      ros,
      name: '/amcl_pose',
      messageType: 'geometry_msgs/msg/PoseWithCovarianceStamped',
    })
    poseTopic.subscribe((msg: any) => {
      const p = msg.pose.pose.position
      const q = msg.pose.pose.orientation
      const now = performance.now()
      const last = lastPoseSampleRef.current
      // /amcl_pose gak selalu publish tiap frame (rate tergantung AMCL), jadi
      // speed didekati dari jarak antar dua pose berturut-turut / delta waktu
      // — bukan dari odom/cmd_vel (belum ada topic itu di sini).
      let speed = 0
      if (last) {
        const dt = (now - last.t) / 1000
        if (dt > 0) {
          speed = Math.hypot(p.x - last.x, p.y - last.y) / dt
        }
      }
      lastPoseSampleRef.current = { x: p.x, y: p.y, t: now }
      setData((d) => ({
        ...d,
        pose: { x: p.x, y: p.y, z: p.z, yaw: quaternionToYaw(q.z, q.w) },
        speed,
      }))
    })

    const batteryTopic = new ROSLIB.Topic({
      ros,
      name: '/a200_1060/platform/bms/state',
      messageType: 'sensor_msgs/msg/BatteryState',
    })
    batteryTopic.subscribe((msg: any) => {
      const now = Date.now()
      setData((d) => {
        let history = d.battery.history
        if (now - lastBatteryHistoryPushRef.current > BATTERY_HISTORY_INTERVAL_MS) {
          lastBatteryHistoryPushRef.current = now
          history = [...history, { t: now, pct: msg.percentage * 100 }].slice(-BATTERY_HISTORY_MAX_SAMPLES)
        }
        return {
          ...d,
          battery: { percentage: msg.percentage, voltage: msg.voltage, current: msg.current, history },
        }
      })
    })

    // person_detector.py: [{track_id,x,y,speed,moving}, ...] frame map, cuma person.
    const personTracksTopic = new ROSLIB.Topic({
      ros,
      name: '/semantic/person_tracks',
      messageType: 'std_msgs/msg/String',
    })
    personTracksTopic.subscribe((msg: any) => {
      try {
        const tracks = JSON.parse(msg.data)
        personTracksRef.current = tracks.map((t: any) => ({
          id: `person_${t.track_id}`,
          label: 'person',
          kind: 'person' as const,
          x: t.x,
          y: t.y,
          speed: t.speed,
          moving: t.moving,
        }))
        publishObstacles()
      } catch {
        // JSON malformed/kosong — abaikan frame ini, tunggu update berikutnya.
      }
    })

    // lab_robot_detector.py: [{track_id,label,x,y,speed,moving}, ...] frame map,
    // dell-e/i-seater/o-seater (robotic-arm sengaja dikecualikan, fixture diam).
    const robotTracksTopic = new ROSLIB.Topic({
      ros,
      name: '/semantic/lab_robot_tracks',
      messageType: 'std_msgs/msg/String',
    })
    robotTracksTopic.subscribe((msg: any) => {
      try {
        const tracks = JSON.parse(msg.data)
        robotTracksRef.current = tracks.map((t: any) => ({
          id: `robot_${t.track_id}`,
          label: t.label,
          kind: 'robot' as const,
          x: t.x,
          y: t.y,
          speed: t.speed,
          moving: t.moving,
        }))
        publishObstacles()
      } catch {
        // JSON malformed/kosong — abaikan frame ini, tunggu update berikutnya.
      }
    })

    return () => {
      poseTopic.unsubscribe()
      batteryTopic.unsubscribe()
      personTracksTopic.unsubscribe()
      robotTracksTopic.unsubscribe()
      ros.close()
    }
  }, [url, publishObstacles])

  useEffect(() => {
    const cleanup = connect()
    return cleanup
  }, [connect])

  return { ...data, reconnect: connect }
}
