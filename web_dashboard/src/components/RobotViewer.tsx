import { memo, Suspense, useMemo, useRef } from 'react'
import { Canvas, useFrame, useLoader } from '@react-three/fiber'
import { OrbitControls, ContactShadows, Environment } from '@react-three/drei'
import { ColladaLoader } from 'three/examples/jsm/loaders/ColladaLoader.js'
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js'
import type { Group } from 'three'
import type { Pose2D } from '../hooks/useRosConnection'
import { rosToThree } from '../lib/rosToThree'

// Mesh asli Husky A200 (clearpath_platform_description), copy dari kmp-orin.
// Velodyne & camera sengaja tidak disertakan di visual.
const MESH_BASE = '/meshes/husky_a200'

// Dimensi dari clearpath_platform_description/urdf/a200/a200.urdf.xacro
// (posisi joint di sini pakai konvensi ROS: x=depan, y=kiri, z=atas)
const WHEELBASE = 0.512
const TRACK = 0.555
const WHEEL_Z = 0.03282
const WHEEL_RADIUS = 0.1651 // clearpath_platform_description a200 wheel radius

function ColladaMesh({ url }: { url: string }) {
  const collada = useLoader(ColladaLoader, url)
  // useLoader cache balikin object Three.js YANG SAMA tiap dipakai berulang
  // (mis. wheel.dae dipakai 4x) — tanpa clone, Three.js cuma bisa taruh 1
  // object di 1 parent sekaligus, jadi instance lain "kecolong" ke posisi
  // terakhir yang di-render.
  const scene = useMemo(() => collada!.scene.clone(true), [collada])
  return <primitive object={scene} />
}

// STLLoader (beda dari ColladaLoader) TIDAK auto-convert Z-up ke Y-up,
// jadi geometrinya perlu dirotasi manual -90deg di sumbu X.
function StlMesh({ url, color }: { url: string; color: string }) {
  const geometry = useLoader(STLLoader, url)
  return (
    <mesh geometry={geometry} rotation={[-Math.PI / 2, 0, 0]} castShadow>
      <meshStandardMaterial color={color} metalness={0.3} roughness={0.6} />
    </mesh>
  )
}

function SpinningWheel({ position, speed }: { position: [number, number, number]; speed: number }) {
  // Roda berputar sesuai kecepatan sim/ROS (rad/s = speed / radius) — cuma
  // efek visual "robot sedang bergerak", bukan odometri nyata.
  const ref = useRef<Group>(null)
  useFrame((_, delta) => {
    if (ref.current) ref.current.rotation.z += (speed / WHEEL_RADIUS) * delta
  })
  return (
    <group position={position}>
      <group ref={ref}>
        <ColladaMesh url={`${MESH_BASE}/wheel.dae`} />
      </group>
    </group>
  )
}

// Garis "kecepatan" ala Tesla Autopilot viz: goresan tipis di kedua sisi
// robot yang mengalir ke belakang selama bergerak. Ditaruh SEJAJAR (bukan
// child) bodyRef supaya tetap di ketinggian tanah, tidak ikut bob/lean
// suspensi chassis. Posisi X = ROS x (sumbu depan-belakang) dianimasikan
// langsung tiap frame — di konvensi rosToThree, three.x = ROS x 1:1, jadi
// tidak perlu re-konversi tiap frame, cukup mutasi position.x.
const LANES_ROS_Y = [TRACK / 2 + 0.16, -(TRACK / 2 + 0.16)]
const STREAKS_PER_LANE = 4
const STREAK_SPACING = 0.24
const STREAK_CYCLE = STREAKS_PER_LANE * STREAK_SPACING
const STREAK_SPEED_MULT = 2.2

function SpeedLines({ speed }: { speed: number }) {
  const refs = useRef<(Group | null)[]>([])
  const visible = speed > 0.02

  useFrame((_, delta) => {
    if (!visible) return
    for (const g of refs.current) {
      if (!g) continue
      g.position.x -= speed * STREAK_SPEED_MULT * delta
      if (g.position.x < -STREAK_CYCLE) g.position.x += STREAK_CYCLE
    }
  })

  let idx = -1
  return (
    <group visible={visible}>
      {LANES_ROS_Y.map((laneY) =>
        Array.from({ length: STREAKS_PER_LANE }).map((_, i) => {
          idx++
          const myIdx = idx
          const startRosX = -0.5 - i * STREAK_SPACING
          const [tx, ty, tz] = rosToThree([startRosX, laneY, WHEEL_Z])
          return (
            <group
              key={`${laneY}-${i}`}
              position={[tx, ty, tz]}
              ref={(el) => {
                refs.current[myIdx] = el
              }}
            >
              <mesh rotation={[-Math.PI / 2, 0, 0]}>
                <planeGeometry args={[0.16, 0.02]} />
                <meshBasicMaterial color="#4fb8ff" transparent opacity={0.55} />
              </mesh>
            </group>
          )
        }),
      )}
    </group>
  )
}

export interface ViewerObstacle {
  id: string
  label: string
  kind: 'person' | 'robot'
  x: number
  y: number
  speed: number
  moving: boolean
}

// Marker animasi 1 obstacle, diposisikan RELATIF ke robot (offset dunia
// nyata, BUKAN dikontra-rotasi oleh yaw robot) — supaya arah "timur dari
// robot" tetap kelihatan konsisten di viewer walau robot lagi muter di
// tempat. Ditaruh sbg SIBLING dari HuskyModel (bukan child), krn HuskyModel
// sendiri yg dirotasi yaw utk menunjukkan heading-nya.
function ObstacleMarker({ obstacle, robotPose }: { obstacle: ViewerObstacle; robotPose: Pose2D }) {
  const ringRef = useRef<Group>(null)
  const bodyRef = useRef<Group>(null)
  const phaseRef = useRef(Math.random() * Math.PI * 2)

  const dx = obstacle.x - robotPose.x
  const dy = obstacle.y - robotPose.y
  const [tx, ty, tz] = rosToThree([dx, dy, 0])

  const color = obstacle.kind === 'person' ? (obstacle.moving ? '#ff9f43' : '#c9ccd1') : '#4fb8ff'

  useFrame((state, delta) => {
    if (obstacle.moving) {
      phaseRef.current += delta * 4
      if (bodyRef.current) {
        // goyang kecil ala "berjalan/bergerak" — gak akurat scr fisik, cuma
        // sinyal visual "ini bergerak" biar kebedaan sama yg diam.
        bodyRef.current.position.y = ty + Math.abs(Math.sin(phaseRef.current)) * 0.03
      }
      if (ringRef.current) {
        const pulse = (state.clock.elapsedTime * 1.2) % 1
        ringRef.current.scale.setScalar(0.5 + pulse * 0.9)
        const mat = (ringRef.current.children[0] as any)?.material
        if (mat) mat.opacity = Math.max(0, 0.6 - pulse * 0.6)
      }
    } else if (bodyRef.current) {
      bodyRef.current.position.y = ty
    }
  })

  return (
    <group position={[tx, 0, tz]}>
      {obstacle.moving && (
        <group ref={ringRef} rotation={[-Math.PI / 2, 0, 0]}>
          <mesh>
            <ringGeometry args={[0.16, 0.2, 24]} />
            <meshBasicMaterial color={color} transparent opacity={0.5} />
          </mesh>
        </group>
      )}
      <group ref={bodyRef} position={[0, ty, 0]}>
        {obstacle.kind === 'person' ? (
          <>
            <mesh position={[0, 0.19, 0]} castShadow>
              <capsuleGeometry args={[0.07, 0.16, 4, 8]} />
              <meshStandardMaterial color={color} />
            </mesh>
            <mesh position={[0, 0.32, 0]} castShadow>
              <sphereGeometry args={[0.055, 12, 12]} />
              <meshStandardMaterial color={color} />
            </mesh>
          </>
        ) : (
          <mesh position={[0, 0.09, 0]} castShadow>
            <cylinderGeometry args={[0.11, 0.13, 0.17, 16]} />
            <meshStandardMaterial color={color} metalness={0.4} roughness={0.4} />
          </mesh>
        )}
      </group>
    </group>
  )
}

function ObstacleField({ obstacles, robotPose }: { obstacles: ViewerObstacle[]; robotPose: Pose2D | null }) {
  if (!robotPose) return null
  return (
    <>
      {obstacles.map((o) => (
        <ObstacleMarker key={o.id} obstacle={o} robotPose={robotPose} />
      ))}
    </>
  )
}

function HuskyModel({ pose, speed }: { pose: Pose2D | null; speed: number }) {
  const yaw = pose?.yaw ?? 0
  const bodyRef = useRef<Group>(null)
  const leanRef = useRef(0)

  // Body chassis: idle bob halus terus-menerus (biar viewer terasa "hidup"
  // walau robot diam), + body lean ke depan saat bergerak (efek akselerasi
  // ala suspensi) — dilerp supaya transisi diam<->gerak halus, bukan snap.
  // PENTING: three.x = ros.x = "depan" robot (lihat rosToThree di atas), jadi
  // rotation.x memutar robot mengelilingi sumbu depannya sendiri — itu ROLL
  // (miring ke samping), bukan PITCH (nunduk ke depan) yang dimaksud. Sumbu
  // lateral (kiri-kanan) robot ada di three.z (= -ros.y), jadi lean ke depan
  // yang benar harus rotation.z, bukan rotation.x.
  useFrame((state, delta) => {
    if (!bodyRef.current) return
    bodyRef.current.position.y = Math.sin(state.clock.elapsedTime * 1.6) * 0.006
    const targetLean = Math.min(speed / 0.55, 1) * -0.045
    leanRef.current += (targetLean - leanRef.current) * Math.min(1, delta * 4)
    bodyRef.current.rotation.z = leanRef.current
  })

  const wheelPositions: [number, number, number][] = [
    rosToThree([WHEELBASE / 2, TRACK / 2, WHEEL_Z]),
    rosToThree([WHEELBASE / 2, -TRACK / 2, WHEEL_Z]),
    rosToThree([-WHEELBASE / 2, TRACK / 2, WHEEL_Z]),
    rosToThree([-WHEELBASE / 2, -TRACK / 2, WHEEL_Z]),
  ]

  return (
    <group rotation={[0, yaw, 0]}>
      <group ref={bodyRef}>
        <ColladaMesh url={`${MESH_BASE}/base_link.dae`} />
        <ColladaMesh url={`${MESH_BASE}/top_chassis.dae`} />
        {wheelPositions.map((pos, i) => (
          <SpinningWheel key={i} position={pos} speed={speed} />
        ))}
        <group position={rosToThree([0.481, 0, 0.091])}>
          <ColladaMesh url={`${MESH_BASE}/bumper.dae`} />
        </group>
        <group position={rosToThree([-0.481, 0, 0.091])} rotation={[0, Math.PI, 0]}>
          <ColladaMesh url={`${MESH_BASE}/bumper.dae`} />
        </group>
        {/* PACS top plate — default_mount (0,0,0.224) + pacs adjust/joint offset (~0.048,0,0) */}
        <group position={rosToThree([0.04825, 0, 0.224])}>
          <StlMesh url={`${MESH_BASE}/pacs_top_plate.stl`} color="#333333" />
        </group>
      </group>
      <SpeedLines speed={speed} />
    </group>
  )
}

function LoadingFallback() {
  return (
    <mesh position={[0, 0.2, 0]}>
      <boxGeometry args={[0.5, 0.2, 0.3]} />
      <meshStandardMaterial color="#333" wireframe />
    </mesh>
  )
}

// Literal array/object props (camera, shadow-mapSize, target, dst) di-hoist
// ke luar komponen supaya identitas referensinya STABIL antar render. Kalau
// dibuat inline di JSX, tiap re-render App (mis. dari tick baterai sim)
// bikin react-three-fiber nganggep prop itu "berubah" (beda referensi) lalu
// re-apply ke object three.js di baliknya — riil kejadian: warning
// "PCFSoftShadowMap deprecated" dari three.js banjir di console tiap re-render.
const CAMERA_PROPS = { position: [1.8, 1.3, 1.8] as [number, number, number], fov: 40 }
const LIGHT_POSITION: [number, number, number] = [3, 5, 2]
const SHADOW_MAP_SIZE: [number, number] = [1024, 1024]
const ORBIT_TARGET: [number, number, number] = [0, 0.2, 0]
const CONTACT_SHADOWS_POSITION: [number, number, number] = [0, 0, 0]
// antialias:true — default WebGLRenderer antialias:false, kelihatan pecah/
// blur di tepi model pas di-stretch CSS ke ukuran panel.
const GL_OPTIONS = { alpha: true, antialias: true }
const CANVAS_DPR: [number, number] = [1, 2]

export const RobotViewer = memo(function RobotViewer({
  pose,
  speed = 0,
  obstacles = [],
}: {
  pose: Pose2D | null
  speed?: number
  obstacles?: ViewerObstacle[]
}) {
  return (
    <div className="robot-viewer">
      {/* shadows="percentage" (bukan shadows boolean) — r3f memetakan boolean
          `true` ke THREE.PCFSoftShadowMap yang sudah deprecated di three.js
          terbaru & console.warn tiap kali renderer di-reconfigure; string
          'percentage' memetakan ke THREE.PCFShadowMap (non-deprecated),
          hasil visual soft-shadow-nya nyaris sama. */}
      {/* gl alpha:true + tanpa <color background> — canvas transparan supaya
          gradient CSS di .robot-pane (App.css) kelihatan di belakang robot,
          bukan putih polos kosong. */}
      <Canvas shadows="percentage" camera={CAMERA_PROPS} gl={GL_OPTIONS} dpr={CANVAS_DPR}>
        <ambientLight intensity={0.5} />
        <directionalLight
          position={LIGHT_POSITION}
          intensity={1.2}
          castShadow
          shadow-mapSize={SHADOW_MAP_SIZE}
        />
        {/* background={false} — tanpa ini drei otomatis pakai HDRI "city" sbg
            background scene (kelihatan sbg pola kotak2 gedung), nutupin
            gradient CSS transparan yang dimaksud. Environment tetap dipakai
            cuma utk pencahayaan/refleksi material, bukan visual background. */}
        <Environment preset="city" environmentIntensity={0.3} background={false} />
        <Suspense fallback={<LoadingFallback />}>
          <HuskyModel pose={pose} speed={speed} />
        </Suspense>
        <ObstacleField obstacles={obstacles} robotPose={pose} />
        <ContactShadows position={CONTACT_SHADOWS_POSITION} opacity={0.5} scale={4} blur={2} far={1} />
        <OrbitControls
          enablePan={false}
          minDistance={1.2}
          maxDistance={5}
          target={ORBIT_TARGET}
          maxPolarAngle={Math.PI / 2 - 0.05}
        />
      </Canvas>
    </div>
  )
})
