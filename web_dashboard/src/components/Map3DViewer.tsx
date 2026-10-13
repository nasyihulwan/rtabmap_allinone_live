import { memo, useMemo } from 'react'
import { Canvas } from '@react-three/fiber'
import { OrbitControls } from '@react-three/drei'
import * as THREE from 'three'
import { rosToThree } from '../lib/rosToThree'
import type { SemanticObject, Mesh3D } from '../lib/mapData'
import type { SimPose } from '../hooks/useSimEngine'

// Peta lab BRIN (sementara/rtabmap.db) rentang puluhan meter (x: -18..22,
// y: -4..20) — jauh lebih besar dari skala RobotViewer.tsx (closeup ~1-2m),
// jadi kamera/kontrol di sini pakai jarak & kecepatan berbeda.
const CAMERA_PROPS = { position: [4, 14, 4] as [number, number, number], fov: 50 }
const ORBIT_TARGET: [number, number, number] = [0, 0, 0]

function ReconstructedMesh({ mesh }: { mesh: Mesh3D }) {
  const geometry = useMemo(() => {
    const geo = new THREE.BufferGeometry()
    const n = mesh.vertexCount
    const positions = new Float32Array(n * 3)
    const colors = new Float32Array(n * 3)
    for (let i = 0; i < n; i++) {
      const rx = mesh.positions[i * 3]
      const ry = mesh.positions[i * 3 + 1]
      const rz = mesh.positions[i * 3 + 2]
      const [tx, ty, tz] = rosToThree([rx, ry, rz])
      positions[i * 3] = tx
      positions[i * 3 + 1] = ty
      positions[i * 3 + 2] = tz
      colors[i * 3] = mesh.colors[i * 3] / 255
      colors[i * 3 + 1] = mesh.colors[i * 3 + 1] / 255
      colors[i * 3 + 2] = mesh.colors[i * 3 + 2] / 255
    }
    geo.setAttribute('position', new THREE.BufferAttribute(positions, 3))
    geo.setAttribute('color', new THREE.BufferAttribute(colors, 3))
    // rosToThree membalik chirality (x,z,-y) — winding order segitiga jadi
    // kebalik (muka "belakang" menghadap kamera). setIndex tetap pakai
    // urutan asli, cukup balik normal via flipY-equivalent: pakai
    // side:DoubleSide di material drpd itung ulang winding manual (lebih
    // murah komputasi drpd re-order index array 188rb segitiga di JS).
    geo.setIndex(new THREE.BufferAttribute(mesh.indices, 1))
    geo.computeVertexNormals()
    return geo
  }, [mesh])

  return (
    <mesh geometry={geometry}>
      <meshStandardMaterial vertexColors side={THREE.DoubleSide} roughness={0.9} metalness={0} />
    </mesh>
  )
}

function RobotArrow({ pose }: { pose: SimPose }) {
  const [tx, ty, tz] = rosToThree([pose.x, pose.y, 0.15])
  return (
    <group position={[tx, ty, tz]} rotation={[0, -pose.yaw, 0]}>
      <mesh rotation={[0, 0, -Math.PI / 2]}>
        <coneGeometry args={[0.25, 0.6, 12]} />
        <meshStandardMaterial color="#0a84ff" />
      </mesh>
      <pointLight color="#0a84ff" intensity={2} distance={2} />
    </group>
  )
}

const OBJECT_COLOR = '#ff9f43'

function ObjectMarkers({ objects }: { objects: SemanticObject[] }) {
  return (
    <>
      {objects.map((o) => {
        const [tx, ty, tz] = rosToThree([o.x, o.y, 0.1])
        return (
          <mesh key={o.id} position={[tx, ty, tz]}>
            <sphereGeometry args={[0.08, 8, 8]} />
            <meshStandardMaterial color={OBJECT_COLOR} />
          </mesh>
        )
      })}
    </>
  )
}

export const Map3DViewer = memo(function Map3DViewer({
  mesh,
  objects,
  pose,
  showObjects = true,
}: {
  mesh: Mesh3D | null
  objects: SemanticObject[]
  pose: SimPose
  showObjects?: boolean
}) {
  return (
    <div className="pointcloud-viewer">
      {/* antialias:true + dpr eksplisit — tanpa ini WebGLRenderer default
          antialias:false, DAN r3f kadang gak otomatis sync resolusi kanvas
          internal ke devicePixelRatio layar (terutama layar HiDPI/retina) —
          hasilnya kanvas di-render resolusi rendah lalu di-stretch CSS jadi
          keliatan blur/pecah. dpr={[1,2]} clamp ke devicePixelRatio asli,
          maksimum 2x (cukup tajam, gak boros GPU di layar >2x). */}
      <Canvas camera={CAMERA_PROPS} gl={{ alpha: true, antialias: true }} dpr={[1, 2]}>
        <ambientLight intensity={0.8} />
        <directionalLight position={[10, 20, 10]} intensity={0.9} />
        <directionalLight position={[-10, 10, -10]} intensity={0.3} />
        {mesh && <ReconstructedMesh mesh={mesh} />}
        <RobotArrow pose={pose} />
        {showObjects && <ObjectMarkers objects={objects} />}
        <OrbitControls
          target={ORBIT_TARGET}
          minDistance={1}
          maxDistance={45}
          maxPolarAngle={Math.PI / 2 - 0.02}
        />
      </Canvas>
    </div>
  )
})
