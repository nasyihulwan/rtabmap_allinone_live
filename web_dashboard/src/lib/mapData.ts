import { useEffect, useState } from 'react'

// Semua asset di /public/map/ adalah data DEMO (lihat scripts/prepare_map_assets.mjs)
// dibangkitkan dari folder sementara/ (lab_map.pgm/yaml, semantic_map.yaml) +
// output/scene_zone_test (scene_zone_final_v9). Saat robot riil terhubung,
// jalankan ulang script itu dengan sumber data live map_server/RTABMap, TIDAK
// perlu ubah kode di bawah ini — bentuk JSON-nya tetap sama.

export interface MapMeta {
  demo: boolean
  note: string
  width: number
  height: number
  resolution: number
  origin: [number, number, number]
  occupiedThresh: number
  freeThresh: number
  zoneLabelMap: Record<string, string>
}

export interface SemanticObject {
  id: string
  name: string
  cell_count: number
  class_id: number
  x: number
  y: number
  zone: string
}

export interface WalkableGrid {
  width: number
  height: number
  downsample: number
  resolution: number
  origin: [number, number, number]
  fullHeightPx: number
  bits: Uint8Array // packed, test via isWalkable()
}

function decodeBase64ToBytes(b64: string): Uint8Array {
  const bin = atob(b64)
  const out = new Uint8Array(bin.length)
  for (let i = 0; i < bin.length; i++) out[i] = bin.charCodeAt(i)
  return out
}

export function isWalkableCell(grid: WalkableGrid, gx: number, gy: number): boolean {
  if (gx < 0 || gx >= grid.width || gy < 0 || gy >= grid.height) return false
  const idx = gy * grid.width + gx
  return (grid.bits[idx >> 3] & (1 << (idx & 7))) !== 0
}

/** Konversi koordinat dunia (meter, frame "map") -> piksel gambar full-res
 * (lab_map.png / scene_zones.png), row 0 = atas gambar — sama persis
 * konvensi map_server yang dipakai semantic/scene_zone_exporter.py. */
export function worldToPixel(meta: MapMeta, wx: number, wy: number): { row: number; col: number } {
  const col = Math.floor((wx - meta.origin[0]) / meta.resolution)
  const rowFromBottom = Math.floor((wy - meta.origin[1]) / meta.resolution)
  const row = meta.height - 1 - rowFromBottom
  return { row, col }
}

export function pixelToWorld(meta: MapMeta, row: number, col: number): { x: number; y: number } {
  const rowFromBottom = meta.height - 1 - row
  return {
    x: meta.origin[0] + (col + 0.5) * meta.resolution,
    y: meta.origin[1] + (rowFromBottom + 0.5) * meta.resolution,
  }
}

export function worldToGridCell(meta: MapMeta, grid: WalkableGrid, wx: number, wy: number) {
  const { row, col } = worldToPixel(meta, wx, wy)
  return { gx: Math.floor(col / grid.downsample), gy: Math.floor(row / grid.downsample) }
}

export function gridCellToWorld(meta: MapMeta, grid: WalkableGrid, gx: number, gy: number) {
  const row = gy * grid.downsample + grid.downsample / 2
  const col = gx * grid.downsample + grid.downsample / 2
  return pixelToWorld(meta, row, col)
}

interface MapDataBundle {
  meta: MapMeta
  objects: SemanticObject[]
  grid: WalkableGrid
}

let cached: MapDataBundle | null = null
let inflight: Promise<MapDataBundle> | null = null

async function loadBundle(): Promise<MapDataBundle> {
  const [meta, objects, walkableRaw] = await Promise.all([
    fetch('/map/map_meta.json').then((r) => r.json() as Promise<MapMeta>),
    fetch('/map/objects.json').then((r) => r.json() as Promise<SemanticObject[]>),
    fetch('/map/walkable.json').then((r) => r.json()),
  ])
  const grid: WalkableGrid = {
    width: walkableRaw.width,
    height: walkableRaw.height,
    downsample: walkableRaw.downsample,
    resolution: walkableRaw.resolution,
    origin: walkableRaw.origin,
    fullHeightPx: walkableRaw.fullHeightPx,
    bits: decodeBase64ToBytes(walkableRaw.packedBase64),
  }
  return { meta, objects, grid }
}

/** Hook: load & cache data map/objek/walkable-grid sekali untuk seluruh app. */
export function useMapData() {
  const [data, setData] = useState<MapDataBundle | null>(cached)
  const [error, setError] = useState<Error | null>(null)

  useEffect(() => {
    if (cached) return
    if (!inflight) inflight = loadBundle()
    inflight
      .then((bundle) => {
        cached = bundle
        setData(bundle)
      })
      .catch((err) => setError(err instanceof Error ? err : new Error(String(err))))
  }, [])

  return { data, error }
}

// ── Mesh 3D (dari sementara/rtabmap.db, lihat scripts/prepare_mesh.py) ──
// Permukaan solid (triangulasi), BUKAN point cloud sparse — voxel point
// cloud (percobaan pertama, lihat scripts/prepare_pointcloud.py, masih
// disimpan sbg alternatif lebih ringan/cepat-generate) kelihatan
// "dikit-dikit"/berlubang krn spacing antar titik > ukuran titik yg wajar
// dirender. Mesh ngisi celah itu jadi permukaan kontinu.
export interface Mesh3D {
  vertexCount: number
  faceCount: number
  positions: Float32Array // frame 'map' ROS (x,y,z, z=atas), BUKAN Three.js
  colors: Uint8Array
  indices: Uint32Array
}

let meshCached: Mesh3D | null = null
let meshInflight: Promise<Mesh3D> | null = null

async function loadMesh(): Promise<Mesh3D> {
  const buf = await fetch('/map/mesh.bin').then((r) => {
    if (!r.ok) throw new Error(`mesh.bin ${r.status}`)
    return r.arrayBuffer()
  })
  const view = new DataView(buf)
  const vertexCount = view.getUint32(0, true)
  const faceCount = view.getUint32(4, true)
  const positions = new Float32Array(buf, 8, vertexCount * 3)
  const colorsOffset = 8 + vertexCount * 3 * 4
  const colors = new Uint8Array(buf, colorsOffset, vertexCount * 3)
  // Uint32Array butuh byte-offset kelipatan 4 — offset indices (setelah
  // header+positions+colors) gak dijamin kelipatan 4 krn colors 3 byte/vertex
  // (RGB uint8). .slice() bikin ArrayBuffer baru (mulai dari 0, otomatis
  // aligned) — copy ~2-3MB sekali pas load, bukan overhead berulang.
  const indicesOffset = colorsOffset + vertexCount * 3
  const indicesBytes = buf.slice(indicesOffset, indicesOffset + faceCount * 3 * 4)
  const indices = new Uint32Array(indicesBytes)
  return { vertexCount, faceCount, positions, colors, indices }
}

/** Hook: load & cache mesh 3D sekali untuk seluruh app (file ~4-5MB, jangan
 * fetch ulang tiap kali komponen viewer 3D di-mount/unmount). */
export function useMesh() {
  const [data, setData] = useState<Mesh3D | null>(meshCached)
  const [error, setError] = useState<Error | null>(null)

  useEffect(() => {
    if (meshCached) return
    if (!meshInflight) meshInflight = loadMesh()
    meshInflight
      .then((mesh) => {
        meshCached = mesh
        setData(mesh)
      })
      .catch((err) => setError(err instanceof Error ? err : new Error(String(err))))
  }, [])

  return { data, error }
}
