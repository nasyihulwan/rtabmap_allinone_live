// Konversi data map "sementara" (dummy/demo) jadi assets statis yang bisa
// di-fetch langsung oleh dashboard: PNG (bukan PGM) + JSON (bukan YAML).
//
// INI DATA DEMO. Saat robot riil sudah terhubung, ganti sumbernya di sini
// (path lab_map.pgm/yaml, semantic_map.yaml, scene_zone_*_final.png/yaml)
// dengan output live dari map_server / semantic_map_exporter.py /
// scene_zone_exporter.py, lalu jalankan ulang: node scripts/prepare_map_assets.mjs
//
// Kenapa pre-build, bukan parse pgm/yaml di browser saat runtime: PGM &
// YAML bukan format native web, dan datanya (untuk sekarang) statis —
// tidak ada gunanya bundle PGM/YAML parser ke bundle produksi.
//
// TODO (belum dikerjakan): sementara/rtabmap.db (249MB) berisi data SLAM
// mentah (pose keyframe di tabel Node + depth/RGB terkompresi) yang bisa
// direkonstruksi jadi point cloud 3D. Belum diekstrak — perlu pipeline
// offline TERPISAH (Python, bukan script Node ini): baca Node+Data via
// sqlite3, decompress depth per keyframe, reproject ke XYZ pakai intrinsics
// kamera, decimate, lalu export .ply kecil buat di-load sebagai layer
// THREE.Points. Untuk sekarang dashboard cuma pakai peta 2D (lab_map.pgm)
// sebagai proxy yang jauh lebih ringan.

import { readFileSync, writeFileSync, mkdirSync, copyFileSync } from 'node:fs'
import { fileURLToPath } from 'node:url'
import path from 'node:path'
import { encode as encodePng } from 'fast-png'

const __dirname = path.dirname(fileURLToPath(import.meta.url))
const ROOT = path.resolve(__dirname, '..', '..') // repo root
const SEMENTARA = path.join(__dirname, '..', 'sementara')
const SCENE_ZONE_DIR = path.join(ROOT, 'output', 'scene_zone_test')
const OUT_DIR = path.join(__dirname, '..', 'public', 'map')

mkdirSync(OUT_DIR, { recursive: true })

// ── 1. lab_map.pgm (P5 binary) -> lab_map.png ──────────────────────────
function parsePgm(buf) {
  // Header P5\n<width> <height>\n<maxval>\n<raw bytes>, komentar '#' diabaikan.
  let offset = 0
  function readToken() {
    while (buf[offset] === 0x23) { // '#' comment line
      while (buf[offset] !== 0x0a) offset++
      offset++
    }
    while (buf[offset] === 0x20 || buf[offset] === 0x0a || buf[offset] === 0x09 || buf[offset] === 0x0d) offset++
    const start = offset
    while (offset < buf.length && !(buf[offset] === 0x20 || buf[offset] === 0x0a || buf[offset] === 0x09 || buf[offset] === 0x0d)) offset++
    return buf.slice(start, offset).toString('ascii')
  }
  const magic = readToken()
  if (magic !== 'P5') throw new Error(`Unsupported PGM magic: ${magic}`)
  const width = parseInt(readToken(), 10)
  const height = parseInt(readToken(), 10)
  const maxval = parseInt(readToken(), 10)
  offset += 1 // single whitespace byte before raster data
  const data = buf.slice(offset, offset + width * height)
  return { width, height, maxval, data }
}

const pgmBuf = readFileSync(path.join(SEMENTARA, 'lab_map.pgm'))
const pgm = parsePgm(pgmBuf)
console.log(`[lab_map.pgm] ${pgm.width}x${pgm.height}, maxval=${pgm.maxval}`)

const mapPng = encodePng({ width: pgm.width, height: pgm.height, data: pgm.data, channels: 1, depth: 8 })
writeFileSync(path.join(OUT_DIR, 'lab_map.png'), mapPng)

// ── 2. lab_map.yaml -> merged into map_meta.json ───────────────────────
function parseSimpleYaml(text) {
  // Parser tangan-buatan, HANYA utk struktur flat key:value / key:[a,b,c]
  // dipakai lab_map.yaml (map_server format) — bukan YAML parser umum.
  const out = {}
  for (const rawLine of text.split('\n')) {
    const line = rawLine.trim()
    if (!line || line.startsWith('#')) continue
    const idx = line.indexOf(':')
    if (idx === -1) continue
    const key = line.slice(0, idx).trim()
    let value = line.slice(idx + 1).trim()
    if (value.startsWith('[') && value.endsWith(']')) {
      out[key] = value.slice(1, -1).split(',').map((v) => Number(v.trim()))
    } else if (!Number.isNaN(Number(value)) && value !== '') {
      out[key] = Number(value)
    } else {
      out[key] = value
    }
  }
  return out
}

const labMapYaml = parseSimpleYaml(readFileSync(path.join(SEMENTARA, 'lab_map.yaml'), 'utf-8'))
console.log('[lab_map.yaml]', labMapYaml)

// ── 3. scene_zone_final_v9: copy colored PNG + parse yaml sidecar ──────
const ZONE_PREFIX = 'scene_zone_final_v9'
copyFileSync(
  path.join(SCENE_ZONE_DIR, `${ZONE_PREFIX}_colored.png`),
  path.join(OUT_DIR, 'scene_zones.png'),
)
copyFileSync(
  path.join(SCENE_ZONE_DIR, `${ZONE_PREFIX}_final.png`),
  path.join(OUT_DIR, 'scene_zones_labels.png'),
)

function parseZoneYaml(text) {
  // Struktur: height/image/origin(list)/resolution/width/label_map(dict int->str)
  const lines = text.split('\n')
  const out = { label_map: {} }
  let inLabelMap = false
  let inOrigin = false
  const origin = []
  for (const rawLine of lines) {
    if (rawLine.startsWith('label_map:')) { inLabelMap = true; continue }
    if (rawLine.startsWith('origin:')) { inOrigin = true; continue }
    if (inLabelMap) {
      const m = rawLine.match(/^\s*(\d+):\s*(\S+)/)
      if (m) { out.label_map[m[1]] = m[2]; continue }
      inLabelMap = false
    }
    if (inOrigin) {
      const m = rawLine.match(/^\s*-\s*(-?[\d.]+)/)
      if (m) { origin.push(Number(m[1])); continue }
      inOrigin = false
    }
    const m = rawLine.match(/^(\w+):\s*(.+)$/)
    if (m) {
      const key = m[1]
      const val = m[2].trim()
      out[key] = Number.isNaN(Number(val)) ? val : Number(val)
    }
  }
  out.origin = origin
  return out
}

const zoneMeta = parseZoneYaml(readFileSync(path.join(SCENE_ZONE_DIR, `${ZONE_PREFIX}_final.yaml`), 'utf-8'))
console.log('[scene_zone yaml]', zoneMeta)

// ── 4. semantic_map.yaml -> objects.json ───────────────────────────────
function parseSemanticMap(text) {
  // Struktur: <instance_label>:\n  cell_count: N\n  class_id: N\n  x: F\n  y: F
  const objects = []
  let current = null
  for (const rawLine of text.split('\n')) {
    if (!rawLine.trim()) continue
    if (!rawLine.startsWith(' ')) {
      const label = rawLine.replace(/:\s*$/, '').trim()
      if (!label) continue
      current = { id: label, name: label.replace(/_\d+$/, '') }
      objects.push(current)
      continue
    }
    if (!current) continue
    const m = rawLine.trim().match(/^(\w+):\s*(.+)$/)
    if (m) current[m[1]] = Number(m[2])
  }
  return objects
}

const objects = parseSemanticMap(readFileSync(path.join(SEMENTARA, 'semantic_map.yaml'), 'utf-8'))
console.log(`[semantic_map.yaml] ${objects.length} objek`)

// Decode scene_zones_labels.png (1-channel grayscale, sudah kita tulis di
// langkah 3) supaya bisa lookup zona (corridor/workshop/dst) per objek —
// perlu decoder PNG di sini, jadi baca ulang lewat fast-png.
const { decode: decodePng } = await import('fast-png')
const zoneLabelsPng = decodePng(readFileSync(path.join(OUT_DIR, 'scene_zones_labels.png')))

function worldToPixel(wx, wy) {
  const col = Math.floor((wx - labMapYaml.origin[0]) / labMapYaml.resolution)
  const rowFromBottom = Math.floor((wy - labMapYaml.origin[1]) / labMapYaml.resolution)
  const row = pgm.height - 1 - rowFromBottom
  if (col < 0 || col >= pgm.width || row < 0 || row >= pgm.height) return null
  return { row, col }
}

function zoneLabelAt(row, col) {
  return zoneLabelsPng.data[row * pgm.width + col]
}

function nearestNamedZone(px) {
  // Titik objek sering jatuh persis di sel occupied (objek itu sendiri
  // adalah obstacle di lidar), jadi label mentah di situ sering "wall".
  // Cari sel walkable-bernama terdekat (bukan wall/unknown) dlm radius
  // kecil — sama semangatnya dgn room_zone_lookup.get_room_id_nearest_walkable
  // di semantic/room_zone_lookup.py, tapi di sini cuma utk label tampilan.
  const raw = zoneLabelAt(px.row, px.col)
  if (raw !== 0 && raw !== 5) return zoneMeta.label_map[String(raw)]
  for (let r = 2; r <= 16; r += 2) {
    for (let dy = -r; dy <= r; dy++) {
      for (let dx = -r; dx <= r; dx++) {
        const row = px.row + dy
        const col = px.col + dx
        if (row < 0 || row >= pgm.height || col < 0 || col >= pgm.width) continue
        const lbl = zoneLabelAt(row, col)
        if (lbl !== 0 && lbl !== 5) return zoneMeta.label_map[String(lbl)]
      }
    }
  }
  return zoneMeta.label_map[String(raw)] ?? 'unknown'
}

for (const obj of objects) {
  const px = worldToPixel(obj.x, obj.y)
  obj.zone = px ? nearestNamedZone(px) : 'unknown'
}
writeFileSync(path.join(OUT_DIR, 'objects.json'), JSON.stringify(objects, null, 2))

// ── 5. walkable.json — grid free-space downsampled, utk A* di browser ──
// Sama persis konvensi map_server trinary (negate=0): <50 occupied(wall),
// >230 free, sisanya (unknown) DIANGGAP TERHALANG demi keamanan simulasi
// (jangan rencanakan lintasan lewat area yang belum pernah ke-scan).
const DOWNSAMPLE = 3
const gw = Math.ceil(pgm.width / DOWNSAMPLE)
const gh = Math.ceil(pgm.height / DOWNSAMPLE)
const walkable = new Uint8Array(gw * gh)
for (let gy = 0; gy < gh; gy++) {
  for (let gx = 0; gx < gw; gx++) {
    // Sebuah sel grid kasar dianggap walkable hanya jika SEMUA piksel asli
    // di dalamnya free — supaya robot (yg punya lebar fisik) tidak
    // direncanakan lewat celah yang sebenarnya sempit/dekat tembok.
    let allFree = true
    outer: for (let py = gy * DOWNSAMPLE; py < Math.min((gy + 1) * DOWNSAMPLE, pgm.height); py++) {
      for (let px_ = gx * DOWNSAMPLE; px_ < Math.min((gx + 1) * DOWNSAMPLE, pgm.width); px_++) {
        if (pgm.data[py * pgm.width + px_] <= 230) { allFree = false; break outer }
      }
    }
    walkable[gy * gw + gx] = allFree ? 1 : 0
  }
}
// Pack jadi bitset (8 sel per byte) lalu base64 — jauh lebih kecil dari JSON array.
const packed = new Uint8Array(Math.ceil((gw * gh) / 8))
for (let i = 0; i < gw * gh; i++) {
  if (walkable[i]) packed[i >> 3] |= 1 << (i & 7)
}
const walkableB64 = Buffer.from(packed).toString('base64')

writeFileSync(path.join(OUT_DIR, 'walkable.json'), JSON.stringify({
  width: gw,
  height: gh,
  downsample: DOWNSAMPLE,
  resolution: labMapYaml.resolution * DOWNSAMPLE,
  origin: labMapYaml.origin,
  fullHeightPx: pgm.height,
  packedBase64: walkableB64,
}))
console.log(`[walkable.json] grid ${gw}x${gh} (downsample ${DOWNSAMPLE}x), ${walkable.filter(Boolean).length} sel walkable`)

// ── 6. map_meta.json gabungan ───────────────────────────────────────────
const meta = {
  demo: true,
  note: 'Data DEMO dari folder sementara/ + output/scene_zone_test — ganti dengan export live map_server/RTABMap saat robot riil terhubung.',
  width: pgm.width,
  height: pgm.height,
  resolution: labMapYaml.resolution,
  origin: labMapYaml.origin,
  occupiedThresh: labMapYaml.occupied_thresh,
  freeThresh: labMapYaml.free_thresh,
  zoneLabelMap: zoneMeta.label_map,
}
writeFileSync(path.join(OUT_DIR, 'map_meta.json'), JSON.stringify(meta, null, 2))

console.log(`\nSelesai -> ${OUT_DIR}`)
