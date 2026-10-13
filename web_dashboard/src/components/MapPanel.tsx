import { useMemo, useRef, useState, type PointerEvent, type ReactNode, type WheelEvent } from 'react'
import type { MapMeta, SemanticObject } from '../lib/mapData'
import { worldToPixel, useMesh } from '../lib/mapData'
import { splitPathAtDistance } from '../lib/pathfinding'
import type { NavState, SimPose } from '../hooks/useSimEngine'
import { formatDuration } from '../lib/batteryEstimate'
import { Map3DViewer } from './Map3DViewer'

// Kategori objek semantic_map.yaml -> warna kategorikal (slot tetap dari
// palet ter-validasi skill dataviz, referensi/palette.md) supaya tiap
// kategori konsisten & CVD-safe. Ganti/​tambah entri di sini kalau kelas
// YOLO baru muncul di semantic_map.yaml yang riil nanti.
const CATEGORY_BY_NAME: Record<string, string> = {
  cabinet: 'furniture', shelf: 'furniture', table: 'furniture', 'o-seater': 'furniture',
  door: 'door',
  'fire-extinguisher': 'safety', 'fire-hydrant-box': 'safety',
  tv: 'electronics', laptop: 'electronics',
  stairs: 'hazard', 'vertical-ladder': 'hazard',
  'trash-can': 'misc', shoes: 'misc',
}
const CATEGORY_META: Record<string, { label: string; cssVar: string }> = {
  furniture: { label: 'Furnitur', cssVar: '--series-1' },
  door: { label: 'Pintu', cssVar: '--series-3' },
  safety: { label: 'Keselamatan', cssVar: '--series-6' },
  electronics: { label: 'Elektronik', cssVar: '--series-5' },
  hazard: { label: 'Tangga / akses vertikal', cssVar: '--series-8' },
  misc: { label: 'Lainnya', cssVar: '--series-2' },
}
function categoryOf(name: string) {
  return CATEGORY_BY_NAME[name] ?? 'misc'
}

const ZONE_META: { key: string; label: string; color: string }[] = [
  { key: 'corridor', label: 'Koridor', color: '#5dbf9e' },
  { key: 'workshop', label: 'Workshop / lab', color: '#e0916b' },
  { key: 'open_area', label: 'Area terbuka', color: '#e6d48f' },
  { key: 'doorway', label: 'Pintu', color: '#d0559a' },
  { key: 'staircase', label: 'Tangga', color: '#9a7fd6' },
]

function clamp(v: number, min: number, max: number) {
  return Math.min(max, Math.max(min, v))
}

/** Daftar objek + search box — dipakai di sidebar halaman Peta penuh MAUPUN
 * drawer ringkas di panel Overview, supaya navigasi selalu bisa dipicu dari
 * mana saja (bukan cuma dari halaman Peta terpisah). */
function ObjectSearchList({
  objects,
  search,
  onSearchChange,
  selectedId,
  onSelect,
  compact,
}: {
  objects: SemanticObject[]
  search: string
  onSearchChange: (v: string) => void
  selectedId: string | null
  onSelect: (id: string) => void
  compact?: boolean
}) {
  const sorted = useMemo(
    () => [...objects].sort((a, b) => a.name.localeCompare(b.name) || a.id.localeCompare(b.id)),
    [objects],
  )
  const filtered = useMemo(() => {
    if (!search.trim()) return sorted
    const q = search.trim().toLowerCase()
    return sorted.filter((o) => o.name.toLowerCase().includes(q) || o.zone.toLowerCase().includes(q))
  }, [sorted, search])

  return (
    <>
      <input
        className="map-search-input"
        placeholder="Cari objek… (mis. shelf, door)"
        value={search}
        onChange={(e) => onSearchChange(e.target.value)}
      />
      <div className={`map-object-list${compact ? ' compact' : ''}`}>
        {filtered.map((o) => (
          <button
            key={o.id}
            className={`map-object-row${o.id === selectedId ? ' selected' : ''}`}
            onClick={() => onSelect(o.id)}
          >
            <span
              className="map-object-dot-swatch"
              style={{ background: `var(${CATEGORY_META[categoryOf(o.name)].cssVar})` }}
            />
            <span className="map-object-row-name">{o.name}</span>
            <span className="map-object-row-zone">{o.zone}</span>
          </button>
        ))}
        {filtered.length === 0 && <div className="map-object-empty">Tidak ada objek cocok.</div>}
      </div>
    </>
  )
}

function NavActionsAndStatus({
  nav,
  selectedId,
  selectedName,
  onNavigate,
  onCancel,
  distanceRemaining,
  etaSeconds,
}: {
  nav: NavState
  selectedId: string | null
  selectedName: string | null
  onNavigate: (id: string) => void
  onCancel: () => void
  distanceRemaining: number
  etaSeconds: number | null
}) {
  return (
    <>
      <div className="map-nav-actions">
        <button
          className="map-nav-btn primary"
          disabled={!selectedId || nav.status === 'moving' || nav.status === 'planning'}
          onClick={() => selectedId && onNavigate(selectedId)}
        >
          Navigasi ke {selectedName ?? 'objek'}
        </button>
        {(nav.status === 'moving' || nav.status === 'planning') && (
          <button className="map-nav-btn" onClick={onCancel}>Batalkan</button>
        )}
      </div>

      <div className="map-nav-status">
        {nav.status === 'planning' && <span>Merencanakan jalur…</span>}
        {nav.status === 'moving' && nav.targetName && (
          <span>
            Menuju <strong>{nav.targetName}</strong> — sisa {distanceRemaining.toFixed(1)} m
            {etaSeconds != null ? ` · ETA ${formatDuration(etaSeconds)}` : ''}
          </span>
        )}
        {nav.status === 'arrived' && nav.targetName && <span>Sampai di {nav.targetName}.</span>}
        {nav.status === 'failed' && <span className="map-nav-error">{nav.error}</span>}
        {nav.status === 'idle' && <span>Pilih objek lalu klik Navigasi untuk simulasi pergerakan.</span>}
      </div>
    </>
  )
}

function Legend({ compact }: { compact?: boolean }) {
  return (
    <div className={`map-legend${compact ? ' compact' : ''}`}>
      <div className="map-legend-title">Scene zone</div>
      {ZONE_META.map((z) => (
        <div key={z.key} className="map-legend-row">
          <span className="map-legend-swatch" style={{ background: z.color }} />
          {z.label}
        </div>
      ))}
      <div className="map-legend-title" style={{ marginTop: 10 }}>Objek</div>
      {Object.entries(CATEGORY_META).map(([key, m]) => (
        <div key={key} className="map-legend-row">
          <span className="map-legend-swatch" style={{ background: `var(${m.cssVar})` }} />
          {m.label}
        </div>
      ))}
    </div>
  )
}

interface MapPanelProps {
  meta: MapMeta
  objects: SemanticObject[]
  pose: SimPose
  moving: boolean
  nav: NavState
  distanceRemaining: number
  etaSeconds: number | null
  onNavigate: (objectId: string) => void
  onCancel: () => void
  showZones: boolean
  showObjects: boolean
  onToggleZones: () => void
  onToggleObjects: () => void
}

function RobotMarker({ meta, pose, moving }: { meta: MapMeta; pose: SimPose; moving: boolean }) {
  const px = worldToPixel(meta, pose.x, pose.y)
  const rotationDeg = -(pose.yaw * 180) / Math.PI
  return (
    <g transform={`translate(${px.col},${px.row})`}>
      {moving && <circle r={10} className="robot-pulse" />}
      <g transform={`rotate(${rotationDeg})`}>
        <path d="M9,0 L-6,6 L-2,0 L-6,-6 Z" className="robot-arrow" />
      </g>
    </g>
  )
}

function MapSvg({
  meta,
  objects,
  pose,
  moving,
  nav,
  showZones,
  showObjects,
  interactive,
  selectedId,
  onSelectObject,
}: {
  meta: MapMeta
  objects: SemanticObject[]
  pose: SimPose
  moving: boolean
  nav: NavState
  showZones: boolean
  showObjects: boolean
  interactive: boolean
  selectedId: string | null
  onSelectObject?: (id: string) => void
}) {
  const { traveled, remaining } = useMemo(
    () => splitPathAtDistance(nav.path, nav.traveledDistance),
    [nav.path, nav.traveledDistance],
  )

  const traveledPx = traveled.map((p) => worldToPixel(meta, p.x, p.y))
  const remainingPx = remaining.map((p) => worldToPixel(meta, p.x, p.y))
  const goal = nav.path.length > 0 ? nav.path[nav.path.length - 1] : null
  const goalPx = goal ? worldToPixel(meta, goal.x, goal.y) : null

  return (
    <svg viewBox={`0 0 ${meta.width} ${meta.height}`} width={meta.width} height={meta.height} className="map-svg">
      <image href="/map/lab_map.png" x={0} y={0} width={meta.width} height={meta.height} />
      {showZones && (
        <image
          href="/map/scene_zones.png"
          x={0}
          y={0}
          width={meta.width}
          height={meta.height}
          className="map-zone-layer"
        />
      )}

      {traveledPx.length > 1 && (
        <polyline
          points={traveledPx.map((p) => `${p.col},${p.row}`).join(' ')}
          className="path-line path-traveled"
        />
      )}
      {remainingPx.length > 1 && (
        <polyline
          points={remainingPx.map((p) => `${p.col},${p.row}`).join(' ')}
          className="path-line path-remaining"
        />
      )}
      {goalPx && (
        <g transform={`translate(${goalPx.col},${goalPx.row})`}>
          {(nav.status === 'moving' || nav.status === 'planning') && <circle r={9} className="goal-pulse" />}
          <circle r={5} className="goal-dot" />
        </g>
      )}

      {showObjects &&
        objects.map((obj) => {
          const p = worldToPixel(meta, obj.x, obj.y)
          const category = categoryOf(obj.name)
          const meta_ = CATEGORY_META[category]
          const isSelected = obj.id === selectedId
          return (
            <circle
              key={obj.id}
              cx={p.col}
              cy={p.row}
              r={isSelected ? 5.5 : 3.5}
              className={`object-dot${isSelected ? ' selected' : ''}`}
              style={{ fill: `var(${meta_.cssVar})` }}
              onClick={interactive && onSelectObject ? () => onSelectObject(obj.id) : undefined}
            >
              <title>{`${obj.name} (${obj.zone})`}</title>
            </circle>
          )
        })}

      <RobotMarker meta={meta} pose={pose} moving={moving} />
    </svg>
  )
}

/** Canvas peta pan/zoom yang dipakai bersama oleh panel Peta penuh MAUPUN
 * panel split-view di Overview — supaya interaksi (drag, scroll-zoom)
 * konsisten di semua tempat map muncul. */
export function MapCanvas({
  meta,
  objects,
  pose,
  moving,
  nav,
  showZones,
  showObjects,
  interactive,
  selectedId,
  onSelectObject,
  children,
}: {
  meta: MapMeta
  objects: SemanticObject[]
  pose: SimPose
  moving: boolean
  nav: NavState
  showZones: boolean
  showObjects: boolean
  interactive: boolean
  selectedId?: string | null
  onSelectObject?: (id: string) => void
  children?: ReactNode
}) {
  const [transform, setTransform] = useState({ x: 0, y: 0, scale: 1 })
  const dragRef = useRef<{ startX: number; startY: number; origX: number; origY: number } | null>(null)
  const draggedRef = useRef(false)

  function onWheel(e: WheelEvent) {
    if (!interactive) return
    e.preventDefault()
    setTransform((t) => ({ ...t, scale: clamp(t.scale * (e.deltaY < 0 ? 1.12 : 0.89), 0.6, 6) }))
  }
  function onPointerDown(e: PointerEvent) {
    if (!interactive) return
    ;(e.target as Element).setPointerCapture(e.pointerId)
    draggedRef.current = false
    dragRef.current = { startX: e.clientX, startY: e.clientY, origX: transform.x, origY: transform.y }
  }
  function onPointerMove(e: PointerEvent) {
    const drag = dragRef.current
    if (!drag) return
    const dx = e.clientX - drag.startX
    const dy = e.clientY - drag.startY
    if (Math.abs(dx) > 3 || Math.abs(dy) > 3) draggedRef.current = true
    // Snapshot origX/origY LOKAL (bukan baca dragRef.current lagi di dalam
    // updater) — setTransform bisa dieksekusi belakangan oleh React (batched),
    // dan pada saat itu dragRef.current mungkin sudah di-null-kan oleh
    // onPointerUp yang keburu jalan duluan, bikin crash "reading origX of null".
    setTransform((t) => ({ ...t, x: drag.origX + dx, y: drag.origY + dy }))
  }
  function onPointerUp() {
    dragRef.current = null
  }
  function resetView() {
    setTransform({ x: 0, y: 0, scale: 1 })
  }

  return (
    <div
      className="map-canvas-wrap"
      onWheel={onWheel}
      onPointerDown={onPointerDown}
      onPointerMove={onPointerMove}
      onPointerUp={onPointerUp}
      onPointerLeave={onPointerUp}
    >
      <div
        className="map-canvas-transform"
        style={{ transform: `translate(${transform.x}px, ${transform.y}px) scale(${transform.scale})` }}
      >
        <MapSvg
          meta={meta}
          objects={objects}
          pose={pose}
          moving={moving}
          nav={nav}
          showZones={showZones}
          showObjects={showObjects}
          interactive={interactive}
          selectedId={selectedId ?? null}
          onSelectObject={
            interactive && onSelectObject ? (id) => !draggedRef.current && onSelectObject(id) : undefined
          }
        />
      </div>
      {interactive && (transform.x !== 0 || transform.y !== 0 || transform.scale !== 1) && (
        <button className="map-reset-btn map-reset-btn-floating" onClick={resetView}>
          Reset tampilan
        </button>
      )}
      {children}
    </div>
  )
}

export function MapPanel({
  meta,
  objects,
  pose,
  moving,
  nav,
  distanceRemaining,
  etaSeconds,
  onNavigate,
  onCancel,
  showZones,
  showObjects,
  onToggleZones,
  onToggleObjects,
}: MapPanelProps) {
  const [selectedId, setSelectedId] = useState<string | null>(null)
  const [search, setSearch] = useState('')
  const [view3d, setView3d] = useState(false)
  const { data: mesh, error: meshError } = useMesh()

  const selectedObject = objects.find((o) => o.id === selectedId) ?? null

  return (
    <div className="panel-detail map-panel">
      <header className="panel-detail-header">
        <h2>Peta</h2>
        <div className="map-layer-toggles">
          <div className="toggle-pill-group">
            <button
              className={`toggle-pill-btn${!view3d ? ' active' : ''}`}
              onClick={() => setView3d(false)}
            >
              2D
            </button>
            <button
              className={`toggle-pill-btn${view3d ? ' active' : ''}`}
              onClick={() => setView3d(true)}
            >
              3D
            </button>
          </div>
          {!view3d && (
            <label className="toggle-pill">
              <input type="checkbox" checked={showZones} onChange={onToggleZones} />
              Scene zone
            </label>
          )}
          <label className="toggle-pill">
            <input type="checkbox" checked={showObjects} onChange={onToggleObjects} />
            Objek semantic
          </label>
        </div>
      </header>

      <div className="map-body">
        {view3d ? (
          meshError ? (
            <div className="map-3d-error">Gagal memuat mesh 3D: {meshError.message}</div>
          ) : (
            <Map3DViewer mesh={mesh} objects={showObjects ? objects : []} pose={pose} />
          )
        ) : (
          <MapCanvas
            meta={meta}
            objects={objects}
            pose={pose}
            moving={moving}
            nav={nav}
            showZones={showZones}
            showObjects={showObjects}
            interactive
            selectedId={selectedId}
            onSelectObject={setSelectedId}
          />
        )}

        <aside className="map-sidebar-panel">
          <div className="map-nav-card">
            <label className="map-field-label">Navigasi ke objek (simulasi)</label>
            <ObjectSearchList
              objects={objects}
              search={search}
              onSearchChange={setSearch}
              selectedId={selectedId}
              onSelect={setSelectedId}
            />
            <NavActionsAndStatus
              nav={nav}
              selectedId={selectedId}
              selectedName={selectedObject?.name ?? null}
              onNavigate={onNavigate}
              onCancel={onCancel}
              distanceRemaining={distanceRemaining}
              etaSeconds={etaSeconds}
            />
          </div>

          <Legend />
        </aside>
      </div>

      {/* sim-note disembunyikan sementara atas permintaan user (2026-07-25) — konten aslinya
          dipertahankan di sini, tinggal un-comment kalau mau ditampilkan lagi.
      <div className="sim-note">
        <strong>Simulasi.</strong> Peta, zona, dan titik objek di sini adalah data demo (folder{' '}
        <code>sementara/</code>). Jalur navigasi dihitung dengan A* lokal di browser murni untuk
        visualisasi — pada robot Husky A200 riil, ganti dengan action Nav2 <code>NavigateToPose</code>{' '}
        dan gambar jalur dari <code>/plan</code>/feedback action, bukan hasil pathfinding lokal ini.
        <br />
        <strong>Peta 3D:</strong> toggle di atas ("3D") pakai MESH (permukaan solid, bukan point
        cloud/titik terpisah) hasil ekstraksi <code>sementara/rtabmap.db</code> lewat{' '}
        <code>rtabmap-export --mesh</code> (CLI resmi RTAB-Map, bukan hand-parse binary internal) — lihat{' '}
        <code>scripts/prepare_mesh.py</code>. ~139rb vertex / ~188rb segitiga (~4.3MB binary); jalankan
        ulang script itu (~5-6 menit, proses berat) kalau <code>rtabmap.db</code> live sudah tersedia
        dari robot riil.
      </div>
      */}
    </div>
  )
}

/** Panel peta di sisi split-view Overview — sejajar dengan viewer robot
 * (bukan kartu kecil di pojok). Interaktif penuh (pan/zoom + objek + navigasi
 * simulasi langsung dari sini, TIDAK perlu pindah ke halaman Peta terpisah),
 * dengan toolbar toggle layer, legend ringkas, dan tombol buka halaman Peta
 * detail untuk yang mau tampilan lebih luas. */
export function OverviewMapPane({
  meta,
  objects,
  pose,
  moving,
  nav,
  distanceRemaining,
  etaSeconds,
  onNavigate,
  onCancel,
  showZones,
  showObjects,
  onToggleZones,
  onToggleObjects,
  onExpand,
  onHide,
}: {
  meta: MapMeta
  objects: SemanticObject[]
  pose: SimPose
  moving: boolean
  nav: NavState
  distanceRemaining: number
  etaSeconds: number | null
  onNavigate: (objectId: string) => void
  onCancel: () => void
  showZones: boolean
  showObjects: boolean
  onToggleZones: () => void
  onToggleObjects: () => void
  onExpand: () => void
  onHide: () => void
}) {
  const [navOpen, setNavOpen] = useState(false)
  const [legendOpen, setLegendOpen] = useState(false)
  const [selectedId, setSelectedId] = useState<string | null>(null)
  const [search, setSearch] = useState('')
  const selectedObject = objects.find((o) => o.id === selectedId) ?? null

  return (
    <MapCanvas
      meta={meta}
      objects={objects}
      pose={pose}
      moving={moving}
      nav={nav}
      showZones={showZones}
      showObjects={showObjects}
      interactive
      selectedId={selectedId}
      onSelectObject={(id) => {
        setSelectedId(id)
        setNavOpen(true)
      }}
    >
      <div className="map-pane-toolbar">
        <button
          className={`toolbar-chip${showZones ? ' active' : ''}`}
          onClick={(e) => {
            e.stopPropagation()
            onToggleZones()
          }}
        >
          Zona
        </button>
        <button
          className={`toolbar-chip${showObjects ? ' active' : ''}`}
          onClick={(e) => {
            e.stopPropagation()
            onToggleObjects()
          }}
        >
          Objek
        </button>
        <button
          className={`toolbar-chip icon-chip${legendOpen ? ' active' : ''}`}
          title="Legend"
          onClick={(e) => {
            e.stopPropagation()
            setLegendOpen((v) => !v)
          }}
        >
          ?
        </button>
      </div>
      {legendOpen && (
        <div className="map-pane-legend-popover" onClick={(e) => e.stopPropagation()}>
          <Legend compact />
        </div>
      )}

      <button
        className="map-pane-hide"
        title="Sembunyikan peta"
        onClick={(e) => {
          e.stopPropagation()
          onHide()
        }}
      >
        ✕
      </button>
      <button className="map-pane-expand" onClick={onExpand}>
        Buka peta detail →
      </button>

      <div className={`map-pane-nav-drawer${navOpen ? ' open' : ''}`} onClick={(e) => e.stopPropagation()}>
        {navOpen ? (
          <>
            <div className="map-pane-nav-drawer-header">
              <span>Navigasi ke objek</span>
              <button className="map-pane-nav-drawer-close" onClick={() => setNavOpen(false)}>✕</button>
            </div>
            <ObjectSearchList
              objects={objects}
              search={search}
              onSearchChange={setSearch}
              selectedId={selectedId}
              onSelect={setSelectedId}
              compact
            />
            <NavActionsAndStatus
              nav={nav}
              selectedId={selectedId}
              selectedName={selectedObject?.name ?? null}
              onNavigate={onNavigate}
              onCancel={onCancel}
              distanceRemaining={distanceRemaining}
              etaSeconds={etaSeconds}
            />
          </>
        ) : (
          <button className="map-pane-nav-trigger" onClick={() => setNavOpen(true)}>
            Navigasi ke objek…
          </button>
        )}
      </div>
    </MapCanvas>
  )
}
