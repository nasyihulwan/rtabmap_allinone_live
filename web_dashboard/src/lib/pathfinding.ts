import type { MapMeta, WalkableGrid } from './mapData'
import { gridCellToWorld, isWalkableCell, worldToGridCell } from './mapData'

interface Cell {
  gx: number
  gy: number
}

const NEIGHBORS: [number, number, number][] = [
  [1, 0, 1], [-1, 0, 1], [0, 1, 1], [0, -1, 1],
  [1, 1, Math.SQRT2], [1, -1, Math.SQRT2], [-1, 1, Math.SQRT2], [-1, -1, Math.SQRT2],
]

class MinHeap<T> {
  private items: { priority: number; value: T }[] = []

  push(priority: number, value: T) {
    this.items.push({ priority, value })
    let i = this.items.length - 1
    while (i > 0) {
      const parent = (i - 1) >> 1
      if (this.items[parent].priority <= this.items[i].priority) break
      ;[this.items[parent], this.items[i]] = [this.items[i], this.items[parent]]
      i = parent
    }
  }

  pop(): T | undefined {
    if (this.items.length === 0) return undefined
    const top = this.items[0]
    const last = this.items.pop()!
    if (this.items.length > 0) {
      this.items[0] = last
      let i = 0
      for (;;) {
        const l = i * 2 + 1
        const r = i * 2 + 2
        let smallest = i
        if (l < this.items.length && this.items[l].priority < this.items[smallest].priority) smallest = l
        if (r < this.items.length && this.items[r].priority < this.items[smallest].priority) smallest = r
        if (smallest === i) break
        ;[this.items[smallest], this.items[i]] = [this.items[i], this.items[smallest]]
        i = smallest
      }
    }
    return top.value
  }

  get size() {
    return this.items.length
  }
}

/** Cari sel walkable terdekat dari (gx,gy) via spiral BFS — dipakai saat titik
 * start/goal jatuh di sel terhalang (mis. goal = posisi objek yang secara
 * fisik adalah obstacle di lidar). */
export function nearestWalkable(grid: WalkableGrid, gx: number, gy: number, maxRadius = 40): Cell | null {
  if (isWalkableCell(grid, gx, gy)) return { gx, gy }
  for (let r = 1; r <= maxRadius; r++) {
    for (let dy = -r; dy <= r; dy++) {
      for (let dx = -r; dx <= r; dx++) {
        if (Math.max(Math.abs(dx), Math.abs(dy)) !== r) continue
        const nx = gx + dx
        const ny = gy + dy
        if (isWalkableCell(grid, nx, ny)) return { gx: nx, gy: ny }
      }
    }
  }
  return null
}

function cellKey(gx: number, gy: number) {
  return gy * 100000 + gx
}

/** A* di atas grid walkable (8-connected). Return jalur dunia (meter) dari
 * start ke goal, atau null kalau tidak ada jalur. */
export function findPath(
  meta: MapMeta,
  grid: WalkableGrid,
  startWorld: { x: number; y: number },
  goalWorld: { x: number; y: number },
): { x: number; y: number }[] | null {
  const startCellRaw = worldToGridCell(meta, grid, startWorld.x, startWorld.y)
  const goalCellRaw = worldToGridCell(meta, grid, goalWorld.x, goalWorld.y)
  const start = nearestWalkable(grid, startCellRaw.gx, startCellRaw.gy)
  const goal = nearestWalkable(grid, goalCellRaw.gx, goalCellRaw.gy)
  if (!start || !goal) return null

  const heuristic = (a: Cell, b: Cell) => Math.hypot(a.gx - b.gx, a.gy - b.gy)

  const gScore = new Map<number, number>()
  const cameFrom = new Map<number, number>()
  const open = new MinHeap<Cell>()

  gScore.set(cellKey(start.gx, start.gy), 0)
  open.push(heuristic(start, goal), start)

  const visited = new Set<number>()
  let reached = false
  const maxIterations = grid.width * grid.height
  let iterations = 0

  while (open.size > 0 && iterations++ < maxIterations) {
    const current = open.pop()!
    const ck = cellKey(current.gx, current.gy)
    if (visited.has(ck)) continue
    visited.add(ck)

    if (current.gx === goal.gx && current.gy === goal.gy) {
      reached = true
      break
    }

    const currentG = gScore.get(ck) ?? Infinity
    for (const [dx, dy, cost] of NEIGHBORS) {
      const nx = current.gx + dx
      const ny = current.gy + dy
      if (!isWalkableCell(grid, nx, ny)) continue
      // Cegah "motong sudut" diagonal lewat celah tembok selebar 1 sel.
      if (dx !== 0 && dy !== 0) {
        if (!isWalkableCell(grid, current.gx + dx, current.gy) || !isWalkableCell(grid, current.gx, current.gy + dy)) {
          continue
        }
      }
      const nk = cellKey(nx, ny)
      const tentativeG = currentG + cost
      if (tentativeG < (gScore.get(nk) ?? Infinity)) {
        gScore.set(nk, tentativeG)
        cameFrom.set(nk, ck)
        open.push(tentativeG + heuristic({ gx: nx, gy: ny }, goal), { gx: nx, gy: ny })
      }
    }
  }

  if (!reached) return null

  const pathCells: Cell[] = []
  let curKey = cellKey(goal.gx, goal.gy)
  const keyToCell = (k: number): Cell => ({ gx: k % 100000, gy: Math.floor(k / 100000) })
  pathCells.push(goal)
  while (cameFrom.has(curKey)) {
    curKey = cameFrom.get(curKey)!
    pathCells.push(keyToCell(curKey))
  }
  pathCells.reverse()

  const worldPath = pathCells.map((c) => gridCellToWorld(meta, grid, c.gx, c.gy))
  return simplifyCollinear(worldPath)
}

/** Buang titik-titik yang segaris dgn tetangganya supaya polyline animasi
 * halus (bukan zig-zag per-sel grid). */
function simplifyCollinear(points: { x: number; y: number }[]): { x: number; y: number }[] {
  if (points.length <= 2) return points
  const out = [points[0]]
  for (let i = 1; i < points.length - 1; i++) {
    const a = out[out.length - 1]
    const b = points[i]
    const c = points[i + 1]
    const cross = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)
    if (Math.abs(cross) > 1e-6) out.push(b)
  }
  out.push(points[points.length - 1])
  return out
}

export function pathLength(points: { x: number; y: number }[]): number {
  let total = 0
  for (let i = 1; i < points.length; i++) {
    total += Math.hypot(points[i].x - points[i - 1].x, points[i].y - points[i - 1].y)
  }
  return total
}

export interface PathSample {
  point: { x: number; y: number }
  /** Sudut hadap (radian, konvensi ROS/REP103) menuju titik berikutnya. */
  heading: number
}

/** Cari posisi + heading pada jarak tempuh tertentu di sepanjang polyline. */
export function pointAtDistance(points: { x: number; y: number }[], distance: number): PathSample {
  if (points.length === 0) return { point: { x: 0, y: 0 }, heading: 0 }
  if (points.length === 1) return { point: points[0], heading: 0 }

  let remaining = Math.max(0, distance)
  for (let i = 1; i < points.length; i++) {
    const a = points[i - 1]
    const b = points[i]
    const segLen = Math.hypot(b.x - a.x, b.y - a.y)
    const heading = Math.atan2(b.y - a.y, b.x - a.x)
    if (remaining <= segLen || i === points.length - 1) {
      const t = segLen === 0 ? 0 : Math.min(1, remaining / segLen)
      return { point: { x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t }, heading }
    }
    remaining -= segLen
  }
  const last = points[points.length - 1]
  const prev = points[points.length - 2]
  return { point: last, heading: Math.atan2(last.y - prev.y, last.x - prev.x) }
}

/** Pecah polyline jadi bagian [sudah dilalui] dan [sisa] pada jarak tertentu
 * — dipakai buat gambar path dgn warna beda (dilalui vs belum). */
export function splitPathAtDistance(
  points: { x: number; y: number }[],
  distance: number,
): { traveled: { x: number; y: number }[]; remaining: { x: number; y: number }[] } {
  if (points.length === 0) return { traveled: [], remaining: [] }
  const { point: cut } = pointAtDistance(points, distance)
  const traveled: { x: number; y: number }[] = [points[0]]
  const remaining: { x: number; y: number }[] = []
  let acc = 0
  let cutInserted = false
  for (let i = 1; i < points.length; i++) {
    const a = points[i - 1]
    const b = points[i]
    const segLen = Math.hypot(b.x - a.x, b.y - a.y)
    if (!cutInserted && acc + segLen >= distance) {
      traveled.push(cut)
      remaining.push(cut)
      cutInserted = true
    } else if (!cutInserted) {
      traveled.push(b)
    } else {
      remaining.push(b)
    }
    acc += segLen
  }
  if (!cutInserted) remaining.push(points[points.length - 1])
  return { traveled, remaining }
}
