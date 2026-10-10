"""Pembuat rute pengepelan zig-zag (boustrophedon) otomatis dari peta.

Tidak bergantung pada ROS, jadi bisa diuji tanpa robot (lihat test/).

Langkah:
  1. Area kosong peta dirapikan: lubang noise kecil ditutup, "garis sinar"
     tipis keluar ruangan (LiDAR tembus pintu/jendela) dibuang -- yang tersisa
     hanya area yang badan robot muat.
  2. Titik tengah robot boleh di mana: jarak ke dinding/benda/area tak dikenal
     minimal `clearance_m`.
  3. Arah dinding utama ruangan dicari (sudut yang membuat area paling "kotak"),
     baris zig-zag dibuat lurus sejajar arah itu, jarak antar baris `spacing_m`.
  4. Segmen baris diurutkan: ambil ujung segmen terdekat berikutnya; pindah
     antar segmen lewat jalur aman terpendek (A*). Terakhir pulang ke titik awal.
"""

import heapq
import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import numpy as np
from scipy import ndimage
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra

XY = Tuple[float, float]


@dataclass
class Waypoint:
    x: float
    y: float
    kind: str = "row"  # "row" (baris pel), "turn" (pindah baris), "travel" (lewat A*), "home"


@dataclass
class CoveragePlan:
    waypoints: List[Waypoint]
    angle_rad: float
    rows: List[Tuple[XY, XY]]
    area_mask: np.ndarray          # True = titik tengah robot boleh di sini
    coverage_length_m: float = 0.0
    travel_length_m: float = 0.0
    notes: List[str] = field(default_factory=list)


class GridMap:
    def __init__(self, grid: np.ndarray, resolution: float, origin_x: float, origin_y: float) -> None:
        """grid: nilai OccupancyGrid (-1/0/100), baris 0 = y terkecil."""
        self.grid = grid
        self.res = float(resolution)
        self.ox = float(origin_x)
        self.oy = float(origin_y)
        self.h, self.w = grid.shape

    def to_cell(self, x: float, y: float) -> Tuple[int, int]:
        return int((y - self.oy) / self.res), int((x - self.ox) / self.res)

    def to_xy(self, r: float, c: float) -> XY:
        return self.ox + (c + 0.5) * self.res, self.oy + (r + 0.5) * self.res


def _disk(radius_cells: int) -> np.ndarray:
    r = max(1, radius_cells)
    y, x = np.ogrid[-r:r + 1, -r:r + 1]
    return x * x + y * y <= r * r


def clean_map(m: GridMap, min_obstacle_m2: float = 0.04, max_hole_m2: float = 0.5) -> Tuple[np.ndarray, np.ndarray]:
    """Peta untuk PERENCANAAN rute (peta asli tidak diubah).
    Bintik hitam kecil (noise / kaki kursi) diabaikan -- saat jalan, LiDAR tetap
    menghentikan robot kalau ada benda nyata di depannya. Lubang "tidak dikenal"
    kecil di tengah area kosong dianggap kosong.
    Hasil: (kosong, penghalang_besar)."""
    occupied = m.grid >= 65
    lab, n = ndimage.label(occupied, structure=np.ones((3, 3)))
    big = occupied.copy()
    if n:
        sizes = ndimage.sum(np.ones_like(lab), lab, range(1, n + 1))
        small_ids = np.nonzero(sizes * m.res * m.res < min_obstacle_m2)[0] + 1
        big &= ~np.isin(lab, small_ids)
    free = (m.grid == 0) | (occupied & ~big)
    holes = ndimage.binary_fill_holes(free) & ~free & ~big
    hlab, hn = ndimage.label(holes)
    if hn:
        hsizes = ndimage.sum(np.ones_like(hlab), hlab, range(1, hn + 1))
        small_holes = np.nonzero(hsizes * m.res * m.res <= max_hole_m2)[0] + 1
        free |= np.isin(hlab, small_holes)
    return free, big


def polygon_mask(m: GridMap, polygon: Sequence[XY]) -> np.ndarray:
    """True = sel di DALAM batas area yang digambar operator di RViz."""
    from matplotlib.path import Path

    rr, cc = np.mgrid[0:m.h, 0:m.w]
    xs = m.ox + (cc + 0.5) * m.res
    ys = m.oy + (rr + 0.5) * m.res
    inside = Path(list(polygon)).contains_points(np.stack([xs.ravel(), ys.ravel()], axis=1))
    return inside.reshape(m.h, m.w)


def safe_area(
    m: GridMap,
    clearance_m: float = 0.30,
    hole_fill_m: float = 0.15,
    start_xy: Optional[XY] = None,
    min_obstacle_m2: float = 0.04,
    max_hole_m2: float = 0.5,
    boundary: Optional[Sequence[XY]] = None,
    boundary_extra_m: float = 0.0,
) -> np.ndarray:
    """Mask sel tempat titik tengah robot aman, satu area tersambung saja.
    `boundary` (>= 3 titik) = batas area dari operator: di luar batas dianggap
    dinding (mis. kaca yang tembus LiDAR), dengan jarak aman yang sama."""
    free, big = clean_map(m, min_obstacle_m2, max_hole_m2)
    if boundary is not None and len(boundary) >= 3:
        inside = polygon_mask(m, boundary)
        if boundary_extra_m > 0:  # jarak aman ekstra khusus dari garis batas
            inside &= ndimage.distance_transform_edt(inside) * m.res >= boundary_extra_m
        free &= inside
    # Tutup celah sempit sisa noise, tapi jangan sampai menimpa penghalang besar.
    k = _disk(int(round(hole_fill_m / m.res)))
    free = ndimage.binary_closing(free, structure=k) & ~ndimage.binary_dilation(big, structure=_disk(1))
    # Jarak ke sel yang bukan bebas (dinding, benda besar, area tak dikenal).
    dist = ndimage.distance_transform_edt(free) * m.res
    ok = dist >= clearance_m
    # Garis sinar tipis keluar ruangan otomatis hilang di sini (lebarnya < 2 x clearance).
    lab, n = ndimage.label(ok)
    if n == 0:
        return ok
    if start_xy is not None:
        r, c = m.to_cell(*start_xy)
        rr, cc = np.nonzero(ok)
        nearest = int(np.argmin((rr - r) ** 2 + (cc - c) ** 2))
        keep = lab[rr[nearest], cc[nearest]]
    else:
        sizes = ndimage.sum(np.ones_like(lab), lab, range(1, n + 1))
        keep = int(np.argmax(sizes)) + 1
    return lab == keep


def dominant_angle(mask: np.ndarray, res: float) -> float:
    """Sudut (rad, 0..pi/2) yang membuat kotak pembatas area paling kecil =
    arah dinding utama ruangan. Baris zig-zag dibuat sejajar sisi terpanjang."""
    rr, cc = np.nonzero(mask)
    pts = np.stack([cc, rr], axis=1).astype(float) * res
    pts -= pts.mean(axis=0)
    best = (float("inf"), 0.0, 0.0, 0.0)
    for deg in np.arange(0.0, 90.0, 0.5):
        a = math.radians(deg)
        x = pts[:, 0] * math.cos(a) + pts[:, 1] * math.sin(a)
        y = -pts[:, 0] * math.sin(a) + pts[:, 1] * math.cos(a)
        w, h = np.ptp(x), np.ptp(y)
        if w * h < best[0]:
            best = (w * h, a, w, h)
    _, a, w, h = best
    return a if w >= h else a + math.pi / 2  # baris sepanjang sisi panjang


@dataclass
class Segment:
    row: int
    u0: float
    u1: float
    a: XY
    b: XY
    v: float = 0.0


BRIDGE_GAP_M = 0.0  # diisi plan_coverage: celah sependek ini di tengah lintasan disambung


def _row_segments(
    m: GridMap, mask: np.ndarray, angle: float, spacing_m: float, min_len_m: float, fit: bool = False
) -> List[Segment]:
    """Potongan baris sejajar `angle`. fit=True: lintasan dipusatkan di lebar area
    (sisa tepi sama besar), lihat komentar di bawah."""
    rr, cc = np.nonzero(mask)
    xs = m.ox + (cc + 0.5) * m.res
    ys = m.oy + (rr + 0.5) * m.res
    ca, sa = math.cos(angle), math.sin(angle)
    u = xs * ca + ys * sa          # sepanjang baris
    v = -xs * sa + ys * ca         # tegak lurus baris
    step = m.res / 2
    rows: List[Segment] = []
    lo, hi = float(v.min()), float(v.max())
    width = hi - lo
    if fit:
        # Jumlah lintasan pas dengan lebar area, jarak tetap spacing_m, semua lintasan
        # DIGESER KE TENGAH: sisa di tiap tepi sama besar (<= spacing/2 = 20 cm), dan
        # setengah lebar pel (25 cm) menutup sisa itu -> tepi terkena pel tanpa
        # lintasan tambahan, titik tengah robot tidak pernah keluar area aman.
        n = int(math.floor(width / spacing_m + 1e-9)) + 1
        offset = (width - (n - 1) * spacing_m) / 2
        vs = [lo + offset + k * spacing_m for k in range(n)]
    else:
        vs = list(np.arange(lo + spacing_m / 2, hi + 1e-9, spacing_m))
    for row_id, vk in enumerate(vs):
        vk = float(vk)
        us = np.arange(u.min(), u.max() + step, step)
        px = us * ca - vk * sa
        py = us * sa + vk * ca
        r = ((py - m.oy) / m.res).astype(int)
        c = ((px - m.ox) / m.res).astype(int)
        inside = (r >= 0) & (r < m.h) & (c >= 0) & (c < m.w)
        on = np.zeros(len(us), dtype=bool)
        on[inside] = mask[r[inside], c[inside]]
        if BRIDGE_GAP_M > 0 and on.any():
            # sambung celah pendek di TENGAH lintasan (bintik peta / kaki kursi) supaya
            # lintasan utuh lurus; benda nyata tetap dihindari saat jalan (kembali3)
            idx = np.nonzero(on)[0]
            gaps = np.nonzero(np.diff(idx) > 1)[0]
            for g in gaps:
                a, b = idx[g], idx[g + 1]
                if (b - a) * step <= BRIDGE_GAP_M:
                    on[a:b] = True
        # potong jadi interval-interval kontinu
        edges = np.diff(np.concatenate([[0], on.astype(int), [0]]))
        starts, ends = np.nonzero(edges == 1)[0], np.nonzero(edges == -1)[0] - 1
        for s, e in zip(starts, ends):
            if (e - s) * step >= min_len_m:
                rows.append(Segment(row_id, float(us[s]), float(us[e]),
                                    (float(px[s]), float(py[s])), (float(px[e]), float(py[e])), float(vk)))
    return rows


def _astar(mask: np.ndarray, start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
    h, w = mask.shape
    if not (0 <= goal[0] < h and 0 <= goal[1] < w):
        return None
    moves = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
             (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414)]
    g = {start: 0.0}
    prev = {}
    pq = [(0.0, start)]
    while pq:
        _, cur = heapq.heappop(pq)
        if cur == goal:
            path = [cur]
            while cur in prev:
                cur = prev[cur]
                path.append(cur)
            return path[::-1]
        for dr, dc, cost in moves:
            nb = (cur[0] + dr, cur[1] + dc)
            if not (0 <= nb[0] < h and 0 <= nb[1] < w) or not mask[nb]:
                continue
            ng = g[cur] + cost
            if ng < g.get(nb, float("inf")):
                g[nb] = ng
                prev[nb] = cur
                heapq.heappush(pq, (ng + math.hypot(goal[0] - nb[0], goal[1] - nb[1]), nb))
    return None


class Geodesic:
    """Jarak jalan sebenarnya (menghindari dinding) di dalam area aman,
    pakai Dijkstra sekali per titik asal."""

    def __init__(self, mask: np.ndarray, res: float) -> None:
        self.mask = mask
        self.res = res
        self.idx = -np.ones(mask.shape, dtype=np.int64)
        rr, cc = np.nonzero(mask)
        self.rr, self.cc = rr, cc
        self.idx[rr, cc] = np.arange(len(rr))
        rows, cols, w = [], [], []
        h, wd = mask.shape
        for dr, dc, cost in ((0, 1, 1.0), (1, 0, 1.0), (1, 1, 1.4142), (1, -1, 1.4142)):
            r2, c2 = rr + dr, cc + dc
            ok = (r2 >= 0) & (r2 < h) & (c2 >= 0) & (c2 < wd)
            ok[ok] = mask[r2[ok], c2[ok]]
            a, b = self.idx[rr[ok], cc[ok]], self.idx[r2[ok], c2[ok]]
            rows += [a, b]
            cols += [b, a]
            w += [np.full(len(a), cost)] * 2
        n = len(rr)
        self.graph = coo_matrix((np.concatenate(w), (np.concatenate(rows), np.concatenate(cols))), shape=(n, n)).tocsr()
        self._src = None

    def run(self, cell: Tuple[int, int]) -> None:
        if self._src == cell:
            return
        self._src = cell
        self.dist, self.pred = dijkstra(self.graph, indices=int(self.idx[cell]), return_predecessors=True)

    def distance_m(self, cell: Tuple[int, int]) -> float:
        return float(self.dist[self.idx[cell]]) * self.res

    def path(self, cell: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
        i = int(self.idx[cell])
        if not np.isfinite(self.dist[i]):
            return None
        out = []
        while i >= 0:
            out.append((int(self.rr[i]), int(self.cc[i])))
            i = int(self.pred[i])
        return out[::-1]


def _line_free(mask: np.ndarray, a: Tuple[int, int], b: Tuple[int, int]) -> bool:
    n = int(max(abs(b[0] - a[0]), abs(b[1] - a[1]))) * 2 + 1
    rs = np.linspace(a[0], b[0], n).round().astype(int)
    cs = np.linspace(a[1], b[1], n).round().astype(int)
    return bool(mask[rs, cs].all())


def _simplify(mask: np.ndarray, path: Sequence[Tuple[int, int]]) -> List[Tuple[int, int]]:
    """Buang titik A* yang tidak perlu: sambung lurus selama tidak menabrak."""
    out = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not _line_free(mask, path[i], path[j]):
            j -= 1
        out.append(path[j])
        i = j
    return out


def _nearest_inside(mask: np.ndarray, cell: Tuple[int, int]) -> Tuple[int, int]:
    if 0 <= cell[0] < mask.shape[0] and 0 <= cell[1] < mask.shape[1] and mask[cell]:
        return cell
    rr, cc = np.nonzero(mask)
    i = int(np.argmin((rr - cell[0]) ** 2 + (cc - cell[1]) ** 2))
    return int(rr[i]), int(cc[i])


def plan_coverage(
    m: GridMap,
    start_xy: XY,
    spacing_m: float = 0.40,
    clearance_m: float = 0.30,
    min_row_len_m: float = 0.50,
    hole_fill_m: float = 0.15,
    min_obstacle_m2: float = 0.04,
    max_hole_m2: float = 0.5,
    boundary: Optional[Sequence[XY]] = None,
    boundary_extra_m: float = 0.0,
    row_end_extra_m: float = 0.0,
    bridge_gap_m: float = 0.0,
    done_mask: Optional[np.ndarray] = None,
    blocked_mask: Optional[np.ndarray] = None,
    home_xy: Optional[XY] = None,
) -> CoveragePlan:
    """done_mask: lantai yang SUDAH dipel (tidak dibuatkan baris lagi, tapi boleh
    dilewati). blocked_mask: halangan yang sedang terlihat (tidak boleh dilewati).
    home_xy: titik pulang (default = start_xy) -- dipakai saat rute dibuat ulang
    di tengah pengepelan."""
    global BRIDGE_GAP_M
    BRIDGE_GAP_M = bridge_gap_m
    mask = safe_area(m, clearance_m, hole_fill_m, start_xy, min_obstacle_m2, max_hole_m2, boundary, boundary_extra_m)
    if blocked_mask is not None:
        mask &= ~blocked_mask
    travel_mask = mask.copy()
    home = home_xy if home_xy is not None else start_xy
    notes: List[str] = []
    if not mask.any():
        return CoveragePlan([], 0.0, [], mask, notes=["Tidak ada area aman untuk dipel."])
    angle = _boundary_angle(boundary) if boundary is not None and len(boundary) >= 3 else dominant_angle(mask, m.res)
    # Blok dibentuk dari SEMUA potongan baris (termasuk yang pendek), supaya lorong
    # sempit tetap dikenali sebagai satu blok; potongan pendek baru dibuang setelah
    # arah terbaik tiap blok dipilih.
    rows = _row_segments(m, mask, angle, spacing_m, 0.05)

    # Jalur pindah antar segmen dihitung di area aman (tanpa tutup lubang), versi kasar.
    geo = Geodesic(travel_mask, m.res)

    def travel(a: XY, b: XY) -> Optional[List[XY]]:
        ca = _nearest_inside(travel_mask, m.to_cell(*a))
        cb = _nearest_inside(travel_mask, m.to_cell(*b))
        if _line_free(travel_mask, ca, cb):
            return [b]
        geo.run(ca)
        p = geo.path(cb)
        if p is None:
            return None
        return [m.to_xy(r, c) for r, c in _simplify(travel_mask, p)[1:-1]] + [b]

    angles = _candidate_angles(angle, boundary)
    cover_mask = mask & ~done_mask if done_mask is not None else mask
    cells = _greedy_cells(m, cover_mask, angles, spacing_m, min_row_len_m)
    rows = [sg for c in cells for sg in c]
    cells = [_straighten_ends(c, m, mask) for c in cells]
    if row_end_extra_m > 0:
        # Moncong robot menjorok ke depan: ujung baris dimundurkan dari dinding di depan.
        trimmed = []
        for c in cells:
            keep = []
            for sg in c:
                u0, u1 = sg.u0 + row_end_extra_m, sg.u1 - row_end_extra_m
                if u1 - u0 >= max(0.1, min_row_len_m - 2 * row_end_extra_m):
                    keep.append(Segment(sg.row, u0, u1, _point_at(sg, u0), _point_at(sg, u1), sg.v))
            if keep:
                trimmed.append(keep)
        cells = trimmed
    wps: List[Waypoint] = []
    cur = start_xy
    cov_len = trav_len = 0.0

    def go(to: XY, kind: str) -> bool:
        nonlocal cur, trav_len
        hop = travel(cur, to)
        if hop is None:
            return False
        prev = cur
        for p in hop:
            wps.append(Waypoint(p[0], p[1], kind))
            trav_len += math.hypot(p[0] - prev[0], p[1] - prev[1])
            prev = p
        cur = to
        return True

    remaining = list(cells)
    while remaining:
        # blok berikutnya = sudut masuk dengan JARAK JALAN terdekat (bukan jarak lurus,
        # supaya tidak memilih blok di balik dinding lalu memutar jauh)
        geo.run(_nearest_inside(travel_mask, m.to_cell(*cur)))
        best = None
        for ci, cell in enumerate(remaining):
            for rev in (False, True):
                first = cell[-1] if rev else cell[0]
                for flip in (False, True):
                    p = first.b if flip else first.a
                    d = geo.distance_m(_nearest_inside(travel_mask, m.to_cell(*p)))
                    if best is None or d < best[0]:
                        best = (d, ci, rev, flip)
        _, ci, rev, flip = best
        cell = remaining.pop(ci)
        order = cell[::-1] if rev else cell
        entered = False
        for i, seg in enumerate(order):
            a, b = (seg.b, seg.a) if flip else (seg.a, seg.b)
            kind = "turn" if entered else "travel"
            if not go(a, kind):
                notes.append(f"Baris di ({a[0]:.1f},{a[1]:.1f}) tidak terjangkau, dilewati.")
                continue
            entered = True
            wps.append(Waypoint(b[0], b[1], "row"))
            cov_len += math.hypot(b[0] - a[0], b[1] - a[1])
            cur = b
            flip = not flip  # zig-zag: baris berikutnya arah sebaliknya
    if not go(home, "home"):
        wps.append(Waypoint(home[0], home[1], "home"))
    return CoveragePlan(wps, angle, [(sg.a, sg.b) for sg in rows], mask, cov_len, trav_len, notes)


def _boundary_angle(boundary: Sequence[XY]) -> float:
    """Arah garis batas terpanjang yang digambar operator -> baris sejajar dinding."""
    pts = list(boundary) + [boundary[0]]
    best = max(zip(pts, pts[1:]), key=lambda ab: math.hypot(ab[1][0] - ab[0][0], ab[1][1] - ab[0][1]))
    (x0, y0), (x1, y1) = best
    return math.atan2(y1 - y0, x1 - x0)


def _candidate_angles(base: float, boundary: Optional[Sequence[XY]]) -> List[float]:
    """Arah lintasan yang dicoba. Sisi-sisi batas yang arahnya hampir sama (< 6 deg)
    dirata-ratakan (berbobot panjang sisi) jadi SATU arah: garis batas gambaran
    tangan tidak pernah persis sejajar, dan tanpa ini lintasan dalam satu lorong
    ikut miring berbeda-beda ("mencong"). Tiap arah + tegak lurusnya."""
    edges = []  # (sudut mod pi, panjang)
    if boundary is not None and len(boundary) >= 3:
        pts = list(boundary) + [boundary[0]]
        for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
            L = math.hypot(x1 - x0, y1 - y0)
            if L >= 0.8:
                edges.append((math.atan2(y1 - y0, x1 - x0) % math.pi, L))
    if not edges:
        edges = [(base % math.pi, 1.0)]
    # kelompokkan sudut (mod pi/2 supaya sisi tegak lurus juga sejajar dengan grid lorongnya)
    edges.sort(key=lambda e: -e[1])
    groups: List[List[Tuple[float, float]]] = []
    for a, L in edges:
        for g in groups:
            ref = g[0][0]
            d = (a - ref + math.pi / 4) % (math.pi / 2) - math.pi / 4
            if abs(d) < math.radians(6):
                g.append((ref + d, L))
                break
        else:
            groups.append([(a, L)])
    out: List[float] = []
    for g in groups:
        tot = sum(L for _, L in g)
        a = sum(x * L for x, L in g) / tot
        out += [a % math.pi, (a + math.pi / 2) % math.pi]
    return out[:10]

def _band_region(m: GridMap, mask: np.ndarray, angle: float, cell: List[Segment], spacing_m: float) -> np.ndarray:
    rr, cc = np.nonzero(mask)
    xs = m.ox + (cc + 0.5) * m.res
    ys = m.oy + (rr + 0.5) * m.res
    ca, sa = math.cos(angle), math.sin(angle)
    u = xs * ca + ys * sa
    v = -xs * sa + ys * ca
    sel = np.zeros(len(rr), dtype=bool)
    half = spacing_m / 2
    for sg in cell:
        sel |= (v >= sg.v - half) & (v < sg.v + half) & (u >= sg.u0 - m.res) & (u <= sg.u1 + m.res)
    region = np.zeros_like(mask)
    region[rr[sel], cc[sel]] = True
    return region


def _greedy_cells(
    m: GridMap, mask: np.ndarray, angles: List[float], spacing_m: float, min_len_m: float
) -> List[List[Segment]]:
    """Seperti orang mengepel ruangan berbentuk L: dari semua arah kandidat, ambil
    potongan area dengan lintasan lurus TERPANJANG (rata-rata panjang baris dikurangi
    biaya putar balik), tandai sudah dipel, ulangi untuk sisa area. Tiap lorong jadi
    mendapat lintasan lurus panjang sejajar dindingnya sendiri."""
    remaining = mask.copy()
    cells: List[List[Segment]] = []
    seg_len = lambda sg: sg.u1 - sg.u0  # noqa: E731
    for _ in range(60):
        if remaining.sum() * m.res * m.res < 0.05:
            break
        best = None
        for a in angles:
            fine = _row_segments(m, remaining, a, spacing_m, 0.05)
            for c in _decompose(fine, min_overlap_m=0.0):
                region = _band_region(m, remaining, a, c, spacing_m)
                segs = _row_segments(m, region, a, spacing_m, min_len_m, fit=True)
                if not segs:
                    continue
                total = sum(seg_len(sg) for sg in segs)
                key = (total / len(segs) - TURN_COST_M, total)  # baris panjang dulu, lalu yang luas
                if best is None or key > best[0]:
                    best, best_a = (key, segs, region), a
        if best is None:
            break
        _, segs, region = best
        cells.extend(_decompose(segs))
        # hanya pita lintasan yang DIPAKAI ditandai selesai; sisa potongan pendek
        # tetap boleh diambil arah lain (mis. lorong tegak di tikungan)
        used = _band_region(m, region, best_a, segs, spacing_m)
        remaining &= ~(used if used.any() else region)
    return cells


def _best_orientation(
    cell: List[Segment], m: GridMap, mask: np.ndarray, angle: float, spacing_m: float, min_len_m: float
) -> List[List[Segment]]:
    """Bandingkan dua arah untuk blok ini: arah dasar vs tegak lurus. Skor = panjang
    jalur pel dikurangi TURN_COST_M per baris (tiap baris = satu putar balik).
    Lorong sempit jadi satu-dua jalur lurus sepanjang lorong, bukan arsiran
    pendek-pendek melintang. Potongan < min_len_m dibuang setelah arah dipilih."""
    seg_len = lambda sg: sg.u1 - sg.u0  # noqa: E731
    score = lambda segs: sum(seg_len(sg) for sg in segs) - TURN_COST_M * len(segs)  # noqa: E731
    keep_unused = [sg for sg in cell if seg_len(sg) >= min_len_m]  # noqa: F841
    rr, cc = np.nonzero(mask)
    xs = m.ox + (cc + 0.5) * m.res
    ys = m.oy + (rr + 0.5) * m.res
    ca, sa = math.cos(angle), math.sin(angle)
    u = xs * ca + ys * sa
    v = -xs * sa + ys * ca
    # Wilayah blok TIDAK saling menimpa: tiap potongan baris memiliki pita
    # [v - s/2, v + s/2) -- pita-pita ini membagi habis area aman, jadi tidak ada
    # lantai yang mendapat lintasan dua kali dari blok tetangga.
    sel = np.zeros(len(rr), dtype=bool)
    half = spacing_m / 2
    for sg in cell:
        sel |= (v >= sg.v - half) & (v < sg.v + half) & (u >= sg.u0 - m.res) & (u <= sg.u1 + m.res)
    region = np.zeros_like(mask)
    region[rr[sel], cc[sel]] = True
    # Arah kandidat: arah dasar, tegak lurusnya, dan arah DINDING BLOK ITU SENDIRI
    # (kotak terkecil yang membungkus blok) -- lorong yang sedikit miring terhadap
    # arah dasar tetap mendapat lintasan lurus panjang sepanjang lorong.
    cands = [angle, angle + math.pi / 2]
    if region.sum() >= 20:
        local = dominant_angle(region, m.res)
        cands += [local, local + math.pi / 2]
    best, best_score = [], -float("inf")
    for a in cands:
        segs = _row_segments(m, region, a, spacing_m, min_len_m, fit=True)
        sc = score(segs) if segs else -float("inf")
        if sc > best_score + 0.05:  # utamakan urutan kandidat kalau skornya setara
            best, best_score = segs, sc
    return _decompose(best) if best else []


def _point_at(sg: Segment, u: float) -> XY:
    t = 0.0 if sg.u1 == sg.u0 else (u - sg.u0) / (sg.u1 - sg.u0)
    return (sg.a[0] + (sg.b[0] - sg.a[0]) * t, sg.a[1] + (sg.b[1] - sg.a[1]) * t)


def _straighten_ends(cell: List[Segment], m: GridMap, mask: np.ndarray) -> List[Segment]:
    """Ujung-ujung baris dalam satu blok disejajarkan ke satu garis lurus
    (regresi), supaya sambungan antar baris pendek dan rapi -- dinding peta
    yang bergerigi tidak membuat pola miring-miring. Ujung tidak pernah
    melewati batas aman (hanya dipendekkan, tidak diperpanjang)."""
    if len(cell) < 3:
        return cell
    idx = np.array([sg.row for sg in cell], dtype=float)
    out = [Segment(sg.row, sg.u0, sg.u1, sg.a, sg.b, sg.v) for sg in cell]
    for side in ("u0", "u1"):
        vals = np.array([getattr(sg, side) for sg in cell])
        k, c = np.polyfit(idx, vals, 1)
        fit = k * idx + c
        for sg, f in zip(out, fit):
            if side == "u0":
                sg.u0 = max(sg.u0, min(f, sg.u1 - 0.2))
            else:
                sg.u1 = min(sg.u1, max(f, sg.u0 + 0.2))
    for sg, orig in zip(out, cell):
        sg.a, sg.b = _point_at(orig, sg.u0), _point_at(orig, sg.u1)
    return out


TURN_COST_M = 1.0  # satu putar balik di ujung baris "setara" jalan 1 m
SPLIT_DIFF_M = 3.0  # beda panjang baris tetangga lebih dari ini = blok baru


def _decompose(rows: List[Segment], min_overlap_m: float = 0.2) -> List[List[Segment]]:
    """Boustrophedon cell decomposition: segmen baris berurutan yang saling
    bertumpuk satu-satu digabung jadi satu blok; blok dipel zig-zag sampai habis."""
    by_row: dict = {}
    for sg in rows:
        by_row.setdefault(sg.row, []).append(sg)
    overlap = lambda p, q: min(p.u1, q.u1) - max(p.u0, q.u0) > min_overlap_m  # noqa: E731
    cells: List[List[Segment]] = []
    owner: dict = {}  # id(segmen baris sebelumnya) -> index blok
    prev_row: List[Segment] = []
    for r in sorted(by_row):
        cur_row = sorted(by_row[r], key=lambda sg: sg.u0)
        new_owner = {}
        for sg in cur_row:
            ups = [p for p in prev_row if overlap(p, sg)]
            # Panjang baris yang beda jauh = bentuk area berubah (mis. lorong sempit
            # bertemu lorong lebar di tikungan) -> mulai blok baru, supaya tiap
            # bagian bisa memilih arah sapuannya sendiri.
            similar = len(ups) == 1 and abs((sg.u1 - sg.u0) - (ups[0].u1 - ups[0].u0)) <= SPLIT_DIFF_M
            if similar and sum(overlap(ups[0], q) for q in cur_row) == 1 and id(ups[0]) in owner:
                ci = owner[id(ups[0])]
                cells[ci].append(sg)
            else:
                cells.append([sg])
                ci = len(cells) - 1
            new_owner[id(sg)] = ci
        owner, prev_row = new_owner, cur_row
    return cells


# --------------------------------------------------------------------------
# Pola KONTUR: lintasan sejajar garis batas (permintaan operator 2026-10-09)
# --------------------------------------------------------------------------
def _rdp(pts: np.ndarray, eps: float) -> np.ndarray:
    """Douglas-Peucker: rapikan garis kontur jadi ruas-ruas lurus dengan sudut tegas."""
    if len(pts) < 3:
        return pts
    a, b = pts[0], pts[-1]
    ab = b - a
    n = np.hypot(*ab)
    if n < 1e-9:
        d = np.hypot(*(pts - a).T)
    else:
        d = np.abs(ab[0] * (pts[:, 1] - a[1]) - ab[1] * (pts[:, 0] - a[0])) / n
    i = int(np.argmax(d))
    if d[i] > eps:
        left = _rdp(pts[: i + 1], eps)
        right = _rdp(pts[i:], eps)
        return np.vstack([left[:-1], right])
    return np.vstack([a, b])


def plan_contour(
    m: GridMap,
    start_xy: XY,
    spacing_m: float = 0.40,
    clearance_m: float = 0.30,
    min_loop_len_m: float = 0.8,
    hole_fill_m: float = 0.15,
    min_obstacle_m2: float = 0.04,
    max_hole_m2: float = 0.5,
    boundary: Optional[Sequence[XY]] = None,
    boundary_extra_m: float = 0.0,
    home_xy: Optional[XY] = None,
    simplify_m: float = 0.10,
    smooth_m: float = 0.05,
    mop_half_m: float = 0.25,
) -> CoveragePlan:
    """Lintasan pertama menyusuri tepi area aman (garis batas/dinding, dengan jarak
    aman), lintasan berikutnya spacing_m lebih ke dalam dengan bentuk yang sama, dst.
    Garis biru selalu sejajar batas; sudutnya mengikuti sudut batas (robot berputar
    tepat di sudut). Antar lintasan pindah lewat jalur aman terdekat."""
    from contourpy import contour_generator

    mask = safe_area(m, clearance_m, hole_fill_m, start_xy, min_obstacle_m2, max_hole_m2, boundary, boundary_extra_m)
    home = home_xy if home_xy is not None else start_xy
    notes: List[str] = []
    if not mask.any():
        return CoveragePlan([], 0.0, [], mask, notes=["Tidak ada area aman untuk dipel."])
    # Tepi area dihaluskan dulu (tonjolan/gerigi dinding peta < smooth_m dipangkas,
    # hanya MENGECILKAN area -> tetap aman), lalu jarak tiap sel ke tepi: kontur =
    # garis sejajar tepi.
    k = max(1, int(round(smooth_m / m.res)))
    smooth = ndimage.binary_opening(mask, structure=_disk(k))
    dist = ndimage.distance_transform_edt(np.pad(smooth, 1))[1:-1, 1:-1] * m.res
    xs = m.ox + (np.arange(m.w) + 0.5) * m.res
    ys = m.oy + (np.arange(m.h) + 0.5) * m.res
    gen = contour_generator(x=xs, y=ys, z=dist, name="serial")
    loops: List[np.ndarray] = []
    level = m.res * 0.75
    while level < dist.max():
        for line in gen.lines(level):
            line = np.asarray(line, dtype=float)
            if len(line) < 3:
                continue
            seg = np.hypot(*np.diff(line, axis=0).T).sum()
            if seg < min_loop_len_m:
                continue
            loops.append(_rdp(line, simplify_m))
        level += spacing_m

    # Isi bagian tengah yang belum terkena pel (lorong lebih lebar dari 2 lintasan):
    # tambah lintasan sejajar tepi tepat di tengah bagian itu.
    def rasterize(lines_):
        hit = np.zeros_like(mask)
        for ln in lines_:
            for a, b in zip(ln, ln[1:]):
                n = int(np.hypot(*(b - a)) / (m.res / 2)) + 2
                px = np.linspace(a[0], b[0], n)
                py = np.linspace(a[1], b[1], n)
                r = ((py - m.oy) / m.res).astype(int)
                c = ((px - m.ox) / m.res).astype(int)
                ok = (r >= 0) & (r < m.h) & (c >= 0) & (c < m.w)
                hit[r[ok], c[ok]] = True
        return hit

    for _ in range(3):
        covered = ndimage.distance_transform_edt(~rasterize(loops)) * m.res <= mop_half_m
        uncov = mask & ~covered
        lab, n = ndimage.label(uncov)
        added = False
        for i in range(1, n + 1):
            comp = lab == i
            if comp.sum() * m.res * m.res < 0.08:
                continue
            lvl = max(m.res, float(dist[comp].max()) - 0.10)
            near = ndimage.binary_dilation(comp, structure=_disk(int(round(0.3 / m.res))))
            for line in gen.lines(lvl):
                line = np.asarray(line, dtype=float)
                if len(line) < 3:
                    continue
                r = ((line[:, 1] - m.oy) / m.res).astype(int)
                c = ((line[:, 0] - m.ox) / m.res).astype(int)
                ok = (r >= 0) & (r < m.h) & (c >= 0) & (c < m.w)
                if ok.sum() and near[r[ok], c[ok]].mean() >= 0.5 and np.hypot(*np.diff(line, axis=0).T).sum() >= 0.4:
                    loops.append(_rdp(line, simplify_m))
                    added = True
        if not added:
            break
    geo = Geodesic(mask, m.res)

    def travel(a: XY, b: XY) -> Optional[List[XY]]:
        ca = _nearest_inside(mask, m.to_cell(*a))
        cb = _nearest_inside(mask, m.to_cell(*b))
        if _line_free(mask, ca, cb):
            return [b]
        geo.run(ca)
        p = geo.path(cb)
        if p is None:
            return None
        return [m.to_xy(r, c) for r, c in _simplify(mask, p)[1:-1]] + [b]

    wps: List[Waypoint] = []
    cur = start_xy
    cov = trav = 0.0
    remaining = list(loops)
    while remaining:
        # lintasan berikutnya = titik masuk terdekat (jarak lurus)
        best = min(
            ((li, vi) for li, lp in enumerate(remaining) for vi in range(len(lp))),
            key=lambda t: math.hypot(remaining[t[0]][t[1]][0] - cur[0], remaining[t[0]][t[1]][1] - cur[1]),
        )
        lp = remaining.pop(best[0])
        closed = np.hypot(*(lp[0] - lp[-1])) < 2 * m.res
        if closed:  # mulai dari titik terdekat, keliling sampai kembali ke situ
            ring = lp[:-1]
            order = np.vstack([np.roll(ring, -best[1], axis=0), np.roll(ring, -best[1], axis=0)[:1]])
        else:  # garis terbuka: mulai dari ujung yang lebih dekat
            order = lp if best[1] <= len(lp) // 2 else lp[::-1]
        entry = (float(order[0][0]), float(order[0][1]))
        hop = travel(cur, entry)
        if hop is None:
            notes.append(f"Lintasan di ({entry[0]:.1f},{entry[1]:.1f}) tidak terjangkau, dilewati.")
            continue
        prev = cur
        for p in hop:
            wps.append(Waypoint(p[0], p[1], "travel" if len(hop) > 1 else "turn"))
            trav += math.hypot(p[0] - prev[0], p[1] - prev[1])
            prev = p
        for q in order[1:]:
            wps.append(Waypoint(float(q[0]), float(q[1]), "row"))
            cov += math.hypot(q[0] - prev[0], q[1] - prev[1])
            prev = (float(q[0]), float(q[1]))
        cur = prev
    hop = travel(cur, home) or [home]
    prev = cur
    for p in hop:
        wps.append(Waypoint(p[0], p[1], "home"))
        trav += math.hypot(p[0] - prev[0], p[1] - prev[1])
        prev = p
    rows = [((float(lp[0][0]), float(lp[0][1])), (float(lp[-1][0]), float(lp[-1][1]))) for lp in loops]
    return CoveragePlan(wps, 0.0, rows, mask, cov, trav, notes)


def plan_rings(
    m: GridMap,
    start_xy: XY,
    boundary: Sequence[XY],
    spacing_m: float = 0.40,
    first_offset_m: float = 0.47,
    clearance_m: float = 0.30,
    min_obstacle_m2: float = 0.04,
    max_hole_m2: float = 0.5,
    min_piece_m: float = 0.5,
    home_xy: Optional[XY] = None,
    simplify_m: float = 0.06,
) -> CoveragePlan:
    """Lintasan RAPI dari garis batas operator: garis batas digeser masuk
    first_offset_m, lalu tiap spacing_m lagi -> keliling-keliling bersarang yang
    sejajar persis dengan garis merah (jarak dihitung ke garis batas, bukan ke dinding
    peta yang bergerigi). Bagian yang mengenai dinding/perabot peta dipotong."""
    from contourpy import contour_generator

    inside = polygon_mask(m, boundary)
    safe = safe_area(m, clearance_m, 0.15, start_xy, min_obstacle_m2, max_hole_m2, boundary, 0.0)
    home = home_xy if home_xy is not None else start_xy
    if not safe.any():
        return CoveragePlan([], 0.0, [], safe, notes=["Tidak ada area aman untuk dipel."])
    dist = ndimage.distance_transform_edt(np.pad(inside, 1))[1:-1, 1:-1] * m.res
    xs = m.ox + (np.arange(m.w) + 0.5) * m.res
    ys = m.oy + (np.arange(m.h) + 0.5) * m.res
    gen = contour_generator(x=xs, y=ys, z=dist, name="serial")

    def pieces(line: np.ndarray) -> List[np.ndarray]:
        """Potong garis di bagian yang tidak aman (dinding/perabot peta)."""
        out, cur = [], []
        for a, b in zip(line, line[1:]):
            n = max(2, int(np.hypot(*(b - a)) / 0.05) + 1)
            for t in np.linspace(0, 1, n)[:-1]:
                p = a + (b - a) * t
                r, c = int((p[1] - m.oy) / m.res), int((p[0] - m.ox) / m.res)
                ok = 0 <= r < m.h and 0 <= c < m.w and safe[r, c]
                if ok:
                    cur.append(p)
                elif cur:
                    out.append(np.array(cur))
                    cur = []
        if len(line):
            cur.append(line[-1])
        if cur:
            out.append(np.array(cur))
        keep = []
        for pc in out:
            if len(pc) >= 2 and np.hypot(*np.diff(pc, axis=0).T).sum() >= min_piece_m:
                keep.append(_rdp(pc, simplify_m))
        return keep

    loops: List[np.ndarray] = []
    level = first_offset_m
    while level < dist.max():
        for line in gen.lines(level):
            line = np.asarray(line, dtype=float)
            if len(line) >= 3:
                loops.extend(pieces(line))
        level += spacing_m
    # sisa tengah yang cukup besar dan belum terkena pel -> satu lintasan di tengahnya
    hit = np.zeros_like(safe)
    for ln in loops:
        for a, b in zip(ln, ln[1:]):
            n = int(np.hypot(*(b - a)) / (m.res / 2)) + 2
            r = ((np.linspace(a[1], b[1], n) - m.oy) / m.res).astype(int)
            c = ((np.linspace(a[0], b[0], n) - m.ox) / m.res).astype(int)
            ok = (r >= 0) & (r < m.h) & (c >= 0) & (c < m.w)
            hit[r[ok], c[ok]] = True
    uncov = safe & ~(ndimage.distance_transform_edt(~hit) * m.res <= 0.25)
    lab, n = ndimage.label(uncov)
    for i in range(1, n + 1):
        comp = lab == i
        if comp.sum() * m.res * m.res < 0.3:
            continue
        lvl = max(first_offset_m, float(dist[comp].max()) - 0.05)
        near = ndimage.binary_dilation(comp, structure=_disk(int(round(0.3 / m.res))))
        for line in gen.lines(lvl):
            line = np.asarray(line, dtype=float)
            r = ((line[:, 1] - m.oy) / m.res).astype(int)
            c = ((line[:, 0] - m.ox) / m.res).astype(int)
            ok = (r >= 0) & (r < m.h) & (c >= 0) & (c < m.w)
            if len(line) >= 3 and ok.sum() and near[r[ok], c[ok]].mean() >= 0.5:
                loops.extend(pieces(line))
    sub = CoveragePlan([], 0.0, [], safe)
    return _order_loops(m, safe, loops, start_xy, home, sub)


def _order_loops(m, mask, loops, start_xy, home, plan):
    geo = Geodesic(mask, m.res)

    def travel(a, b):
        ca = _nearest_inside(mask, m.to_cell(*a))
        cb = _nearest_inside(mask, m.to_cell(*b))
        if _line_free(mask, ca, cb):
            return [b]
        geo.run(ca)
        p = geo.path(cb)
        if p is None:
            return None
        return [m.to_xy(r, c) for r, c in _simplify(mask, p)[1:-1]] + [b]

    wps, cur, cov, trav = [], start_xy, 0.0, 0.0
    remaining = list(loops)
    while remaining:
        best = min(
            ((li, vi) for li, lp in enumerate(remaining) for vi in range(len(lp))),
            key=lambda t: math.hypot(remaining[t[0]][t[1]][0] - cur[0], remaining[t[0]][t[1]][1] - cur[1]),
        )
        lp = remaining.pop(best[0])
        if np.hypot(*(lp[0] - lp[-1])) < 2 * m.res and len(lp) > 3:
            ring = lp[:-1]
            order = np.vstack([np.roll(ring, -best[1], axis=0), np.roll(ring, -best[1], axis=0)[:1]])
        else:
            order = lp if best[1] <= len(lp) // 2 else lp[::-1]
        entry = (float(order[0][0]), float(order[0][1]))
        hop = travel(cur, entry)
        if hop is None:
            plan.notes.append(f"Lintasan di ({entry[0]:.1f},{entry[1]:.1f}) tidak terjangkau, dilewati.")
            continue
        prev = cur
        for p in hop:
            wps.append(Waypoint(p[0], p[1], "travel" if len(hop) > 1 else "turn"))
            trav += math.hypot(p[0] - prev[0], p[1] - prev[1])
            prev = p
        for q in order[1:]:
            wps.append(Waypoint(float(q[0]), float(q[1]), "row"))
            cov += math.hypot(q[0] - prev[0], q[1] - prev[1])
            prev = (float(q[0]), float(q[1]))
        cur = prev
    hop = travel(cur, home) or [home]
    prev = cur
    for p in hop:
        wps.append(Waypoint(p[0], p[1], "home"))
        trav += math.hypot(p[0] - prev[0], p[1] - prev[1])
        prev = p
    plan.waypoints = wps
    plan.rows = [((float(lp[0][0]), float(lp[0][1])), (float(lp[-1][0]), float(lp[-1][1]))) for lp in loops]
    plan.coverage_length_m, plan.travel_length_m = cov, trav
    return plan
