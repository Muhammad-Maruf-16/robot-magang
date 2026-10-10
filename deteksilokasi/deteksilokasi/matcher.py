"""Cocokkan satu scan LiDAR dengan peta untuk menemukan posisi robot.

Tidak bergantung pada ROS, jadi bisa diuji tanpa robot (lihat test/).

Cara kerja:
  1. Dari peta dibuat "likelihood field": tiap sel menyimpan seberapa dekat
     sel itu ke dinding (1.0 = tepat di dinding, makin jauh makin kecil).
  2. Untuk satu pose kandidat (x, y, arah), ujung-ujung sinar LiDAR
     diproyeksikan ke peta. Skor = rata-rata nilai likelihood di ujung sinar.
     Scan yang pas dengan dinding di peta -> skor mendekati 1.
  3. Global search: semua posisi bebas di peta (jarak 15 cm) x semua arah
     (tiap 5 derajat), lalu kandidat terbaik dihaluskan dengan pencarian lokal.
"""

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np
from scipy.ndimage import distance_transform_edt


@dataclass
class Candidate:
    x: float
    y: float
    yaw: float
    score: float


class MapMatcher:
    def __init__(
        self,
        grid: np.ndarray,
        resolution: float,
        origin_x: float,
        origin_y: float,
        sigma_m: float = 0.10,
        robot_clearance_m: float = 0.20,
    ) -> None:
        """grid: (tinggi, lebar) int, nilai OccupancyGrid ROS
        (-1 tidak diketahui, 0 bebas, 100 terisi), baris 0 = y terkecil."""
        self.res = float(resolution)
        self.ox = float(origin_x)
        self.oy = float(origin_y)
        self.h, self.w = grid.shape
        occupied = grid >= 65
        free = grid == 0
        dist_m = distance_transform_edt(~occupied) * self.res
        self.lik = np.exp(-(dist_m ** 2) / (2.0 * sigma_m ** 2)).astype(np.float32)
        # Titik tengah robot hanya mungkin di sel bebas yang tidak menempel dinding.
        self.candidate_mask = free & (dist_m >= robot_clearance_m)

    # ------------------------------------------------------------------
    def _lookup(self, wx: np.ndarray, wy: np.ndarray) -> np.ndarray:
        cx = np.floor((wx - self.ox) / self.res).astype(np.int64)
        cy = np.floor((wy - self.oy) / self.res).astype(np.int64)
        inside = (cx >= 0) & (cx < self.w) & (cy >= 0) & (cy < self.h)
        out = np.zeros(wx.shape, dtype=np.float32)
        out[inside] = self.lik[cy[inside], cx[inside]]
        return out

    def score_poses(self, xs: np.ndarray, ys: np.ndarray, yaw: float, beams: np.ndarray) -> np.ndarray:
        """Skor banyak posisi dengan SATU arah. beams: (B, 2) ujung sinar di
        frame robot (x depan, y kiri)."""
        c, s = math.cos(yaw), math.sin(yaw)
        bx = beams[:, 0] * c - beams[:, 1] * s
        by = beams[:, 0] * s + beams[:, 1] * c
        wx = xs[:, None] + bx[None, :]
        wy = ys[:, None] + by[None, :]
        return self._lookup(wx, wy).mean(axis=1)

    def score_pose(self, x: float, y: float, yaw: float, beams: np.ndarray) -> float:
        return float(self.score_poses(np.array([x]), np.array([y]), yaw, beams)[0])

    # ------------------------------------------------------------------
    def global_search(
        self,
        beams: np.ndarray,
        stride_m: float = 0.15,
        angle_step_deg: float = 5.0,
        top_k: int = 15,
    ) -> List[Candidate]:
        """Cari di seluruh peta. Hasil: kandidat terbaik dulu, sudah dihaluskan,
        dan saling berjauhan (>= 1 m atau beda arah >= 30 derajat)."""
        step = max(1, int(round(stride_m / self.res)))
        mask = np.zeros_like(self.candidate_mask)
        mask[::step, ::step] = self.candidate_mask[::step, ::step]
        rows, cols = np.nonzero(mask)
        xs = self.ox + (cols + 0.5) * self.res
        ys = self.oy + (rows + 0.5) * self.res

        coarse: List[Candidate] = []
        for yaw in np.deg2rad(np.arange(-180.0, 180.0, angle_step_deg)):
            scores = self.score_poses(xs, ys, float(yaw), beams)
            k = min(top_k, len(scores))
            idx = np.argpartition(-scores, k - 1)[:k]
            coarse.extend(Candidate(float(xs[i]), float(ys[i]), float(yaw), float(scores[i])) for i in idx)
        coarse.sort(key=lambda c: -c.score)

        refined = [self.refine(c, beams) for c in self._distinct(coarse, top_k)]
        refined.sort(key=lambda c: -c.score)
        return self._distinct(refined, top_k)

    def refine(self, start: Candidate, beams: np.ndarray) -> Candidate:
        """Pencarian lokal di sekitar satu kandidat, dua tahap (kasar lalu halus)."""
        best = start
        for span_m, step_m, span_deg, step_deg in ((0.15, 0.05, 6.0, 2.0), (0.05, 0.025, 2.0, 0.5)):
            offs = np.arange(-span_m, span_m + 1e-9, step_m)
            gx, gy = np.meshgrid(best.x + offs, best.y + offs)
            gx, gy = gx.ravel(), gy.ravel()
            for dyaw in np.deg2rad(np.arange(-span_deg, span_deg + 1e-9, step_deg)):
                yaw = best.yaw + float(dyaw)
                scores = self.score_poses(gx, gy, yaw, beams)
                i = int(np.argmax(scores))
                if scores[i] > best.score:
                    best = Candidate(float(gx[i]), float(gy[i]), _wrap(yaw), float(scores[i]))
        return best

    @staticmethod
    def _distinct(cands: Sequence[Candidate], limit: int) -> List[Candidate]:
        out: List[Candidate] = []
        for c in cands:
            if all(
                math.hypot(c.x - o.x, c.y - o.y) >= 1.0 or abs(_wrap(c.yaw - o.yaw)) >= math.radians(30)
                for o in out
            ):
                out.append(c)
                if len(out) >= limit:
                    break
        return out


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def beams_from_scan(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    laser_x: float = 0.0,
    laser_y: float = 0.0,
    laser_yaw: float = 0.0,
    max_beams: int = 120,
    max_range_m: float = 10.0,
) -> Optional[np.ndarray]:
    """Ubah LaserScan jadi titik-titik (B, 2) di frame robot, diambil merata
    paling banyak max_beams. None kalau sinar valid terlalu sedikit."""
    r = np.asarray(ranges, dtype=np.float64)
    a = angle_min + angle_increment * np.arange(len(r))
    ok = np.isfinite(r) & (r > range_min) & (r < min(range_max, max_range_m))
    r, a = r[ok], a[ok]
    if len(r) < 20:
        return None
    if len(r) > max_beams:
        pick = np.linspace(0, len(r) - 1, max_beams).astype(int)
        r, a = r[pick], a[pick]
    a = a + laser_yaw
    return np.stack([laser_x + r * np.cos(a), laser_y + r * np.sin(a)], axis=1)
