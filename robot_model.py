
import math
from dataclasses import dataclass

import numpy as np

from kinematics import *  # noqa: F401,F403 - hằng số + FK/IK dùng chung
from kinematics import (BIN_DEG, DEG_PICK, P_APPROACH, P_LIFT, P_PICK, POSE_HOME, R_MAX, above_bin, arm_rz, at_bin,
                        ik_polar, leg_ms)

BINS = {
    1: {"name": "KHAY 1 - CHIN (DO)", "deg": BIN_DEG[1], "color": (0, 0, 255)},
    2: {"name": "KHAY 2 - VUA (VANG)", "deg": BIN_DEG[2], "color": (0, 215, 255)},
    3: {"name": "KHAY 3 - XANH", "deg": BIN_DEG[3], "color": (0, 255, 0)},
}
PAYLOAD_DROP_MM = 40.0  # Quả treo thấp hơn tâm kẹp khi đang kẹp


def reach_band(z, deg=90.0, step=0.5):
    """Khoảng bán kính [r_min, r_max] IK chấp nhận ở độ cao z (None nếu không có)."""
    ok_r = []
    r = 0.0
    while r <= R_MAX + 1e-6:
        if ik_polar(r, deg, z)[0]:
            ok_r.append(r)
        r += step
    return (ok_r[0], ok_r[-1]) if ok_r else None


# =========================================================================
# 1. CHU TRÌNH GẮP - THẢ (GIỐNG FSM FIRMWARE)
# =========================================================================
@dataclass
class Move:
    name: str
    a: tuple            # (q1, q2, q3) đầu đoạn
    b: tuple            # (q1, q2, q3) cuối đoạn
    carrying: bool
    pa: tuple = None    # (r, góc, z) đầu đoạn - chỉ có khi đi thẳng (moveArcLinear)
    pb: tuple = None
    ms: int = 1000


def _ik_or_raise(r, deg, z, label):
    ok, q, msg = ik_polar(r, deg, z)
    if not ok:
        raise ValueError(f"{label} (r={r:.0f}, goc={deg:.1f}, z={z:.0f}): {msg}")
    return q


def _linear(name, pa, pb, carrying, ms):
    return Move(name, _ik_or_raise(*pa, name), _ik_or_raise(*pb, name), carrying, pa, pb, ms)


Q_HOME_ROT = (DEG_PICK, POSE_HOME[1], POSE_HOME[2])  # HOME nhưng đế quay về góc gắp


def approach_moves():
    """HOME -> quay đế -> vươn tới trên điểm gắp -> hạ thẳng đứng -> kẹp -> nâng."""
    return [
        Move("XOAY DE TOI GAP", POSE_HOME, Q_HOME_ROT, False, ms=1500),
        Move("VUON TOI TREN GAP", Q_HOME_ROT, _ik_or_raise(*P_LIFT, "Tren diem gap"), False, ms=1000),
        _linear("HA NHANH", P_LIFT, P_APPROACH, False, 900),
        _linear("HA CHAM", P_APPROACH, P_PICK, False, 1200),
        _linear("NANG LEN", P_PICK, P_LIFT, True, 1200),
    ]


def transfer_moves(bin_id, waypoints=None):
    """Mang quả sang trên khay: cung R_PATH (2000ms) hoặc qua các điểm né."""
    if not waypoints:
        return [_linear("CUNG -> KHAY", P_LIFT, above_bin(bin_id), True, 2000)]
    pts = [P_LIFT] + [tuple(w) for w in waypoints] + [above_bin(bin_id)]
    moves = []
    for i in range(len(pts) - 1):
        name = f"NE {i + 1}" if i < len(waypoints) else "NE -> KHAY"
        moves.append(_linear(name, pts[i], pts[i + 1], True, leg_ms(pts[i], pts[i + 1])))
    return moves


def bin_moves(bin_id):
    p_above, p_bin = above_bin(bin_id), at_bin(bin_id)
    return [
        _linear("HA XUONG KHAY", p_above, p_bin, True, 800),
        _linear("NHAC KHOI KHAY", p_bin, p_above, False, 700),
    ]


def return_moves(bin_id, waypoints=None, direct_home=False):
    """Về HOME: nội suy khớp từ trên khay (1800ms) khi không có điểm né hoặc direct_home (PATH có "H");
    ngược lại đi ngược lại đúng các điểm né (đã kiểm tra lúc đi)."""
    if not waypoints or direct_home:
        return [Move("KHAY->HOME", _ik_or_raise(*above_bin(bin_id), "Tren khay"), POSE_HOME, False, ms=1800)]
    pts = [above_bin(bin_id)] + [tuple(w) for w in reversed(waypoints)] + [P_LIFT]
    moves = []
    for i in range(len(pts) - 1):
        name = f"VE NE {len(waypoints) - i}" if i < len(waypoints) else "VE TREN GAP"
        moves.append(_linear(name, pts[i], pts[i + 1], False, leg_ms(pts[i], pts[i + 1])))
    moves.append(Move("THU TAY LEN", _ik_or_raise(*P_LIFT, "Tren diem gap"), Q_HOME_ROT, False, ms=1000))
    moves.append(Move("XOAY VE HOME", Q_HOME_ROT, POSE_HOME, False, ms=1500))
    return moves


def cycle_moves(bin_id, waypoints=None, direct_home=False):
    return (approach_moves() + transfer_moves(bin_id, waypoints) + bin_moves(bin_id)
            + return_moves(bin_id, waypoints, direct_home))


def move_pose(move, s):
    """(q1, q2, q3) tại hệ số s của đoạn đi: đi thẳng -> nội suy (r, góc, z) rồi IK; đi theo khớp -> nội suy khớp."""
    if move.pa is not None:
        p = tuple(a + (b - a) * s for a, b in zip(move.pa, move.pb))
        return _ik_or_raise(*p, move.name)
    return tuple(a + (b - a) * s for a, b in zip(move.a, move.b))


def sample_move(move, n=30):
    return [move_pose(move, i / float(n)) for i in range(n + 1)]


def arm_joints(q):
    """4 điểm (x, y, z): chốt vai, chốt khuỷu, chốt cổ tay, tâm kẹp."""
    a1 = math.radians(q[0])
    c1, s1 = math.cos(a1), math.sin(a1)
    return [(r * c1, r * s1, z) for r, z in arm_rz(q[1], q[2])]


# =========================================================================
# 2. KIỂM TRA VA CHẠM (dùng chung: main, planners)
# =========================================================================
# Vật cản = trụ đứng (tâm x_mm, y_mm, bán kính r_mm, cao obs_h). Thân tay = 3 đoạn thẳng, + quả treo khi kẹp.
# Mỗi đoạn chỉ xét phần thấp hơn z_top = obs_h + z_margin, đo khoảng cách NGANG từ tâm trụ tới phần đó:
#   chạm  <=>  khoảng cách - r_mm < xy_margin
N_ARM_SEGS = 3


def pose_segments(joints, carrying=True):
    """joints (..., 4, 3) -> các đoạn thẳng (..., S, 2, 3): 3 khâu, + 1 đoạn quả treo nếu carrying."""
    j = np.asarray(joints, dtype=float)
    segs = np.stack([j[..., :-1, :], j[..., 1:, :]], axis=-2)
    if not carrying:
        return segs
    tip = j[..., -1:, :]
    drop = tip.copy()
    drop[..., 2] -= PAYLOAD_DROP_MM
    return np.concatenate([segs, np.stack([tip, drop], axis=-2)], axis=-3)


def obstacle_arrays(obstacles):
    """list vật cản (dict) -> (tâm (N, 2), bán kính (N,)); tuple đã đổi sẵn thì trả nguyên."""
    if isinstance(obstacles, tuple) and len(obstacles) == 2 and isinstance(obstacles[0], np.ndarray):
        return obstacles
    xy = np.array([(ob["x_mm"], ob["y_mm"]) for ob in obstacles], dtype=float).reshape(-1, 2)
    return xy, np.array([ob["r_mm"] for ob in obstacles], dtype=float)


def _seg_prep(segs, z_top):
    """(ax, ay, dx, dy, 1/|d|², t0, t1, có_phần_thấp): phần đoạn có z <= z_top ứng với t trong [t0, t1]."""
    segs = np.asarray(segs, dtype=float)
    a, b = segs[..., 0, :], segs[..., 1, :]
    za, zb = a[..., 2], b[..., 2]
    low_a, low_b = za <= z_top, zb <= z_top
    dz = zb - za
    tc = np.minimum(np.maximum((z_top - za) / np.where(dz != 0.0, dz, 1.0), 0.0), 1.0)
    ax, ay = a[..., 0], a[..., 1]
    dx, dy = b[..., 0] - ax, b[..., 1] - ay
    dd = dx * dx + dy * dy
    inv = np.where(dd > 1e-12, 1.0 / np.where(dd > 1e-12, dd, 1.0), 0.0)
    return ax, ay, dx, dy, inv, np.where(low_a, 0.0, tc), np.where(low_b, 1.0, tc), low_a | low_b


def _seg_t(p, cx, cy):
    ax, ay, dx, dy, inv, t0, t1 = p[:7]
    return np.minimum(np.maximum(((cx - ax) * dx + (cy - ay) * dy) * inv, t0), t1)


def _seg_clear(p, cx, cy, r):
    ax, ay, dx, dy = p[:4]
    t = _seg_t(p, cx, cy)
    ex = cx - ax - t * dx
    ey = cy - ay - t * dy
    return np.sqrt(ex * ex + ey * ey) - r


def segment_clearance(segs, obstacles, z_top):
    """segs (..., 2, 3) -> (..., N): khoảng hở ngang tới N vật cản (+inf nếu cả đoạn cao hơn z_top)."""
    c, rad = obstacle_arrays(obstacles)
    p = [v[..., None] for v in _seg_prep(segs, z_top)]
    clear = _seg_clear(p, c[:, 0], c[:, 1], rad)
    return np.where(p[7], clear, np.inf)


def segment_closest_xy(seg, z_top, cx, cy):
    """Điểm (x, y) trên đoạn gần tâm vật nhất (để vẽ chỗ chạm)."""
    p = _seg_prep(seg, z_top)
    t = float(_seg_t(p, cx, cy))
    return float(p[0] + t * p[2]), float(p[1] + t * p[3])


def segments_hit(segs, obstacles, obs_h, xy_margin, z_margin):
    """segs (..., 2, 3) -> (...) bool: đoạn nào chạm vật cản (bỏ qua sớm các cặp ở xa nhau)."""
    c, rad = obstacle_arrays(obstacles)
    segs = np.asarray(segs, dtype=float)
    shape = segs.shape[:-2]
    flat = segs.reshape(-1, 2, 3)
    p = _seg_prep(flat, obs_h + z_margin)
    lo_x, hi_x = np.minimum(flat[:, 0, 0], flat[:, 1, 0]), np.maximum(flat[:, 0, 0], flat[:, 1, 0])
    lo_y, hi_y = np.minimum(flat[:, 0, 1], flat[:, 1, 1]), np.maximum(flat[:, 0, 1], flat[:, 1, 1])
    hit = np.zeros(len(flat), dtype=bool)
    todo = p[7].copy()
    for (cx, cy), r in zip(c, rad):
        reach = r + xy_margin + 1e-6
        idx = np.flatnonzero(todo & (lo_x < cx + reach) & (hi_x > cx - reach) & (lo_y < cy + reach) & (hi_y > cy - reach))
        if len(idx):
            near = _seg_clear([v[idx] for v in p[:7]], cx, cy, r) < xy_margin
            hit[idx[near]] = True
            todo[idx[near]] = False
    return hit.reshape(shape)


class SampledMove(list):
    """Mẫu dọc 1 đoạn đi [(q, khớp), ...] kèm segs (n, 4, 2, 3) tính sẵn 1 lần."""

    def __init__(self, samples=()):
        super().__init__(samples)
        joints = [pts for _q, pts in self]
        self.segs = pose_segments(joints, True) if joints else np.zeros((0, N_ARM_SEGS + 1, 2, 3))


def move_segments(mv, samples):
    segs = getattr(samples, "segs", None)
    if segs is None:
        samples = SampledMove(samples)
        segs = samples.segs
    return segs if mv.carrying else segs[:, :N_ARM_SEGS]


def sample_moves(moves, n=30):
    return [(mv, SampledMove((q, arm_joints(q)) for q in sample_move(mv, n))) for mv in moves]


def check_collision(sampled_cycle, obstacles, obs_height_mm, xy_margin_mm=22.0, z_margin_mm=15.0):
    """Từng đoạn đi: dict(move, hit, min_clear_mm, hit_xy, tip_path)."""
    obs = obstacle_arrays(obstacles)
    z_top = obs_height_mm + z_margin_mm
    results = []
    for mv, samples in sampled_cycle:
        hit, hit_xy, min_clear = False, None, 9999.0
        tip_path = [pts[-1] for _q, pts in samples]
        if len(obs[1]) and len(samples):
            segs = move_segments(mv, samples)
            clear = segment_clearance(segs, obs, z_top)  # (mẫu, đoạn, vật)
            low = np.isfinite(clear)
            if low.any():
                min_clear = min(min_clear, float(clear[low].min()))
            bad = np.argwhere(clear < xy_margin_mm)
            if len(bad):
                i, k, j = bad[0]
                hit, hit_xy = True, segment_closest_xy(segs[i, k], z_top, obs[0][j, 0], obs[0][j, 1])
        results.append({"move": mv, "hit": hit, "min_clear_mm": min_clear, "hit_xy": hit_xy, "tip_path": tip_path})
    return results
