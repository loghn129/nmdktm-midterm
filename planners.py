"""
planners.py - HAI THUẬT TOÁN NÉ VẬT CẢN: "CO DINH" (plan_fixed) và A* (plan_astar)
Đầu vào: khay đích + vật cản (trụ đứng x_mm, y_mm, r_mm) + chiều cao giả định. Đầu ra: Plan CLEAR / PATH / BLOCKED.
Bước chung (_precheck):
  0. Đoạn KHÔNG né được (hạ xuống gắp / hạ xuống khay) chạm vật -> BLOCKED.
  1. Đường mặc định (cung R_PATH ở Z_TRAVEL) an toàn -> CLEAR.
  Có đường né thì kiểm tra thêm đường về thẳng (trên khay -> HOME): không chạm vật -> gửi kèm "H", về thẳng HOME;
  ngược lại lúc về robot đi ngược lại đúng các điểm né.
CO DINH:
  2. Thử 2 điểm né (r, góc gắp, z) -> (r, góc khay, z) theo chi phí |r - R_PATH| + Z_COST*(z - Z_TRAVEL) tăng dần
     (đi vòng ngoài / vòng trong / nâng cao). Hết phương án -> BLOCKED.
  Mọi đường đi được lấy mẫu sẵn (warm_up), mỗi lần hỏi chỉ đo khoảng cách bằng numpy -> vài ms.
A*: lưới (góc đế × bán kính r) ở độ cao z cố định, bước ~2° × 10mm, mỗi nút nối 8 nút quanh nó.
  Nút ảo S = trên điểm gắp (nối MỌI nút cột góc gắp), G = trên khay (nối từ MỌI nút cột góc khay).
  Chi phí cạnh g = độ dài đường tâm kẹp sqrt(dr² + (r_tb*dgóc)² + dz²) (giống kinematics.leg_ms()).
  Heuristic h = sqrt(dr² + (r_nhỏ_nhất*dgóc)² + dz²) tới G, không bao giờ lớn hơn chi phí thật -> đường ngắn nhất.
  2. Với từng z (thấp trước): đánh dấu nút / cạnh chạm vật -> A* từ S tới G -> kéo dây (bỏ điểm giữa nếu đi
     thẳng vẫn an toàn, <= 6 điểm né) -> làm tròn như câu PATH -> kiểm tra lại cả chu trình -> PATH.
  3. Mọi z đều không có đường -> BLOCKED.
Tính quá DEADLINE_MS -> BLOCKED (trả lời muộn thì ESP32 tự đi đường mặc định).
"""

import functools
import heapq
import math
import time
from dataclasses import dataclass

import numpy as np

import esp32_link as proto
import robot_model as rm

Z_LEVELS = tuple(rm.Z_TRAVEL + dz for dz in (0.0, 20.0, 40.0, 60.0, 80.0))
R_STEP_MM = 10.0
N_SAMPLES = 30    # Số mẫu mỗi đoạn khi kiểm tra va chạm
DEADLINE_MS = 0.6 * proto.PC_TIMEOUT_MS
Z_COST = 1.5      # CO DINH: nâng 1mm "đắt" hơn đi vòng 1mm
CHUNK = 8         # CO DINH: số phương án đo cùng lúc
DEG_STEP = 2.0    # A*: bước góc của lưới
LAZY_TRIES = 20   # A*: số lần chạy lại sau khi loại cạnh lưới bị chạm lúc kiểm tra dày
NEIGHBORS = ((1, 0), (0, 1), (1, 1), (1, -1))  # A*: cạnh vô hướng
START, GOAL = -1, -2


@dataclass
class Plan:
    kind: str          # "CLEAR" / "PATH" / "BLOCKED"
    reply: str         # Chuỗi gửi cho ESP32
    waypoints: list    # [(r, góc, z), ...]
    reason: str
    moves: list        # Các đoạn của chu trình sẽ chạy
    results: list      # check_collision() các đoạn trên (để vẽ)
    tried: int         # CO DINH: số phương án đã thử | A*: số nút đã mở rộng
    ms: float
    algo: str = "CO DINH"
    direct_home: bool = False  # PATH có "H": về thẳng HOME sau khi thả quả


# ============================================================
# 1. PHẦN CHUNG: lấy mẫu sẵn, kiểm tra đoạn cố định / mặc định / về thẳng
# ============================================================
@functools.lru_cache(maxsize=1)
def detour_r_max():
    """Bán kính đi vòng ngoài xa nhất mà camera còn thấy vật cản: detour_r_max_mm, hoặc
    scan_r_max_mm - safe_margin_mm, không quá phần cung quét nằm trọn trong khung hình."""
    try:
        import vision_utils as vu
        cfg = vu.load_config()
        visible = vu.visible_r_max(cfg)
    except Exception as e:  # noqa: BLE001
        print(f"[PLANNER] Khong doc duoc calib_config.json ({e!r}) -> di vong toi da R_MAX")
        return float(rm.R_MAX)
    if cfg.get("detour_r_max_mm") is not None:
        cap = float(cfg["detour_r_max_mm"])
    else:
        cap = float(cfg.get("scan_r_max_mm", rm.R_MAX)) - float(cfg.get("safe_margin_mm", 22.0))
    cap = min(max(min(cap, float(visible)), rm.R_PATH), rm.R_MAX)
    print(f"[PLANNER] Di vong ngoai toi da r = {cap:.0f}mm")
    return cap


@functools.lru_cache(maxsize=1)
def candidates():
    """CO DINH: (r, z) sắp theo chi phí tăng dần."""
    out = []
    r_cap = detour_r_max()
    for z in Z_LEVELS:
        band = rm.reach_band(z)
        if band is None:
            continue
        r_lo = math.ceil(band[0] / R_STEP_MM) * R_STEP_MM
        r_hi = math.floor(min(band[1], r_cap) / R_STEP_MM) * R_STEP_MM
        rs = {rm.R_PATH} if band[0] <= rm.R_PATH <= band[1] else set()
        r = r_lo
        while r <= r_hi + 1e-6:
            rs.add(r)
            r += R_STEP_MM
        for r in rs:
            out.append((abs(r - rm.R_PATH) + Z_COST * (z - rm.Z_TRAVEL), r, z))
    out.sort()
    return [(r, z) for _cost, r, z in out]


def detour_waypoints(bin_id, r, z):
    """CO DINH: điểm né của phương án (r, z), làm tròn đúng như câu PATH gửi đi."""
    if r == rm.R_PATH and z == rm.Z_TRAVEL:
        wps = [rm.above_bin(bin_id)]  # Đúng cung mặc định, chỉ đổi đường về
    else:
        wps = [(r, rm.DEG_PICK, z), (r, rm.BINS[bin_id]["deg"], z)]
    return proto.round_trip(wps)


@functools.lru_cache(maxsize=None)
def _sampled(key, bin_id, r=None, z=None, direct_home=False):
    """Lấy mẫu sẵn (IK là phần tốn thời gian nhất và không phụ thuộc vật cản). None nếu IK không tới."""
    try:
        if key == "fixed":
            moves = rm.approach_moves() + rm.bin_moves(bin_id)
        elif key == "default":
            moves = rm.transfer_moves(bin_id) + rm.return_moves(bin_id)
        elif key == "home":  # Về thẳng: trên khay -> HOME (nội suy khớp, giống CLEAR)
            moves = rm.return_moves(bin_id)
        elif key == "cycle":
            moves = rm.cycle_moves(bin_id, detour_waypoints(bin_id, r, z) if r is not None else None, direct_home)
        else:  # "detour"
            moves = rm.transfer_moves(bin_id, detour_waypoints(bin_id, r, z))
        return moves, rm.sample_moves(moves, N_SAMPLES)
    except ValueError:
        if key == "detour":
            return None  # Có điểm trên đường mà IK không tới được
        raise


@functools.lru_cache(maxsize=None)
def _moves_pack(key, bin_id):
    """Các đoạn "fixed" / "default" / "home" gộp thành 1 mảng: (tên đoạn, segs, chỉ số đầu từng đoạn)."""
    names, parts, starts, n = [], [], [], 0
    for mv, s in _sampled(key, bin_id)[1]:
        segs = rm.move_segments(mv, s).reshape(-1, 2, 3)
        names.append(mv.name)
        parts.append(segs)
        starts.append(n)
        n += len(segs)
    return names, np.concatenate(parts), np.array(starts)


def _hits_key(key, bin_id, args):
    """Tên các đoạn của key chạm vật. args = (vật cản, cao vật, lề ngang, lề đứng)."""
    names, segs, starts = _moves_pack(key, bin_id)
    per = np.logical_or.reduceat(rm.segments_hit(segs, *args), starts)
    return [names[i] for i in np.flatnonzero(per)]


def direct_home_ok(bin_id, args):
    """Đường về thẳng (trên khay -> HOME) không chạm vật -> PATH được gửi kèm "H"."""
    return not _hits_key("home", bin_id, args)


def _make_plan(algo, t0, args, bin_id, kind, waypoints, reason, tried, cycle=None, results=None, direct=False):
    moves, sampled = cycle or _sampled("cycle", bin_id)
    reply = proto.format_path(waypoints, direct) if kind == "PATH" else kind
    if results is None:
        results = rm.check_collision(sampled, *args)
    return Plan(kind, reply, list(waypoints or []), reason, moves, results, tried,
                (time.perf_counter() - t0) * 1000.0, algo, direct)


def _precheck(bin_id, args, finish):
    """Bước 0, 1 chung: trả Plan CLEAR / BLOCKED, hoặc None nếu phải tìm đường né."""
    if not len(args[0][1]):
        return finish("CLEAR", None, "Khong co vat can", 0)
    fixed_hits = _hits_key("fixed", bin_id, args)
    if fixed_hits:
        return finish("BLOCKED", None, f"Vat can o doan khong ne duoc: {', '.join(fixed_hits)}", 0)
    if not _hits_key("default", bin_id, args):
        return finish("CLEAR", None, "Duong mac dinh an toan", 1)
    return None


def warm_up():
    """Tính sẵn mọi phương án CO DINH + lưới A* (vài giây) để lần REQ đầu cũng nhanh."""
    for b in rm.BINS:
        _sampled("cycle", b)
        for key in ("fixed", "default", "home"):
            _moves_pack(key, b)
        for r, z in candidates():
            if _sampled("detour", b, r, z) is not None:
                _sampled("cycle", b, r, z)
        _detour_pack(b)
    for z in Z_LEVELS:
        grid_at(z)


def plan(algo, bin_id, obstacles, obs_height_mm, xy_margin_mm=22.0, z_margin_mm=15.0, deadline_ms=DEADLINE_MS):
    """algo = "co_dinh" | "astar"."""
    fn = plan_astar if algo == "astar" else plan_fixed
    return fn(bin_id, obstacles, obs_height_mm, xy_margin_mm, z_margin_mm, deadline_ms)


# ============================================================
# 2. CO DINH: thử lần lượt các điểm né (r, z)
# ============================================================
@functools.lru_cache(maxsize=None)
def _detour_pack(bin_id):
    """Mọi phương án điểm né của 1 khay gộp thành 1 mảng numpy: ([(r, z)], segs, chỉ số mẫu đầu từng phương án)."""
    rz, parts, starts, n = [], [], [], 0
    for r, z in candidates():
        detour = _sampled("detour", bin_id, r, z)
        if detour is None:
            continue
        segs = np.concatenate([rm.move_segments(mv, s) for mv, s in detour[1]])
        rz.append((r, z))
        parts.append(segs)
        starts.append(n)
        n += len(segs)
    big = np.concatenate(parts) if parts else np.zeros((0, rm.N_ARM_SEGS + 1, 2, 3))
    return rz, big, np.array(starts + [n])


def _first_free(bin_id, args, t0, deadline_ms):
    """(chỉ số phương án đầu tiên không chạm hoặc None, số đã thử, hết giờ?)."""
    rz, big, starts = _detour_pack(bin_id)
    for i in range(0, len(rz), CHUNK):
        if (time.perf_counter() - t0) * 1000.0 > deadline_ms:
            return None, i, True
        j = min(i + CHUNK, len(rz))
        lo = starts[i]
        hit = rm.segments_hit(big[lo:starts[j]], *args).any(axis=-1)
        free = np.flatnonzero(~np.logical_or.reduceat(hit, starts[i:j] - lo))
        if len(free):
            return i + int(free[0]), i + int(free[0]) + 1, False
    return None, len(rz), False


def plan_fixed(bin_id, obstacles, obs_height_mm, xy_margin_mm=22.0, z_margin_mm=15.0, deadline_ms=DEADLINE_MS):
    t0 = time.perf_counter()
    args = (rm.obstacle_arrays(obstacles), obs_height_mm, xy_margin_mm, z_margin_mm)

    def finish(kind, rz, reason, tried, direct=False):
        cycle = _sampled("cycle", bin_id, *rz, direct) if rz else None
        waypoints = detour_waypoints(bin_id, *rz) if rz else None
        return _make_plan("CO DINH", t0, args, bin_id, kind, waypoints, reason, tried, cycle, direct=direct)

    early = _precheck(bin_id, args, finish)
    if early is not None:
        return early

    k, n_tried, late = _first_free(bin_id, args, t0, deadline_ms)
    tried = 1 + n_tried
    if k is not None:
        r, z = _detour_pack(bin_id)[0][k]
        how = []
        if r != rm.R_PATH:
            how.append(f"{'vong ngoai' if r > rm.R_PATH else 'vong trong'} r={r:.0f}")
        if z != rm.Z_TRAVEL:
            how.append(f"nang z={z:.0f}")
        direct = direct_home_ok(bin_id, args)
        if direct:
            how.append("ve thang HOME")
        return finish("PATH", (r, z), "Ne: " + (", ".join(how) if how else "ve theo cung"), tried, direct)
    if late:
        return finish("BLOCKED", None, f"Tinh qua {deadline_ms:.0f}ms (thu {tried} phuong an) -> BLOCKED cho an toan",
                      tried)
    return finish("BLOCKED", None, f"Thu {tried} phuong an, khong co duong an toan", tried)


# ============================================================
# 3. A*: tìm đường ngắn nhất trên lưới (góc × r) ở từng độ cao z
# ============================================================
def leg_len(pa, pb):
    """Độ dài đường tâm kẹp đi thẳng pa -> pb trong (r, góc, z)."""
    arc = (pa[0] + pb[0]) / 2.0 * math.radians(abs(pb[1] - pa[1]))
    return math.sqrt((pb[0] - pa[0]) ** 2 + arc ** 2 + (pb[2] - pa[2]) ** 2)


def path_len(bin_id, waypoints):
    pts = [rm.P_LIFT] + [tuple(w) for w in waypoints] + [rm.above_bin(bin_id)]
    return sum(leg_len(a, b) for a, b in zip(pts[:-1], pts[1:]))


def _pose_points(q):
    """Đoạn thân tay + quả treo (đường né luôn đang mang quả): (S, 2, 3)."""
    return rm.pose_segments(rm.arm_joints(q), carrying=True)


POSE_SHAPE = _pose_points(rm.POSE_HOME).shape


def _hit_mask(segs, obstacles, obs_h, xy_margin, z_margin):
    """segs (M, ..., 2, 3) -> (M,) True nếu chạm vật cản."""
    if segs.shape[0] == 0:
        return np.zeros(0, dtype=bool)
    hit = rm.segments_hit(segs, obstacles, obs_h, xy_margin, z_margin)
    return hit.reshape(segs.shape[0], -1).any(axis=1)


def sample_segment(pa, pb):
    """Đoạn thân tay dọc đường thẳng pa -> pb, đúng N_SAMPLES mẫu như CO DINH. None nếu IK không tới."""
    joints = []
    for k in range(N_SAMPLES + 1):
        s = k / float(N_SAMPLES)
        good, q, _ = rm.ik_polar(*(a + (b - a) * s for a, b in zip(pa, pb)))
        if not good:
            return None
        joints.append(rm.arm_joints(q))
    return rm.pose_segments(joints, carrying=True)


def segment_free(pa, pb, args):
    pts = sample_segment(pa, pb)
    return pts is not None and not _hit_mask(pts, *args).any()


class Links:
    """Các đoạn nối nút ảo (S hoặc G) với 1 cột của lưới, lấy mẫu sẵn."""

    def __init__(self, ids, pts, costs):
        self.ids, self.costs = ids, costs
        seg = POSE_SHAPE[1:]
        self.pts = np.array(pts).reshape((len(ids), -1) + seg) if ids else np.zeros((0, 1) + seg)

    def free(self, args):
        """{nút: chi phí} các đoạn nối KHÔNG chạm vật."""
        hit = _hit_mask(self.pts, *args)
        return {n: c for n, c, h in zip(self.ids, self.costs, hit) if not h}


class Grid:
    """Lưới ở độ cao z: thân tay của mọi nút, trung điểm mọi cạnh, đoạn nối nút ảo - tính sẵn 1 lần."""

    def __init__(self, z):
        self.z = z
        self.ok = False
        band = rm.reach_band(z)
        if band is None:
            return
        k_lo = math.ceil((band[0] - rm.R_PATH) / R_STEP_MM)
        k_hi = math.floor((min(band[1], detour_r_max()) - rm.R_PATH) / R_STEP_MM)
        self.rs = [rm.R_PATH + k * R_STEP_MM for k in range(k_lo, k_hi + 1)]
        n = int(round((rm.BASE_DEG_MAX - rm.BASE_DEG_MIN) / DEG_STEP))
        degs = {round(float(d), 6) for d in np.linspace(rm.BASE_DEG_MIN, rm.BASE_DEG_MAX, n + 1)}
        degs |= {round(rm.DEG_PICK, 6)} | {round(b["deg"], 6) for b in rm.BINS.values()}
        self.degs = sorted(degs)
        nd, nr = len(self.degs), len(self.rs)
        self.nr = nr

        # Nút: id = i_góc * nr + j_bán_kính
        self.node_ok = np.zeros(nd * nr, dtype=bool)
        self.node_pts = np.zeros((nd * nr,) + POSE_SHAPE)
        for i, d in enumerate(self.degs):
            for j, r in enumerate(self.rs):
                good, q, _ = rm.ik_polar(r, d, z)
                if good:
                    self.node_ok[i * nr + j] = True
                    self.node_pts[i * nr + j] = _pose_points(q)

        # Cạnh: lưu thêm trung điểm để không lọt vật nhỏ nằm giữa 2 nút
        self.adj = [[] for _ in range(nd * nr)]
        self.edge_id = {}
        mids = []
        for i in range(nd):
            for j in range(nr):
                a = i * nr + j
                if not self.node_ok[a]:
                    continue
                for di, dj in NEIGHBORS:
                    i2, j2 = i + di, j + dj
                    if i2 >= nd or not 0 <= j2 < nr or not self.node_ok[i2 * nr + j2]:
                        continue
                    b = i2 * nr + j2
                    pa, pb = self.polar(a), self.polar(b)
                    good, q, _ = rm.ik_polar(*((u + v) / 2.0 for u, v in zip(pa, pb)))
                    if not good:
                        continue
                    e, cost = len(mids), leg_len(pa, pb)
                    mids.append(_pose_points(q))
                    self.adj[a].append((b, e, cost))
                    self.adj[b].append((a, e, cost))
                    self.edge_id[(min(a, b), max(a, b))] = e
        self.mid_pts = np.array(mids) if mids else np.zeros((0,) + POSE_SHAPE)

        self.start_links = self._links(rm.DEG_PICK, lambda p: (rm.P_LIFT, p))
        self.goal_links = {b: self._links(bd["deg"], lambda p, b=b: (p, rm.above_bin(b))) for b, bd in rm.BINS.items()}
        self.ok = True

    def _links(self, deg, ends):
        i = self.degs.index(round(deg, 6))
        ids, pts, costs = [], [], []
        for j in range(self.nr):
            n = i * self.nr + j
            if not self.node_ok[n]:
                continue
            seg = sample_segment(*ends(self.polar(n)))
            if seg is not None:
                ids.append(n)
                pts.append(seg)
                costs.append(leg_len(*ends(self.polar(n))))
        return Links(ids, pts, costs)

    def polar(self, n):
        return self.rs[n % self.nr], self.degs[n // self.nr], self.z

    def blocked(self, obstacles, obs_h, xy_margin, z_margin):
        """(nút bị chặn, cạnh bị chặn) - tính bằng numpy cho mọi nút / cạnh cùng lúc."""
        node_hit = _hit_mask(self.node_pts, obstacles, obs_h, xy_margin, z_margin) | ~self.node_ok
        return node_hit, _hit_mask(self.mid_pts, obstacles, obs_h, xy_margin, z_margin)


@functools.lru_cache(maxsize=None)
def grid_at(z):
    return Grid(z)


def astar(start, goal, neighbors, h):
    """A*: neighbors(n) -> [(nút kề, chi phí)], h(n) -> ước lượng còn lại. Trả về (đường, số nút đã mở rộng)."""
    g = {start: 0.0}
    parent = {start: None}
    open_heap = [(h(start), 0.0, start)]  # (f = g + h, g, nút)
    closed = set()
    while open_heap:
        _f, g_n, n = heapq.heappop(open_heap)
        if n in closed:
            continue
        closed.add(n)
        if n == goal:
            path = []
            while n is not None:
                path.append(n)
                n = parent[n]
            return path[::-1], len(closed)
        for m, cost in neighbors(n):
            if m in closed:
                continue
            g_m = g_n + cost
            if g_m < g.get(m, math.inf):
                g[m] = g_m
                parent[m] = n
                heapq.heappush(open_heap, (g_m + h(m), g_m, m))
    return None, len(closed)


def shortcut(pts, args):
    """Kéo dây: nối thẳng tới điểm XA NHẤT phía trước còn an toàn.
    Trả về (đường rút gọn, None) hoặc (None, i) nếu đoạn pts[i] -> pts[i+1] chạm (cạnh lưới bị lọt)."""
    out = [pts[0]]
    i = 0
    while i < len(pts) - 1:
        if not segment_free(pts[i], pts[i + 1], args):
            return None, i
        j = i + 1
        while j + 1 < len(pts) and segment_free(pts[i], pts[j + 1], args):
            j += 1
        out.append(pts[j])
        i = j
    return out, None


def _search_z(bin_id, z, args, late=lambda: False):
    """Tìm đường ở độ cao z: (điểm né hoặc None, số nút đã mở rộng, lý do thất bại)."""
    grid = grid_at(z)
    if not grid.ok:
        return None, 0, "khong voi toi"
    node_hit, edge_hit = grid.blocked(*args)
    starts = grid.start_links.free(args)
    goals = grid.goal_links[bin_id].free(args)
    lift, above = rm.P_LIFT, rm.above_bin(bin_id)
    r_lo = grid.rs[0]

    def polar(n):
        return lift if n == START else (above if n == GOAL else grid.polar(n))

    def h(n):
        r, deg, zz = polar(n)
        return math.sqrt((r - above[0]) ** 2 + (r_lo * math.radians(abs(deg - above[1]))) ** 2 + (zz - above[2]) ** 2)

    def neighbors(n):
        if n == START:
            return [(m, c) for m, c in starts.items() if not node_hit[m]]
        out = [(m, c) for m, e, c in grid.adj[n] if not node_hit[m] and not edge_hit[e]]
        if n in goals:
            out.append((GOAL, goals[n]))
        return out

    expanded = 0
    for _ in range(LAZY_TRIES):
        if late():
            return None, expanded, "het thoi gian"
        nodes, n_exp = astar(START, GOAL, neighbors, h)
        expanded += n_exp
        if nodes is None:
            return None, expanded, "A* khong co duong"
        pairs = []  # Bỏ điểm trùng (ở Z_TRAVEL nút (R_PATH, góc gắp) trùng S)
        for n in nodes:
            p = polar(n)
            if pairs and all(abs(u - v) < 1e-6 for u, v in zip(p, pairs[-1][0])):
                if n >= 0:
                    pairs[-1] = (p, n)
                continue
            pairs.append((p, n))
        simple, bad = shortcut([p for p, _n in pairs], args)
        if bad is None:
            break
        a, b = pairs[bad][1], pairs[bad + 1][1]  # Kiểm tra thưa thì qua, kiểm tra dày thì chạm -> loại cạnh, tìm lại
        if a == START:
            starts.pop(b, None)
        elif b == GOAL:
            goals.pop(a, None)
        else:
            edge_hit[grid.edge_id[(min(a, b), max(a, b))]] = True
    else:
        return None, expanded, f"loai {LAZY_TRIES} canh van cham"

    waypoints = simple[1:-1]
    if not waypoints:  # Đi đúng cung mặc định, chỉ đường về bị chạm -> 1 điểm né "trên khay"
        waypoints = [simple[-1]]
    if len(waypoints) > proto.MAX_WAYPOINTS:
        return None, expanded, f"{len(waypoints)} diem ne > {proto.MAX_WAYPOINTS}"
    return waypoints, expanded, ""


def plan_astar(bin_id, obstacles, obs_height_mm, xy_margin_mm=22.0, z_margin_mm=15.0, deadline_ms=DEADLINE_MS):
    t0 = time.perf_counter()
    args = (rm.obstacle_arrays(obstacles), obs_height_mm, xy_margin_mm, z_margin_mm)

    def late():
        return (time.perf_counter() - t0) * 1000.0 > deadline_ms

    def finish(kind, waypoints, reason, tried, cycle=None, results=None, direct=False):
        return _make_plan("A*", t0, args, bin_id, kind, waypoints, reason, tried, cycle, results, direct)

    early = _precheck(bin_id, args, lambda kind, wps, reason, _tried: finish(kind, wps, reason, 0))
    if early is not None:
        return early

    expanded, fails = 0, []
    for z in Z_LEVELS:
        if late():
            fails.append("het thoi gian")
            break
        waypoints, n, why = _search_z(bin_id, z, args, late)
        expanded += n
        if waypoints is None:
            fails.append(f"z{z:.0f}: {why}")
            continue
        waypoints = proto.round_trip(waypoints)  # Kiểm tra đúng đường ESP32 sẽ chạy
        direct = direct_home_ok(bin_id, args)
        try:
            moves = rm.cycle_moves(bin_id, waypoints, direct)
        except ValueError as e:
            fails.append(f"z{z:.0f}: {e}")
            continue
        sampled = rm.sample_moves(moves, N_SAMPLES)
        results = rm.check_collision(sampled, *args)
        hit = [res["move"].name for res in results if res["hit"]]
        if hit:
            fails.append(f"z{z:.0f}: kiem tra lai bi cham ({', '.join(hit)})")
            continue
        how = f"A* z={z:.0f}, {len(waypoints)} diem ne, dai {path_len(bin_id, waypoints):.0f}mm"
        if direct:
            how += ", ve thang HOME"
        return finish("PATH", waypoints, how, expanded, (moves, sampled), results, direct)

    if late():
        return finish("BLOCKED", None, f"Tinh qua {deadline_ms:.0f}ms -> BLOCKED cho an toan (" + "; ".join(fails) + ")",
                      expanded)
    return finish("BLOCKED", None, "A* khong co duong (" + "; ".join(fails) + ")", expanded)
