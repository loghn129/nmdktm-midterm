
import collections
import json
import math
import os
import time

import cv2
import numpy as np

import robot_model as rm

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_FILE = os.path.join(BASE_DIR, "calib_config.json")

DEFAULT_CFG = {
    "cam_id": 1,
    "frame_width": 960,
    "frame_height": 540,
    "mm_per_px": 0.8129,        # Tỉ lệ trung bình (vùng bỏ qua); vật cản dùng tỉ lệ cục bộ của ma trận
    "M_px2mm": None,
    "M_mm2px": None,
    "serial_port": "COM3",
    "baud": 115200,
    # ---- Vùng quét + lọc ổn định ----
    "scan_r_min_mm": 115.0,     # Bỏ qua vùng sát đế (thân robot, dây điện)
    "scan_r_max_mm": 390.0,
    "fan_pad_deg": 8.0,         # Quét rộng hơn quạt đế mỗi bên
    "pick_ignore_mm": 35.0,     # Bỏ qua quả ở điểm gắp
    "bin_ignore_mm": 35.0,      # Bỏ qua 3 khay
    "max_obs_radius_mm": 62.0,  # Vật lớn hơn vẫn là vật cản, chỉ gắn cờ "big"
    "stable_frames": 8,         # Vật phải xuất hiện >= stable_hits trong stable_frames khung gần nhất
    "stable_hits": 5,
    "stable_hold": 4,           # Vật đã công nhận được giữ thêm chừng này khung khi tạm mất
    "match_mm": 25.0,
    "merge_mm": 15.0,           # Gộp các mảnh cách nhau < merge_mm thành 1 vật
    "lock_camera": True,        # Khóa phơi sáng / cân bằng trắng / lấy nét khi bấm [KHOA CAM]
    # ---- Màu vật cản (HSV OpenCV: H 0..179, S/V 0..255) ----
    "color_h": 132,             # Tâm dải H: vật cản TÍM đo bằng camera thật H ~118-141 (đỏ ~0/179, xanh tay ~104)
    "color_h_width": 12,
    "color_s_min": 30,          # Loại giấy trắng, xám (S < 10)
    "color_s_max": 255,
    "color_v_min": 45,          # Loại vùng tối (H của pixel tối không đáng tin)
    "color_min_area_px": 150,
    # ---- Né vật cản ----
    "planner": "co_dinh",       # "co_dinh" | "astar" (planners.py)
    "safe_margin_mm": 22.0,     # Lề ngang an toàn
    "safe_z_margin_mm": 15.0,   # Lề đứng
    "detour_r_max_mm": None,    # Bán kính né xa nhất (None = scan_r_max - lề)
    "obs_height_mm": 150.0,     # Chiều cao giả định của vật cản (camera 2D không đo được)
}


# =========================================================================
# 1. CẤU HÌNH
# =========================================================================
def load_config():
    cfg = dict(DEFAULT_CFG)
    if os.path.exists(CONFIG_FILE):
        with open(CONFIG_FILE, "r", encoding="utf-8-sig") as f:
            data = json.load(f)
        cfg.update({k: v for k, v in data.items() if v is not None or k not in cfg})
    return cfg


def update_config(changes):
    """Đọc LẠI file rồi chỉ sửa các khóa trong changes (không ghi đè hiệu chuẩn). Ghi file tạm rồi thay thế."""
    data = {}
    if os.path.exists(CONFIG_FILE):
        with open(CONFIG_FILE, "r", encoding="utf-8-sig") as f:
            data = json.load(f)
    data.update(changes)
    tmp = CONFIG_FILE + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        f.write(json.dumps(data, indent=4))
    os.replace(tmp, CONFIG_FILE)


# =========================================================================
# 2. QUY ĐỔI TỌA ĐỘ (mm hệ robot <-> pixel)
# =========================================================================
def has_robot_calib(cfg):
    return cfg.get("M_px2mm") is not None and cfg.get("M_mm2px") is not None


def _apply(m, a, b):
    w = m[2][0] * a + m[2][1] * b + m[2][2]
    return (m[0][0] * a + m[0][1] * b + m[0][2]) / w, (m[1][0] * a + m[1][1] * b + m[1][2]) / w


def xy_to_uv(x_mm, y_mm, cfg):
    u, v = _apply(cfg["M_mm2px"], x_mm, y_mm)
    return int(round(u)), int(round(v))


def uv_to_xy(u_px, v_px, cfg):
    return _apply(cfg["M_px2mm"], u_px, v_px)


def polar_to_uv(radius_mm, deg, cfg):
    rad = math.radians(deg)
    return xy_to_uv(radius_mm * math.cos(rad), radius_mm * math.sin(rad), cfg)


def local_mm_per_px(cfg, u, v):
    """Tỉ lệ mm/px CỤC BỘ tại (u, v) (phối cảnh: tỉ lệ đổi theo vị trí). cfg: dict hoặc ma trận M_px2mm."""
    m = np.asarray(cfg.get("M_px2mm") if isinstance(cfg, dict) else cfg, dtype=np.float64)
    p0 = np.array(_apply(m, u, v))
    return math.sqrt(abs(float(np.cross(np.array(_apply(m, u + 1, v)) - p0, np.array(_apply(m, u, v + 1)) - p0))))


def make_obstacle(u, v, r_px, cfg, source="CAMERA", area_px=0.0, r_mm=None, xy_mm=None):
    """Vật cản: (u, v, r_px) để VẼ; x_mm, y_mm, r_mm để tính va chạm."""
    x_mm, y_mm = uv_to_xy(u, v, cfg) if xy_mm is None else (float(xy_mm[0]), float(xy_mm[1]))
    return {"u": int(u), "v": int(v), "r_px": int(r_px), "x_mm": x_mm, "y_mm": y_mm,
            "r_mm": r_px * cfg["mm_per_px"] if r_mm is None else float(r_mm),
            "area_px": area_px, "source": source, "big": False}


# =========================================================================
# 3. CAMERA
# =========================================================================
def open_camera(cfg):
    cap = cv2.VideoCapture(cfg.get("cam_id", 0), cv2.CAP_DSHOW)
    if not cap.isOpened():
        cap = cv2.VideoCapture(cfg.get("cam_id", 0))
    if cap.isOpened():
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg.get("frame_width", 640))
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg.get("frame_height", 480))
        if cfg.get("lock_camera", False):
            set_camera_auto(cap)  # Driver nhớ chế độ chỉnh tay từ lần trước -> trả về tự động
    return cap


def _is_dshow(cap):
    try:
        return cap.getBackendName() == "DSHOW"
    except cv2.error:
        return False


def set_camera_auto(cap):
    """Trả phơi sáng / cân bằng trắng / lấy nét về TỰ ĐỘNG."""
    for prop, val in ((cv2.CAP_PROP_AUTO_EXPOSURE, 1.0 if _is_dshow(cap) else 0.75),
                      (cv2.CAP_PROP_AUTO_WB, 1.0), (cv2.CAP_PROP_AUTOFOCUS, 1.0)):
        try:
            cap.set(prop, val)
        except cv2.error:
            pass


def _frame_stats(frame):
    small = cv2.resize(frame, (frame.shape[1] // 2, frame.shape[0] // 2))
    gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
    b, _, r = cv2.split(small.astype(np.float32))
    return {"bright": float(gray.mean()),
            "sharp": float(cv2.Laplacian(gray, cv2.CV_32F).var()),
            "tint": float((b.mean() + 1.0) / (r.mean() + 1.0))}


class CameraLocker:
    """Khóa lấy nét / cân bằng trắng / phơi sáng KHÔNG chặn giao diện (mỗi khung gọi step(frame) 1 lần).
    Mỗi thứ khóa xong đều kiểm tra lại ảnh: mờ / đổi màu thì trả về tự động; phơi sáng dò nhị phân tới độ sáng
    gần lúc tự động (khóa thẳng giá trị driver báo hay ra ảnh đen)."""
    SETTLE = 6              # Số khung chờ camera áp dụng giá trị mới
    EXP_RANGE = (-13, -1)   # Thang phơi sáng DirectShow: log2(giây)

    def __init__(self, cap, restore_first=False):
        self.cap = cap
        self.dshow = _is_dshow(cap)
        self.locked = []
        self.done = False
        self._restore_wait = 0
        if restore_first:
            set_camera_auto(cap)
            self._restore_wait = 25
        self._gen = self._run()
        next(self._gen)

    def step(self, frame):
        if self.done:
            return True
        try:
            self._gen.send(frame)
        except StopIteration:
            self.done = True
        except cv2.error:
            set_camera_auto(self.cap)
            self.locked, self.done = [], True
        return self.done

    def _frames(self, n):
        frame = None
        for _ in range(n):
            frame = yield
        return frame

    def _settle(self):
        frame = yield from self._frames(self.SETTLE)
        return _frame_stats(frame)

    def _run(self):
        cap = self.cap
        ref = _frame_stats((yield from self._frames(self._restore_wait + 2)))
        # 1. Lấy nét
        if cap.get(cv2.CAP_PROP_FOCUS) != -1:
            focus = cap.get(cv2.CAP_PROP_FOCUS)
            cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
            cap.set(cv2.CAP_PROP_FOCUS, focus)
            st = yield from self._settle()
            if st["sharp"] >= 0.6 * ref["sharp"]:
                self.locked.append("NET")
            else:
                cap.set(cv2.CAP_PROP_AUTOFOCUS, 1)
        # 2. Cân bằng trắng
        if cap.get(cv2.CAP_PROP_WB_TEMPERATURE) not in (-1.0, 0.0):
            wb = cap.get(cv2.CAP_PROP_WB_TEMPERATURE)
            cap.set(cv2.CAP_PROP_AUTO_WB, 0)
            ok = cap.set(cv2.CAP_PROP_WB_TEMPERATURE, wb)
            st = yield from self._settle()
            if ok and abs(st["tint"] / ref["tint"] - 1.0) < 0.15:
                self.locked.append("MAU")
            else:
                cap.set(cv2.CAP_PROP_AUTO_WB, 1)
        # 3. Phơi sáng: dò nhị phân
        target = ref["bright"] if 50.0 <= ref["bright"] <= 200.0 else 120.0
        lo, hi = self.EXP_RANGE
        e0 = cap.get(cv2.CAP_PROP_EXPOSURE)
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0 if self.dshow else 0.25)
        tried = {}
        last = None
        e = int(round(e0)) if lo <= e0 <= hi else (lo + hi) // 2
        while lo <= hi and e not in tried:
            if not cap.set(cv2.CAP_PROP_EXPOSURE, e):
                break
            last = e
            tried[e] = (yield from self._settle())["bright"]
            if 0.8 * target <= tried[e] <= 1.25 * target:
                break
            if tried[e] < target:
                lo = e + 1
            else:
                hi = e - 1
            e = (lo + hi) // 2
        best = min(tried, key=lambda k: abs(tried[k] - target)) if tried else None
        if best is not None and 0.6 * target <= tried[best] <= 1.6 * target:
            if best != last:
                cap.set(cv2.CAP_PROP_EXPOSURE, best)
                yield from self._settle()
            self.locked.append(f"SANG({best})")
        else:
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1.0 if self.dshow else 0.75)


def read_frame(cap, cfg):
    """(có ảnh không, ảnh). Mất camera -> ảnh xám."""
    w, h = cfg.get("frame_width", 640), cfg.get("frame_height", 480)
    ok, frame = (False, None)
    if cap is not None and cap.isOpened():
        ok, frame = cap.read()
    if not ok or frame is None:
        return False, np.full((h, w, 3), 35, dtype=np.uint8)
    if frame.shape[1] != w or frame.shape[0] != h:
        frame = cv2.resize(frame, (w, h))
    return True, frame


# =========================================================================
# 4. VÙNG QUÉT
# =========================================================================
def fan_polygon(cfg, r_in, r_out, deg_from, deg_to, n=60):
    outer = [polar_to_uv(r_out, d, cfg) for d in np.linspace(deg_from, deg_to, n)]
    inner = [polar_to_uv(r_in, d, cfg) for d in np.linspace(deg_to, deg_from, n)]
    return np.array(outer + inner, dtype=np.int32)


def ignore_zones(cfg):
    """[(u, v, r_px)] vùng bỏ qua: điểm gắp + 3 khay (bán kính theo tỉ lệ nhỏ hơn -> phủ ít nhất đủ số mm)."""
    k = cfg["mm_per_px"]
    zones = []
    for deg, r_mm in [(rm.DEG_PICK, cfg.get("pick_ignore_mm", 35.0))] + \
                     [(b["deg"], cfg.get("bin_ignore_mm", 35.0)) for b in rm.BINS.values()]:
        u, v = polar_to_uv(rm.R_PATH, deg, cfg)
        zones.append((u, v, int(round(r_mm / min(k, local_mm_per_px(cfg, u, v))))))
    return zones


def arc_out_of_frame(cfg, r_mm, deg_from, deg_to, step=1.0):
    w, h = cfg.get("frame_width", 640), cfg.get("frame_height", 480)
    out = []
    for d in np.arange(deg_from, deg_to + 1e-6, step):
        u, v = polar_to_uv(r_mm, float(d), cfg)
        if not (0 <= u < w and 0 <= v < h):
            out.append(float(d))
    return out


def scan_coverage(cfg):
    """(r_quét, [góc ra ngoài khung ở r_quét], bán kính lớn nhất mà cả cung quét còn nằm trong ảnh)."""
    r_scan = float(cfg.get("scan_r_max_mm", 390.0))
    pad = float(cfg.get("fan_pad_deg", 8.0))
    lo, hi = rm.BASE_DEG_MIN - pad, rm.BASE_DEG_MAX + pad
    out = arc_out_of_frame(cfg, r_scan, lo, hi)
    r_vis = r_scan
    while out and r_vis > 100.0:
        r_vis -= 5.0
        if not arc_out_of_frame(cfg, r_vis, lo, hi):
            break
    return r_scan, out, r_vis


def visible_r_max(cfg):
    return scan_coverage(cfg)[2]


# =========================================================================
# 5. BỘ NHẬN DIỆN VẬT CẢN THEO MÀU
# =========================================================================
class ColorObstacleDetector:
    """Vật cản = vật có MÀU CHỌN TRƯỚC trong vùng quét (quạt đế, trừ điểm gắp + 3 khay).
    update(frame, frame_ok) -> vật cản ỔN ĐỊNH: vật mới phải xuất hiện >= stable_hits trong stable_frames khung;
    vật đã công nhận được giữ khi bị mảng to hơn trùm lên, hoặc mất <= stable_hold khung.
    data_ok() -> (True, "OK") hoặc (False, lý do): danh sách có đáng tin không (web trả BLOCKED nếu không).
    Màu: H trong [color_h ± color_h_width] (vòng tròn 0..179), S trong [color_s_min, color_s_max], V >= color_v_min."""
    STALE_S = 1.0
    LOST = "MAT CAMERA"
    FROZEN_FRAMES = 5
    DARK_MEAN = 10.0
    FLAT_STD = 1.0
    CLIP_GROW_MM = 20.0   # Vật chạm mép ảnh (bị cắt): nới bán kính

    def __init__(self, cfg):
        self.cfg = cfg
        self.w = cfg.get("frame_width", 640)
        self.h = cfg.get("frame_height", 480)
        self._locker = None
        self.cam_locked = None
        self._hist = collections.deque(maxlen=int(cfg.get("stable_frames", 8)))
        self._tracks = []
        self._last_out = []
        self._last_t = None
        self._bad = None
        self._warm = 0
        self._prev_small = None
        self._same = 0
        self.ws_mask = self.build_workspace_mask()
        r_scan, out, r_vis = scan_coverage(cfg)
        self.unseen = (min(r_vis + 5.0, r_scan), min(out), max(out)) if out else None
        self._merge_px_cache = None

    def build_workspace_mask(self):
        cfg = self.cfg
        mask = np.zeros((self.h, self.w), dtype=np.uint8)
        pad = cfg.get("fan_pad_deg", 8.0)
        poly = fan_polygon(cfg, cfg.get("scan_r_min_mm", 115.0), cfg.get("scan_r_max_mm", 390.0),
                           rm.BASE_DEG_MIN - pad, rm.BASE_DEG_MAX + pad)
        cv2.fillPoly(mask, [poly], 255)
        for u, v, r_px in ignore_zones(cfg):
            cv2.circle(mask, (u, v), r_px, 0, -1)
        return mask

    def _merge_px(self):
        """Bán kính gộp mảnh (px) theo tỉ lệ mm/px nhỏ nhất trong vùng quét (phía an toàn)."""
        if self._merge_px_cache is None:
            cfg = self.cfg
            k = float(cfg["mm_per_px"])
            pad = float(cfg.get("fan_pad_deg", 8.0))
            for r in (float(cfg.get("scan_r_min_mm", 115.0)), rm.R_PATH, float(cfg.get("scan_r_max_mm", 390.0))):
                for deg in (rm.BASE_DEG_MIN - pad, 90.0, rm.BASE_DEG_MAX + pad):
                    u, v = polar_to_uv(r, deg, cfg)
                    if 0 <= u < self.w and 0 <= v < self.h:
                        k = min(k, local_mm_per_px(cfg, u, v))
            self._merge_px_cache = int(round(float(cfg.get("merge_mm", 15.0)) / k))
        return self._merge_px_cache

    # ---------------- Khóa camera ----------------
    def start_lock_camera(self, cap):
        """Nút [KHOA CAM]: khóa phơi sáng / màu / nét (màu vật ổn định hơn). Bấm lại -> mở khóa rồi khóa lại."""
        restore = bool(self.cam_locked)
        if self._locker is not None:
            set_camera_auto(self._locker.cap)
            restore = True
        self._locker = CameraLocker(cap, restore_first=restore)
        self._reset_tracking()

    def is_capturing(self):
        return self._locker is not None

    def _reset_tracking(self):
        self._hist.clear()
        self._tracks = []
        self._last_out = []
        self._warm = 0

    def data_ok(self):
        if self._last_t is None:
            return False, "CHUA CO KHUNG HINH"
        age = time.monotonic() - self._last_t
        if age > self.STALE_S:
            return False, f"DU LIEU CU ({age:.1f}s chua cap nhat)"
        if self._bad is not None:
            return False, self._bad
        if self.is_capturing():
            return False, "DANG KHOA CAMERA"
        need = int(self.cfg.get("stable_frames", 8))
        if self._warm < need:
            return False, f"DANG ON DINH ({self._warm}/{need} khung)"
        return True, "OK"

    def _frame_problem(self, frame, frame_ok):
        """Lý do khung KHÔNG dùng được (None = dùng được): mất camera, ảnh đen, một màu, đứng hình."""
        if not frame_ok or frame is None or getattr(frame, "ndim", 0) != 3:
            self._prev_small, self._same = None, 0
            return self.LOST
        small = np.ascontiguousarray(frame[::4, ::4])
        prev, self._prev_small = self._prev_small, small
        if prev is not None and prev.shape == small.shape and np.array_equal(prev, small):
            self._same += 1
        else:
            self._same = 0
        mean, std = cv2.meanStdDev(small)
        if float(mean.mean()) < self.DARK_MEAN:
            return "ANH DEN (che ong kinh / phoi sang hong)"
        if float(std.max()) < self.FLAT_STD:
            return "ANH MOT MAU (mat camera)"
        if self._same + 1 >= self.FROZEN_FRAMES:
            return "CAMERA DUNG HINH"
        return None

    # ---------------- Nhận diện ----------------
    def color_mask(self, frame):
        hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (5, 5), 0), cv2.COLOR_BGR2HSV)
        c = self.cfg
        h = hsv[..., 0].astype(np.int16)
        dh = np.abs(h - int(c.get("color_h", DEFAULT_CFG["color_h"])))
        hue_ok = np.minimum(dh, 180 - dh) <= int(c.get("color_h_width", DEFAULT_CFG["color_h_width"]))
        sv = cv2.inRange(hsv, np.array([0, int(c.get("color_s_min", DEFAULT_CFG["color_s_min"])), int(c.get("color_v_min", DEFAULT_CFG["color_v_min"]))]),
                         np.array([179, int(c.get("color_s_max", DEFAULT_CFG["color_s_max"])), 255]))
        return np.where(hue_ok, sv, 0).astype(np.uint8)

    def _mask_to_obstacles(self, binary, min_area):
        """Mặt nạ (trắng = vật) -> lọc nhiễu -> gộp mảnh -> vật cản (vòng tròn bao nhỏ nhất tính trong hệ mm)."""
        H, W = binary.shape[:2]
        edge_touch = np.zeros((H, W), dtype=bool)  # Vật chạm mép ảnh (xét trên mặt nạ gốc)
        edge_touch[0, :], edge_touch[H - 1, :], edge_touch[:, 0], edge_touch[:, W - 1] = (binary[0, :] > 0), \
            (binary[H - 1, :] > 0), (binary[:, 0] > 0), (binary[:, W - 1] > 0)
        k3 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        k7 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, k3, iterations=1)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, k7, iterations=2)

        merge_px = self._merge_px()
        grouped = cv2.dilate(binary, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (merge_px, merge_px))) \
            if merge_px >= 2 else binary
        n_lbl, labels, stats, _ = cv2.connectedComponentsWithStats(grouped)
        fg = binary > 0
        counts = np.bincount(labels[fg], minlength=n_lbl)

        cfg = self.cfg
        m_px2mm = np.asarray(cfg["M_px2mm"], dtype=np.float64)
        max_r_mm = float(cfg.get("max_obs_radius_mm", 62.0))
        obstacles = []
        for lbl in range(1, n_lbl):
            area = float(counts[lbl])
            if area < min_area:
                continue
            x, y, w, h = (int(c) for c in stats[lbl, :4])
            ys, xs = np.nonzero((labels[y:y + h, x:x + w] == lbl) & fg[y:y + h, x:x + w])
            pts = np.column_stack([xs + x, ys + y]).astype(np.int32)
            (uc, vc), r_px = cv2.minEnclosingCircle(pts)
            hull = cv2.convexHull(pts).reshape(-1, 1, 2).astype(np.float64)
            mm = cv2.perspectiveTransform(hull, m_px2mm).reshape(-1, 2).astype(np.float32)
            (cx, cy), r_mm = cv2.minEnclosingCircle(mm)
            r_mm = max(float(r_mm), 10.0 * local_mm_per_px(m_px2mm, uc, vc))
            ob = make_obstacle(round(uc), round(vc), max(10, round(r_px)), cfg, "CAMERA", area,
                               r_mm=r_mm, xy_mm=(cx, cy))
            if edge_touch[y:y + h, x:x + w].any():
                ob["clipped"] = True
                ob["r_mm"] += max(ob["r_mm"], self.CLIP_GROW_MM)
            ob["big"] = ob["r_mm"] > max_r_mm
            obstacles.append(ob)
        obstacles.sort(key=lambda o: o["area_px"], reverse=True)
        return obstacles

    def update(self, frame, frame_ok=True):
        """Vật cản ỔN ĐỊNH của khung này. Khung hỏng -> trả lại danh sách lần trước, data_ok() = False."""
        self._last_t = time.monotonic()
        bad = self._frame_problem(frame, frame_ok)
        if bad is not None:
            if bad != self.LOST and self._locker is not None:
                self._step_locker(frame)  # Đang dò phơi sáng: ảnh tối / cháy vẫn đưa cho CameraLocker
            self._bad, self._warm = bad, 0
            return list(self._last_out)
        self._bad = None
        if self._locker is not None:
            self._step_locker(frame)
            self._last_out = []
            return []
        binary = cv2.bitwise_and(self.color_mask(frame), self.ws_mask)
        raw = self._mask_to_obstacles(binary, float(self.cfg.get("color_min_area_px", 150)))
        self._warm += 1

        self._hist.append([(o["x_mm"], o["y_mm"]) for o in raw])
        need = int(self.cfg.get("stable_hits", 5))
        match = float(self.cfg.get("match_mm", 25.0))
        stable = []
        for ob in raw:
            seen = sum(1 for frame_obs in self._hist
                       if any(math.hypot(ob["x_mm"] - x, ob["y_mm"] - y) < match for x, y in frame_obs))
            if seen >= need:
                stable.append(ob)
        self._last_out = self._update_tracks(raw, stable, match)
        return list(self._last_out)

    def _step_locker(self, frame):
        if self._locker.step(frame):
            self.cam_locked = self._locker.locked
            self._locker = None

    def _update_tracks(self, raw, stable, match):
        """Trễ khi BIẾN MẤT: vật đã công nhận được giữ khi có vật thô gần nó, bị vật thô to hơn trùm lên
        (tay người chạm vào vật), hoặc mới mất <= stable_hold khung."""
        hold = int(self.cfg.get("stable_hold", 4))
        tracks, used = [], set()
        for t in self._tracks:
            near = [o for o in raw if id(o) not in used and math.hypot(o["x_mm"] - t["x_mm"], o["y_mm"] - t["y_mm"]) < match]
            if near:
                o = min(near, key=lambda o: math.hypot(o["x_mm"] - t["x_mm"], o["y_mm"] - t["y_mm"]))
                o["miss"] = 0
                used.add(id(o))
                tracks.append(o)
            elif any(math.hypot(o["x_mm"] - t["x_mm"], o["y_mm"] - t["y_mm"]) < o["r_mm"] for o in raw):
                tracks.append(t if t.get("miss", 0) == 0 else dict(t, miss=0))
            elif t.get("miss", 0) < hold:
                tracks.append(dict(t, miss=t.get("miss", 0) + 1))
        for s in stable:
            if id(s) not in used and not any(math.hypot(s["x_mm"] - t["x_mm"], s["y_mm"] - t["y_mm"]) < match
                                             for t in tracks):
                s["miss"] = 0
                tracks.append(s)
        tracks.sort(key=lambda o: o["area_px"], reverse=True)
        self._tracks = tracks
        return tracks


# =========================================================================
# 6. HÀM VẼ
# =========================================================================
def draw_polyline_mm(img, cfg, pts_xy, color, thickness=2):
    pts = np.array([xy_to_uv(p[0], p[1], cfg) for p in pts_xy], dtype=np.int32).reshape((-1, 1, 2))
    cv2.polylines(img, [pts], False, color, thickness, cv2.LINE_AA)


def draw_arc(img, cfg, r_mm, deg_from, deg_to, color, thickness=2, n=60):
    pts = [(r_mm * math.cos(math.radians(d)), r_mm * math.sin(math.radians(d)))
           for d in np.linspace(deg_from, deg_to, n)]
    draw_polyline_mm(img, cfg, pts, color, thickness)


def draw_robot_layout(img, cfg):
    """Đế, cung R_PATH, HOME, điểm gắp, 3 khay (chiếu ở mặt phẳng hiệu chuẩn z = 80mm)."""
    bu, bv = xy_to_uv(0.0, 0.0, cfg)
    draw_arc(img, cfg, rm.R_PATH, rm.BASE_DEG_MIN, rm.BASE_DEG_MAX, (200, 200, 0), 1)
    hx, hy, _hz = rm.fk(*rm.POSE_HOME)
    hu, hv = xy_to_uv(hx, hy, cfg)
    cv2.circle(img, (hu, hv), 6, (255, 255, 255), -1)
    cv2.putText(img, "HOME", (hu - 20, hv - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (220, 255, 220), 1)
    pu, pv = polar_to_uv(rm.R_PATH, rm.DEG_PICK, cfg)
    cv2.circle(img, (pu, pv), 10, (255, 255, 0), -1)
    cv2.circle(img, (pu, pv), 13, (255, 255, 255), 2)
    cv2.putText(img, f"GAP ({rm.DEG_PICK:.0f}d)", (pu - 40, pv - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 2)
    for b_id, b in rm.BINS.items():
        u, v = polar_to_uv(rm.R_PATH, b["deg"], cfg)
        cv2.circle(img, (u, v), 9, b["color"], -1)
        cv2.circle(img, (u, v), 11, (255, 255, 255), 1)
        cv2.putText(img, f"K{b_id} ({b['deg']:.0f}d)", (u - 30, v - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.42, b["color"], 1)
    cv2.circle(img, (bu, bv), 7, (0, 0, 255), -1)
    cv2.circle(img, (bu, bv), 10, (255, 255, 255), 2)


def draw_detection_zone(img, detector, alpha=0.08):
    """Tô vùng quét (màu tô + đường viền tính 1 lần, lưu trên detector) + cảnh báo phần ngoài khung hình."""
    cache = getattr(detector, "_zone_cache", None)
    if cache is None or cache[0] != img.shape:
        color = np.empty(img.shape, np.uint8)
        color[:] = (255, 160, 60)
        cnt, _ = cv2.findContours(detector.ws_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cache = (img.shape, color, cnt)
        detector._zone_cache = cache
    _, color, cnt = cache
    blend = cv2.addWeighted(img, 1.0 - alpha, color, alpha, 0)
    np.copyto(img, blend, where=(detector.ws_mask > 0)[..., None])
    cv2.drawContours(img, cnt, -1, (200, 140, 60), 1, cv2.LINE_AA)
    if detector.unseen is not None:
        r, d0, d1 = detector.unseen
        cv2.putText(img, f"NGOAI KHUNG HINH: r>={r:.0f}mm goc {d0:.0f}-{d1:.0f}d -> camera KHONG thay vat can o do",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 255), 1)
