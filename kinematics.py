
import math

# MicroPython không có math.hypot; CPython dùng hypot để kết quả PC giống hệt bản cũ
_hypot = getattr(math, "hypot", None) or (lambda a, b: math.sqrt(a * a + b * b))

# =========================================================================
# 1. THÔNG SỐ HÌNH HỌC (mm) - đo thật, fit bằng fit_geometry.py
# =========================================================================
L1 = 142.0       # Chốt vai -> chốt khuỷu
L2 = 160.0       # Chốt khuỷu -> chốt cổ tay
Z_BASE = 90.0    # Độ cao chốt vai so với mặt bàn
R_OFF = 6.0      # Tâm trục đế -> chốt vai (ngang)
TOOL_R = 84.775  # Chốt cổ tay -> tâm kẹp (ngang, cụm kẹp luôn nằm ngang)
TOOL_Z = -4.640  # Chốt cổ tay -> tâm kẹp (dọc)

SHOULDER_GAIN = -0.850
SHOULDER_OFFSET_DEG = 157.671
ELBOW_COUPLING = 0.024
ELBOW_GAIN = 0.901
ELBOW_OFFSET_DEG = -53.721

R_MIN = 70.0
R_MAX = 380.0
D_MIN = 100.0   # Chốt vai -> cổ tay ngắn nhất

# =========================================================================
# 2. ĐẾ: 3 MỐC servo 0° / 90° / 180° -> góc thật (đo bằng camera 01/11/2026)
# =========================================================================
BASE_DEG_AT_SERVO0 = 55.1     # Điểm gắp
BASE_DEG_AT_SERVO180 = 126.7  # Khay 1
BASE_DEG_MIN = min(BASE_DEG_AT_SERVO0, BASE_DEG_AT_SERVO180)
BASE_DEG_MAX = max(BASE_DEG_AT_SERVO0, BASE_DEG_AT_SERVO180)

LIM_BASE = (BASE_DEG_MIN, BASE_DEG_MAX)  # Góc THỰC
LIM_SHOULDER = (20.0, 160.0)             # Góc servo
LIM_ELBOW = (0.0, 75.0)
LIM_GRIPPER = (35.0, 140.0)

GRIPPER_OPEN = 40.0
GRIPPER_CLOSE = 140.0


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def base_servo_to_deg(servo_deg):
    """Góc servo đế -> góc thật: đường gấp khúc qua (0, A0) - (90, 90) - (180, A180)."""
    if servo_deg <= 90.0:
        return 90.0 + (BASE_DEG_AT_SERVO0 - 90.0) * (90.0 - servo_deg) / 90.0
    return 90.0 + (BASE_DEG_AT_SERVO180 - 90.0) * (servo_deg - 90.0) / 90.0


def base_knots_to_servo(q1, k0, k180):
    """Ngược của đường gấp khúc (0, k0) - (90, 90) - (180, k180)."""
    if (q1 - 90.0) * (k0 - 90.0) >= 0.0:
        return 90.0 - 90.0 * (q1 - 90.0) / (k0 - 90.0)
    return 90.0 + 90.0 * (q1 - 90.0) / (k180 - 90.0)


def base_deg_to_servo_dir(q1, direction, backlash):
    """Góc thực -> góc servo CÓ BÙ RƠ theo chiều quay (+1 tăng, -1 giảm, 0 chưa biết).
    Tâm bánh răng có 2 đầu lệch ra ngoài b/2, đế trễ b/2 sau chiều quay -> ghi servo ứng với (q1 + dir*b/2)."""
    h = 0.5 * backlash
    k0 = BASE_DEG_AT_SERVO0 + (-h if BASE_DEG_AT_SERVO0 < 90.0 else h)
    k180 = BASE_DEG_AT_SERVO180 + (-h if BASE_DEG_AT_SERVO180 < 90.0 else h)
    return clamp(base_knots_to_servo(q1 + direction * h, k0, k180), 0.0, 180.0)


# =========================================================================
# 3. CUNG ĐƯỜNG ĐI & CÁC VỊ TRÍ
# =========================================================================
R_PATH = 300.0      # Bán kính cung gắp/thả
Z_PICK = 120.0      # Độ cao kẹp quả
Z_APPROACH = 160.0  # Từ đây hạ chậm xuống Z_PICK
Z_TRAVEL = 190.0    # Độ cao mang quả
Z_BIN = 150.0       # Độ cao thả vào khay

DEG_PICK = base_servo_to_deg(0.0)
BIN_DEG = {1: base_servo_to_deg(180.0),  # Khay 1 - quả chín
           2: base_servo_to_deg(150.0),  # Khay 2 - quả vừa
           3: base_servo_to_deg(120.0)}  # Khay 3 - quả xanh
POSE_HOME = (90.0, 60.0, 40.0)  # (q1, q2, q3) - tâm kẹp r=204mm, z=180mm

P_LIFT = (R_PATH, DEG_PICK, Z_TRAVEL)
P_APPROACH = (R_PATH, DEG_PICK, Z_APPROACH)
P_PICK = (R_PATH, DEG_PICK, Z_PICK)


def above_bin(bin_id):
    return (R_PATH, BIN_DEG[bin_id], Z_TRAVEL)


def at_bin(bin_id):
    return (R_PATH, BIN_DEG[bin_id], Z_BIN)


# Thời gian đoạn đi qua điểm né = lớn nhất của: độ dài x LEG_MS_PER_MM, góc đế x LEG_MS_PER_DEG, LEG_MS_MIN
# -> điểm né gần đế không làm đế quay vụt
LEG_MS_PER_MM = 10.0
LEG_MS_PER_DEG = 30.0
LEG_MS_MIN = 700


def leg_ms(pa, pb):
    """Thời gian (ms) đi thẳng pa -> pb trong (r, góc, z)."""
    d_deg = abs(pb[1] - pa[1])
    arc = (pa[0] + pb[0]) / 2.0 * math.radians(d_deg)
    dist = math.sqrt((pb[0] - pa[0]) ** 2 + arc ** 2 + (pb[2] - pa[2]) ** 2)
    return max(LEG_MS_MIN, int(dist * LEG_MS_PER_MM), int(d_deg * LEG_MS_PER_DEG))


# =========================================================================
# 4. ĐỘNG HỌC THUẬN / NGHỊCH
# =========================================================================
def link_angles(q2, q3):
    th2 = SHOULDER_GAIN * q2 + SHOULDER_OFFSET_DEG
    phi = ELBOW_COUPLING * th2 + ELBOW_GAIN * q3 + ELBOW_OFFSET_DEG
    return th2, phi


def servo_angles(th2, phi):
    q2 = (th2 - SHOULDER_OFFSET_DEG) / SHOULDER_GAIN
    q3 = (phi - ELBOW_COUPLING * th2 - ELBOW_OFFSET_DEG) / ELBOW_GAIN
    return q2, q3


def arm_rz(q2, q3):
    """(r, z) của chốt vai, chốt khuỷu, chốt cổ tay, tâm kẹp."""
    th2, phi = link_angles(q2, q3)
    th2, phi = math.radians(th2), math.radians(phi)
    shoulder = (R_OFF, Z_BASE)
    elbow = (R_OFF + L1 * math.cos(th2), Z_BASE + L1 * math.sin(th2))
    wrist = (elbow[0] + L2 * math.cos(phi), elbow[1] + L2 * math.sin(phi))
    tip = (wrist[0] + TOOL_R, wrist[1] + TOOL_Z)
    return shoulder, elbow, wrist, tip


def fk(q1, q2, q3):
    r, z = arm_rz(q2, q3)[-1]
    a1 = math.radians(q1)
    return r * math.cos(a1), r * math.sin(a1), z


def ik(x, y, z):
    """Trả về (ok, (q1, q2, q3), lỗi). Thử khuỷu-trên rồi khuỷu-dưới, kiểm tra giới hạn servo."""
    if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):  # NaN lọt qua mọi phép so sánh
        return False, None, "Toa do khong hop le"
    if y < -0.05:
        return False, None, "Y phai >= 0"

    # Góc đế THỰC trong quạt [BASE_DEG_MIN, BASE_DEG_MAX] (cho lệch 0.25° rồi cắt về biên)
    q1 = math.degrees(math.atan2(y, x))
    if q1 < 0.0:
        q1 += 360.0
    if q1 > 180.0:
        q1 = 360.0 - q1
    if q1 < LIM_BASE[0] - 0.25 or q1 > LIM_BASE[1] + 0.25:
        return False, None, "Goc de %.1f ngoai [%.1f, %.1f]" % (q1, LIM_BASE[0], LIM_BASE[1])
    q1 = clamp(q1, LIM_BASE[0], LIM_BASE[1])

    r = _hypot(x, y)
    r_rel = r - R_OFF - TOOL_R  # Cổ tay so với chốt vai
    z_rel = z - Z_BASE - TOOL_Z
    d = _hypot(r_rel, z_rel)
    if d > (L1 + L2) or d < D_MIN or r < R_MIN or r > R_MAX:
        return False, None, "Ngoai tam voi D=%.1fmm r=%.1fmm" % (d, r)

    # Định lý cosin: góc gập gamma giữa 2 khâu
    gamma = math.acos(clamp((d * d - L1 * L1 - L2 * L2) / (2.0 * L1 * L2), -1.0, 1.0))
    alpha = math.atan2(z_rel, r_rel)
    beta = math.atan2(L2 * math.sin(gamma), L1 + L2 * math.cos(gamma))

    err = ""
    for sign in (1.0, -1.0):  # Khuỷu trên trước, khuỷu dưới sau
        th2 = math.degrees(alpha + sign * beta)
        phi = th2 - sign * math.degrees(gamma)
        q2, q3 = servo_angles(th2, phi)
        if q2 < LIM_SHOULDER[0] or q2 > LIM_SHOULDER[1]:
            err = err or "Goc vai %.1f ngoai [%.0f, %.0f]" % (q2, LIM_SHOULDER[0], LIM_SHOULDER[1])
        elif q3 < LIM_ELBOW[0] or q3 > LIM_ELBOW[1]:
            err = err or "Goc khuyu %.1f ngoai [%.0f, %.0f]" % (q3, LIM_ELBOW[0], LIM_ELBOW[1])
        else:
            return True, (q1, q2, q3), ""
    return False, None, err


def polar_to_xyz(r, deg, z):
    rad = math.radians(deg)
    return r * math.cos(rad), r * math.sin(rad), z


def ik_polar(r, deg, z):
    if not (math.isfinite(r) and math.isfinite(deg) and math.isfinite(z)):
        return False, None, "Toa do khong hop le"
    return ik(*polar_to_xyz(r, deg, z))


def lerp3(a, b, s):
    return (a[0] + (b[0] - a[0]) * s, a[1] + (b[1] - a[1]) * s, a[2] + (b[2] - a[2]) * s)


def leg_reachable(pa, pb, n=20):
    """IK tới được mọi điểm i/n (i = 0..n) dọc đoạn thẳng pa -> pb trong (r, góc, z)."""
    for i in range(n + 1):
        if not ik_polar(*lerp3(pa, pb, i / n))[0]:
            return False
    return True


# =========================================================================
# 5. GIAO THỨC ESP32 <-> PC
# =========================================================================
PC_TIMEOUT_MS = 500   # ESP32 chờ PC mặc định (lệnh PCWAIT đổi được)
MAX_WAYPOINTS = 6

CLEAR = "CLEAR"
BLOCKED = "BLOCKED"
DIRECT_HOME = "H"     # Chữ cuối câu PATH: về thẳng HOME


def parse_req(line):
    """'REQ 2' -> 2; dòng khác -> None."""
    parts = line.strip().split()
    if len(parts) != 2 or parts[0] != "REQ":
        return None
    try:
        bin_id = int(parts[1])
    except ValueError:
        return None
    return bin_id if bin_id in (1, 2, 3) else None


def _finite_all(vals):
    for v in vals:
        if not math.isfinite(v):
            return False
    return True


def format_path(waypoints, direct_home=False):
    """[(r, góc, z), ...] -> 'PATH n r1 g1 z1 ...' (1 chữ số thập phân), thêm ' H' nếu direct_home."""
    if not 1 <= len(waypoints) <= MAX_WAYPOINTS:
        raise ValueError("So diem ne phai tu 1 den %d, dang co %d" % (MAX_WAYPOINTS, len(waypoints)))
    if not _finite_all([v for wp in waypoints for v in wp]):
        raise ValueError("PATH co so khong hop le (nan/inf)")
    nums = " ".join("%.1f %.1f %.1f" % (r, deg, z) for r, deg, z in waypoints)
    return "PATH %d %s" % (len(waypoints), nums) + (" " + DIRECT_HOME if direct_home else "")


def parse_path(line):
    """'PATH n ... [H]' -> ([(r, góc, z), ...], về thẳng HOME?); sai cú pháp -> ValueError."""
    parts = line.strip().split()
    if not parts or parts[0] != "PATH":
        raise ValueError("Khong phai lenh PATH")
    direct = len(parts) > 2 and parts[-1] == DIRECT_HOME
    if direct:
        parts = parts[:-1]
    try:
        for tok in parts[1:]:  # CPython nhận '1_90' / '0x1' -> chặn để 2 phía hiểu như nhau
            for c in "_xX":
                if c in tok:
                    raise ValueError("so co ky tu la")
        n = int(parts[1])
        vals = [float(v) for v in parts[2:]]
    except (IndexError, ValueError) as e:
        raise ValueError("PATH sai cu phap: %s" % e)
    if not 1 <= n <= MAX_WAYPOINTS:
        raise ValueError("PATH: n=%d ngoai [1, %d]" % (n, MAX_WAYPOINTS))
    if len(vals) != 3 * n:
        raise ValueError("PATH: can %d so, nhan duoc %d" % (3 * n, len(vals)))
    if not _finite_all(vals):
        raise ValueError("PATH co so khong hop le (nan/inf)")
    return [tuple(vals[i:i + 3]) for i in range(0, 3 * n, 3)], direct


def round_trip(waypoints):
    """Điểm né làm tròn đúng như ESP32 sẽ nhận (format_path -> parse_path)."""
    return parse_path(format_path(waypoints))[0]
