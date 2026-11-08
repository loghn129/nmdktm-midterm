import gc
import math
import select
import struct
import sys
import time

from machine import PWM, Pin, time_pulse_us
from micropython import const

import kinematics as km
from kinematics import (BIN_DEG, DEG_PICK, GRIPPER_CLOSE, GRIPPER_OPEN, LIM_BASE, LIM_ELBOW, LIM_GRIPPER,
                        LIM_SHOULDER, MAX_WAYPOINTS, P_APPROACH, P_LIFT, P_PICK, R_PATH, Z_TRAVEL, above_bin,
                        at_bin, clamp, ik_polar, leg_ms, lerp3)

# =========================================================================
# 1. PHẦN CỨNG
# =========================================================================
SERVO_PINS = (13, 26, 14, 27)   # Đế, vai, khuỷu, kẹp
SERVO_US_MIN = const(600)       # Xung (us) ứng với 0° servo
SERVO_US_MAX = const(2350)      # Xung (us) ứng với 180° servo
SERVO_PWM_HZ = const(50)

# Cảm biến màu TCS3200
S0_PIN = const(18)
S1_PIN = const(19)
S2_PIN = const(4)
S3_PIN = const(16)
OUT_PIN = const(15)

# Độ rơ từng khớp 
BASE_BACKLASH_DEFAULT = 2.39
JOINT_BACKLASH_MAX = 15.0
BACKLASH_KEYS = ("backlash", "bl_vai", "bl_khuyu")  # đế, vai, khuỷu

POSE_HOME = (km.POSE_HOME[0], km.POSE_HOME[1], km.POSE_HOME[2], GRIPPER_OPEN)

# Trạng thái FSM: mỗi trạng thái = động tác ĐANG CHẠY
ST_WAIT_NEW_FRUIT = const(0)   # Đứng ở HOME, quét tìm quả
ST_WAIT_PC = const(1)          # Đã gửi REQ, chờ PC
ST_BLOCKED = const(2)          # PC báo BLOCKED -> chờ rồi hỏi lại
ST_ROTATE_TO_PICK = const(3)
ST_REACH_ABOVE_PICK = const(4)
ST_DESCEND_FAST = const(5)
ST_DESCEND_SLOW = const(6)
ST_GRAB_ITEM = const(7)
ST_LIFT_ITEM = const(8)
ST_FOLLOW_WP = const(9)
ST_MOVE_TO_BIN = const(10)
ST_LOWER_TO_BIN = const(11)
ST_RELEASE_ITEM = const(12)
ST_LIFT_FROM_BIN = const(13)
ST_RETURN_VIA_WP = const(14)
ST_RETURN_UP = const(15)
ST_RETURN_HOME = const(16)
ST_ERROR = const(17)           # IK lỗi -> chờ lệnh HOME


class Servo:
    """1 servo trên 1 chân PWM 50Hz. Tạo đối tượng = bật xung ngay với độ rộng us."""

    def __init__(self, pin, us):
        # Đặt độ rộng xung ngay lúc tạo: tạo PWM rồi mới đặt thì có 1 nhịp xung mặc định -> servo giật
        self.pwm = PWM(Pin(pin), freq=SERVO_PWM_HZ, duty_ns=self._ns(us))

    @staticmethod
    def _ns(us):
        return int(clamp(us, SERVO_US_MIN, SERVO_US_MAX) * 1000.0 + 0.5)

    def write_us(self, us):
        self.pwm.duty_ns(self._ns(us))


class BacklashStore:
    """Độ rơ lưu trong NVS "arm" (blob float 4 byte - giống putFloat() của Preferences)."""

    def __init__(self):
        import esp32
        self.nvs = esp32.NVS("arm")

    def load(self, key, default):
        buf = bytearray(4)
        try:
            self.nvs.get_blob(key, buf)
            v = struct.unpack("<f", buf)[0]
        except (OSError, ValueError):
            return default
        return v if math.isfinite(v) and 0.0 <= v <= JOINT_BACKLASH_MAX else default

    def save(self, key, v):
        self.nvs.set_blob(key, struct.pack("<f", v))
        self.nvs.commit()


class SerialIn:
    """Đọc lệnh từ cổng USB-Serial (sys.stdin) KHÔNG chặn: gom từng ký tự, đủ 1 dòng mới trả về."""

    def __init__(self):
        self.poll = select.poll()
        self.poll.register(sys.stdin, select.POLLIN)
        self.buf = []

    def readline(self):
        while self.poll.poll(0):
            c = sys.stdin.read(1)
            if not c:
                break
            if c == "\n":
                line = "".join(self.buf)
                self.buf = []
                return line
            if c != "\r" and len(self.buf) < 200:
                self.buf.append(c)
        return None


# =========================================================================
# 2. BỘ ĐIỀU KHIỂN CÁNH TAY (QUỸ ĐẠO BẬC 5)
# =========================================================================
def servo_deg_to_us(deg):
    return SERVO_US_MIN + clamp(deg, 0.0, 180.0) * (SERVO_US_MAX - SERVO_US_MIN) / 180.0


def quintic(tau):
    if tau <= 0.0:
        return 0.0
    if tau >= 1.0:
        return 1.0
    return tau * tau * tau * (10.0 + tau * (-15.0 + 6.0 * tau))


def grip_or_open(g):
    return clamp(g, LIM_GRIPPER[0], LIM_GRIPPER[1]) if math.isfinite(g) else GRIPPER_OPEN


class JointDir:
    """Chiều quay gần nhất của 1 khớp (để bù rơ). anchor chỉ dời khi khớp đã đi > 0.05° -> bước nội suy nhỏ
    ở đầu / cuối đoạn được cộng dồn, không đảo chiều nhầm."""

    def __init__(self):
        self.dir = 0
        self.anchor = 0.0

    def update(self, q):
        if q > self.anchor + 0.05:
            self.dir, self.anchor = 1, q
        elif q < self.anchor - 0.05:
            self.dir, self.anchor = -1, q
        return self.dir


class RobotArm:
    """Pose = [đế thực, vai, khuỷu, kẹp] (độ). update(now) gọi liên tục, ghi servo 50Hz."""

    def __init__(self, backlash):
        self.backlash = backlash          # [đế, vai, khuỷu] - list dùng chung với lệnh RO
        self.servos = []
        self.q_start = list(POSE_HOME)
        self.q_target = list(POSE_HOME)
        self.q_cur = list(POSE_HOME)
        self.moving = False
        self.polar = False                # Nội suy theo (r, góc, z) (move_arc_linear) hay theo khớp (move_to_pose)
        self.p_start = self.p_target = None
        self.t_start = self.t_dur = self.last_pwm = 0
        self.dirs = (JointDir(), JointDir(), JointDir())
        self.out = [0.0, 0.0, 0.0, 0.0]   # Góc servo sẽ ghi (sau bù rơ)

    def _compute_servo(self, p):
        b = self.backlash
        d = self.dirs
        self.out[0] = km.base_deg_to_servo_dir(p[0], d[0].update(p[0]), b[0])
        self.out[1] = p[1] + 0.5 * d[1].update(p[1]) * b[1]
        self.out[2] = p[2] + 0.5 * d[2].update(p[2]) * b[2]
        self.out[3] = p[3]

    def _write(self, p):
        self._compute_servo(p)
        for s, deg in zip(self.servos, self.out):
            s.write_us(servo_deg_to_us(deg))

    def init(self):
        """Bật xung từng servo cách nhau 150ms (chống sụt áp nguồn 5V)."""
        for i in range(3):
            self.dirs[i].anchor = self.q_cur[i]
        self._compute_servo(self.q_cur)
        for pin, deg in zip(SERVO_PINS, self.out):
            try:
                self.servos.append(Servo(pin, servo_deg_to_us(deg)))
            except Exception as e:  # noqa: BLE001
                print("-> [WARN] Khong tao duoc xung servo o chan %d: %s" % (pin, e))
            time.sleep_ms(150)

    def _start(self, ms, now=None):
        self.t_dur = max(int(ms), 300)
        self.t_start = self.last_pwm = time.ticks_ms() if now is None else now
        self.moving = True

    def move_to_pose(self, target, ms):
        """Đi theo khớp. Trả về False nếu phải cắt góc về giới hạn (góc NaN -> không chạy)."""
        for v in target:
            if not math.isfinite(v):
                return False
        t = [clamp(target[0], LIM_BASE[0], LIM_BASE[1]), clamp(target[1], LIM_SHOULDER[0], LIM_SHOULDER[1]),
             clamp(target[2], LIM_ELBOW[0], LIM_ELBOW[1]), clamp(target[3], LIM_GRIPPER[0], LIM_GRIPPER[1])]
        self.q_start = list(self.q_cur)
        self.q_target = t
        self.polar = False
        self._start(ms)
        return t == list(target)

    def current_arc(self):
        x, y, z = km.fk(self.q_cur[0], self.q_cur[1], self.q_cur[2])
        return math.sqrt(x * x + y * y), math.degrees(math.atan2(y, x)), z

    def move_arc_linear(self, target, grip, ms):
        """Đi THẲNG trong (r, góc, z): cùng r, góc -> thẳng đứng; cùng r, z -> cung tròn quanh đế.
        Kiểm tra IK trước 20 điểm dọc đường, có điểm lỗi thì không chạy."""
        start = self.current_arc()
        q = None
        for i in range(1, 21):
            ok, q, err = ik_polar(*lerp3(start, target, i / 20.0))
            if not ok:
                print("[IK LOI] Duong di tai %d%%: %s" % (i * 5, err))
                return False
        self.q_start = list(self.q_cur)
        self.q_target = [q[0], q[1], q[2], grip_or_open(grip)]
        self.p_start, self.p_target = start, target
        self.polar = True
        self._start(ms)
        return True

    def move_to_arc(self, p, grip, ms):
        """Đi tới điểm (r, góc thực, z) theo khớp."""
        ok, q, err = ik_polar(p[0], p[1], p[2])
        if not ok:
            print("[IK LOI] " + err)
            return False
        self.move_to_pose((q[0], q[1], q[2], grip_or_open(grip)), ms)
        return True

    def update(self, now):
        if not self.moving:
            return
        tau = time.ticks_diff(now, self.t_start) / self.t_dur
        if tau >= 1.0:
            self.moving = False
            self.q_cur = list(self.q_target)
            self._write(self.q_cur)
        elif time.ticks_diff(now, self.last_pwm) >= 20:  # Cập nhật 50Hz
            self.last_pwm = now
            s = quintic(tau)
            a, b, c = self.q_start, self.q_target, self.q_cur
            if self.polar:
                ok, q, _err = ik_polar(*lerp3(self.p_start, self.p_target, s))
                if ok:
                    c[0], c[1], c[2] = q
            else:
                for i in range(3):
                    c[i] = a[i] + (b[i] - a[i]) * s
            c[3] = a[3] + (b[3] - a[3]) * s
            self._write(c)

    def pose(self):
        return list(self.q_cur)


# =========================================================================
# 3. CẢM BIẾN MÀU TCS3200
# =========================================================================
GROUP_GREEN = "Qua xanh"
GROUP_MEDIUM = "Qua vua"
GROUP_RIPE = "Qua chin"
GROUP_NONE = "Khong co qua"

COLOR_TABLE = (  # (tên, r, g, b, nhóm)
    ("Den", 0, 0, 0, GROUP_NONE), ("Trang", 255, 255, 255, GROUP_NONE),
    ("Xanh nuoc", 44, 150, 200, GROUP_GREEN), ("Xanh la", 117, 174, 69, GROUP_GREEN),
    ("Vang", 255, 255, 154, GROUP_MEDIUM), ("Cam", 255, 175, 94, GROUP_MEDIUM),
    ("Hong", 182, 153, 190, GROUP_RIPE), ("Do", 255, 70, 60, GROUP_RIPE),
)
COLOR_CAL = ((1512, 7633), (1275, 6211), (1512, 7352))  # (min, max) tần số kênh R, G, B


class ColorSensor:
    def __init__(self):
        Pin(S0_PIN, Pin.OUT).value(1)  # Tần số ra 20%
        Pin(S1_PIN, Pin.OUT).value(0)
        self.s2, self.s3 = Pin(S2_PIN, Pin.OUT), Pin(S3_PIN, Pin.OUT)
        self.out = Pin(OUT_PIN, Pin.IN)

    def _freq(self, s2, s3):
        self.s2.value(s2)
        self.s3.value(s3)
        time.sleep_ms(15)
        if self.out.value() == 0:  # Đang giữa xung thấp -> bỏ phần còn lại (giống pulseIn của Arduino)
            time_pulse_us(self.out, 0, 25000)
        d = time_pulse_us(self.out, 0, 25000)
        return 1000000 // d if d > 0 else 0

    def classify(self):
        """Đọc 3 kênh -> RGB 0..255 -> mẫu màu gần nhất -> nhóm quả."""
        f = (self._freq(0, 0), self._freq(1, 1), self._freq(0, 1))
        rgb = [clamp((v - lo) * 255 // (hi - lo), 0, 255) for v, (lo, hi) in zip(f, COLOR_CAL)]
        best, group = -1, GROUP_NONE
        for _name, r, g, b, grp in COLOR_TABLE:
            d = (rgb[0] - r) ** 2 + (rgb[1] - g) ** 2 + (rgb[2] - b) ** 2
            if best < 0 or d < best:
                best, group = d, grp
        return group


# =========================================================================
# 4. CUNG ĐƯỜNG, THỜI GIAN, GIAO THỨC
# =========================================================================
PC_TIMEOUT_MS = km.PC_TIMEOUT_MS   # Mặc định chờ PC (lệnh PCWAIT đổi được)
BLOCKED_RETRY_MS = const(1500)
COLOR_SAME_NEEDED = const(3)
FRUIT_BIN = {GROUP_RIPE: 1, GROUP_MEDIUM: 2, GROUP_GREEN: 3}
BIN_FRUIT = {1: GROUP_RIPE, 2: GROUP_MEDIUM, 3: GROUP_GREEN}


def parse_path_cmd(cmd, bin_id):
    """'PATH n r1 g1 z1 ... [H]' -> ((điểm né, về thẳng HOME?), "") hoặc (None, lỗi).
    Cú pháp dùng chung kinematics.parse_path; thêm kiểm IK 21 điểm mỗi đoạn trên điểm gắp -> ... -> trên khay."""
    try:
        wps, direct = km.parse_path(cmd)
    except ValueError as e:
        return None, str(e)
    prev = P_LIFT
    for i, w in enumerate(wps):
        if not km.leg_reachable(prev, w):
            return None, "IK khong toi doan -> diem %d" % (i + 1)
        prev = w
    if not km.leg_reachable(prev, above_bin(bin_id)):
        return None, "IK khong toi doan diem cuoi -> khay"
    return (wps, direct), ""


def to_int(s):
    """Giống String.toInt() của Arduino: số nguyên ở đầu chuỗi, không có -> 0."""
    s = s.strip()
    i, sign, n = 0, 1, 0
    if s and s[0] in "+-":
        sign, i = (-1 if s[0] == "-" else 1), 1
    while i < len(s) and "0" <= s[i] <= "9":
        n = n * 10 + ord(s[i]) - 48
        i += 1
    return sign * n


def scan_numbers(s, n_float, n_int=0):
    """Giống sscanf("%f ... %lu"): đọc lần lượt n_float số thực rồi n_int số nguyên, dừng ở chỗ sai đầu tiên."""
    out = []
    for k, tok in enumerate(s.split()[:n_float + n_int]):
        try:
            out.append(float(tok) if k < n_float else int(tok))
        except ValueError:
            break
    return out


# =========================================================================
# 5. MÁY TRẠNG THÁI PHÂN LOẠI TỰ ĐỘNG + LỆNH SERIAL
# =========================================================================
class Sorter:
    def __init__(self, arm, sensor, serial_in, store=None):
        self.arm, self.sensor, self.serial_in, self.store = arm, sensor, serial_in, store
        self.state = ST_WAIT_NEW_FRUIT
        self.auto_enabled = True      # False khi điều khiển tay qua Serial
        self.run_pending = False      # RUN n: đang về HOME, tới nơi (nghỉ 400ms) mới gửi REQ
        self.fruit = GROUP_NONE
        self.bin_id = 1
        self.wait_start = self.wait_ms = 0
        self.color_last, self.color_same = GROUP_NONE, 0
        self.last_scan = 0
        self.wp = []                  # Điểm né PC gửi ([] = đường mặc định)
        self.wp_idx = 0               # Đi tới: số điểm né đã bắt đầu đi
        self.ret_step = 0             # Đi về: còn bao nhiêu điểm né (-1 = đã về trên điểm gắp)
        self.direct_home = False      # PATH có "H": về thẳng HOME sau khi thả quả
        self.pc_timeout_ms = PC_TIMEOUT_MS
        self.pc_req_time = 0
        self.was_busy = False
        self.last_gc = 0

    # ---------------- Tiện ích ----------------
    def wait_for(self, ms):
        self.wait_start, self.wait_ms = time.ticks_ms(), ms

    def reset_color_filter(self):
        self.color_same, self.color_last = 0, GROUP_NONE

    def enter_error(self, where):
        print("[AUTO LOI] IK that bai o buoc: %s -> DUNG. Go HOME de chay lai." % where)
        self.state = ST_ERROR

    def auto_move(self, p, grip, ms, nxt, where):
        """Đi THẲNG tới p; IK lỗi -> ST_ERROR."""
        if not self.arm.move_arc_linear(p, grip, ms):
            self.enter_error(where)
            return False
        self.state = nxt
        return True

    def set_gripper(self, grip, ms, nxt):
        cur = self.arm.pose()
        cur[3] = grip
        self.arm.move_to_pose(cur, ms)
        self.state = nxt

    def start_cycle(self):
        self.arm.move_to_pose((DEG_PICK, POSE_HOME[1], POSE_HOME[2], GRIPPER_OPEN), 1500)  # Chỉ quay đế
        self.state = ST_ROTATE_TO_PICK

    def request_path(self):
        print("REQ %d" % self.bin_id)
        self.pc_req_time = time.ticks_ms()
        self.state = ST_WAIT_PC

    def block_cycle(self, msg):
        print(msg)
        self.wait_for(BLOCKED_RETRY_MS)
        self.state = ST_BLOCKED

    def handle_pc_reply(self, cmd):
        """Câu trả lời của PC cho REQ."""
        if not self.auto_enabled or self.state != ST_WAIT_PC:
            print("[PC] Bo qua '%s' (khong o trang thai cho PC)" % cmd)
            return
        if cmd == km.CLEAR:
            print("[PC] CLEAR -> duong mac dinh")
            self.wp = []
            self.start_cycle()
        elif cmd == km.BLOCKED:
            self.block_cycle("[PC] BLOCKED -> dung cho")
        else:
            res, err = parse_path_cmd(cmd, self.bin_id)
            if res is None:
                self.block_cycle("[PC LOI] PATH sai (" + err + ") -> coi nhu BLOCKED")
                return
            self.wp, self.direct_home = res
            print("[PC] PATH %d diem ne%s" % (len(self.wp), ", ve thang HOME" if self.direct_home else ""))
            self.start_cycle()

    def next_transfer_leg(self):
        """Mang quả: trên điểm gắp -> wp[0] -> ... -> wp[n-1] -> trên khay."""
        wp = self.wp
        if self.wp_idx < len(wp):
            frm = P_LIFT if self.wp_idx == 0 else wp[self.wp_idx - 1]
            to = wp[self.wp_idx]
            self.wp_idx += 1
            self.auto_move(to, GRIPPER_CLOSE, leg_ms(frm, to), ST_FOLLOW_WP, "Diem ne")
        else:
            frm = wp[-1] if wp else P_LIFT
            to = above_bin(self.bin_id)
            self.auto_move(to, GRIPPER_CLOSE, leg_ms(frm, to) if wp else 2000, ST_MOVE_TO_BIN, "Cung -> khay")

    def next_return_leg(self):
        """Về khi có điểm né (không có H): đi NGƯỢC lại đúng các điểm né (PC đã kiểm tra lúc đi) -> trên điểm gắp
        -> HOME."""
        wp = self.wp
        if self.ret_step > 0:
            frm = above_bin(self.bin_id) if self.ret_step == len(wp) else wp[self.ret_step]
            self.ret_step -= 1
            to = wp[self.ret_step]
            self.auto_move(to, GRIPPER_OPEN, leg_ms(frm, to), ST_RETURN_VIA_WP, "Ve qua diem ne")
        elif self.ret_step == 0:
            self.ret_step = -1
            self.auto_move(P_LIFT, GRIPPER_OPEN, leg_ms(wp[0], P_LIFT), ST_RETURN_VIA_WP, "Ve tren diem gap")
        else:
            self.arm.move_to_pose((DEG_PICK, POSE_HOME[1], POSE_HOME[2], GRIPPER_OPEN), 1000)
            self.state = ST_RETURN_UP

    # ---------------- FSM ----------------
    def step(self):
        if not self.auto_enabled or self.arm.moving:  # Chỉ chuyển trạng thái khi động tác trước đã xong
            return
        now = time.ticks_ms()
        if time.ticks_diff(now, self.wait_start) < self.wait_ms:
            return
        st = self.state

        if st == ST_WAIT_NEW_FRUIT:
            if self.run_pending:  # RUN n: hỏi luôn, không cần cảm biến
                self.run_pending = False
                self.request_path()
                return
            if time.ticks_diff(now, self.last_scan) < 350:
                return
            self.last_scan = now
            res = self.sensor.classify()
            if res == GROUP_NONE:
                self.reset_color_filter()
                return
            if res == self.color_last:  # Phải đọc CÙNG 1 nhóm màu 3 lần liên tiếp
                self.color_same += 1
            else:
                self.color_last, self.color_same = res, 1
            print("[COLOR] %s (%d/%d)" % (res, self.color_same, COLOR_SAME_NEEDED))
            if self.color_same < COLOR_SAME_NEEDED:
                return
            self.reset_color_filter()
            self.fruit = res
            print("[AUTO] PHAT HIEN: %s -> HOI DUONG PC ROI GAP!" % res)
            self.bin_id = FRUIT_BIN.get(res, 1)
            self.request_path()
        elif st == ST_WAIT_PC:
            if time.ticks_diff(now, self.pc_req_time) > self.pc_timeout_ms:
                print("[PC] Khong tra loi -> duong mac dinh")
                self.wp = []
                self.start_cycle()
        elif st == ST_BLOCKED:
            self.request_path()
        elif st == ST_ROTATE_TO_PICK:  # Cùng góc đế -> vươn tới trên điểm gắp ở Z_TRAVEL
            ok, q, _err = ik_polar(R_PATH, DEG_PICK, Z_TRAVEL)
            if not ok:
                self.enter_error("Tren diem gap")
                return
            self.arm.move_to_pose((q[0], q[1], q[2], GRIPPER_OPEN), 1000)
            self.state = ST_REACH_ABOVE_PICK
        elif st == ST_REACH_ABOVE_PICK:  # Hạ thẳng đứng nhanh tới Z_APPROACH
            self.auto_move(P_APPROACH, GRIPPER_OPEN, 900, ST_DESCEND_FAST, "Ha nhanh")
        elif st == ST_DESCEND_FAST:  # ... rồi chậm tới Z_PICK
            self.auto_move(P_PICK, GRIPPER_OPEN, 1200, ST_DESCEND_SLOW, "Ha cham")
        elif st == ST_DESCEND_SLOW:
            self.set_gripper(GRIPPER_CLOSE, 900, ST_GRAB_ITEM)
        elif st == ST_GRAB_ITEM:  # Nâng thẳng đứng lên Z_TRAVEL
            self.auto_move(P_LIFT, GRIPPER_CLOSE, 1200, ST_LIFT_ITEM, "Nang len")
        elif st == ST_LIFT_ITEM:  # Cung tròn (hoặc qua điểm né) sang trên khay
            self.wp_idx = 0
            self.next_transfer_leg()
        elif st == ST_FOLLOW_WP:
            self.next_transfer_leg()
        elif st == ST_MOVE_TO_BIN:
            self.auto_move(at_bin(self.bin_id), GRIPPER_CLOSE, 800, ST_LOWER_TO_BIN, "Ha xuong khay")
        elif st == ST_LOWER_TO_BIN:
            self.set_gripper(GRIPPER_OPEN, 600, ST_RELEASE_ITEM)
        elif st == ST_RELEASE_ITEM:
            self.auto_move(above_bin(self.bin_id), GRIPPER_OPEN, 700, ST_LIFT_FROM_BIN, "Nhac khoi khay")
        elif st == ST_LIFT_FROM_BIN:
            if not self.wp or self.direct_home:  # Về thẳng HOME (đường mặc định, hoặc PATH có H)
                self.arm.move_to_pose(POSE_HOME, 1800)
                self.state = ST_RETURN_HOME
            else:
                self.ret_step = len(self.wp)
                self.next_return_leg()
        elif st == ST_RETURN_VIA_WP:
            self.next_return_leg()
        elif st == ST_RETURN_UP:
            self.arm.move_to_pose(POSE_HOME, 1500)
            self.state = ST_RETURN_HOME
        elif st == ST_RETURN_HOME:
            print("[AUTO] Da ve HOME. Bat dau do mau qua tiep theo ngay!")
            self.wait_for(400)  # Nghỉ 400ms cho cánh tay hết rung
            self.reset_color_filter()
            self.state = ST_WAIT_NEW_FRUIT

    # ---------------- Lệnh Serial ----------------
    def set_manual(self):
        self.run_pending = False
        if self.auto_enabled:
            self.auto_enabled = False
            print("[AUTO] TAT (dang dieu khien tay). Go 'AUTO ON' de chay lai chu trinh.")

    def go_home(self):
        self.arm.move_to_pose(POSE_HOME, 1500)
        self.run_pending = False
        self.reset_color_filter()
        self.wait_for(0)
        self.state = ST_WAIT_NEW_FRUIT

    def print_backlash(self):
        b = self.arm.backlash
        print("[RO] de=%.2f vai=%.2f khuyu=%.2f (de: do thuc, vai/khuyu: do servo)" % (b[0], b[1], b[2]))

    def handle_backlash(self, a):
        """"RO" | "RO DE|VAI|KHUYU <do>" (lưu flash) | "RO TAM DE|VAI|KHUYU <do>" (chỉ RAM)."""
        a = a.strip().upper()
        if not a:
            self.print_backlash()
            return
        save = True
        if a.startswith("TAM "):
            save, a = False, a[4:].strip()
        parts = a.split(" ", 1)
        joint, rest = parts[0], (parts[1] if len(parts) > 1 else "")
        idx = {"DE": 0, "VAI": 1, "KHUYU": 2}.get(joint)
        v = None
        if idx is not None and rest:
            try:
                v = float(rest)
            except ValueError:
                v = None
        if v is None or not math.isfinite(v) or not 0.0 <= v <= JOINT_BACKLASH_MAX:
            print("[LOI] Cu phap: RO | RO DE|VAI|KHUYU <0..15> | RO TAM DE|VAI|KHUYU <0..15>")
            return
        self.arm.backlash[idx] = v
        if save and self.store is not None:
            self.store.save(BACKLASH_KEYS[idx], v)
        self.print_backlash()

    def handle_command(self, cmd):
        cmd = cmd.strip()
        if not cmd:
            return
        cu = cmd.upper()
        arm = self.arm

        if cmd == km.CLEAR or cmd == km.BLOCKED or cmd.startswith("PATH "):  # Câu trả lời của PC (không tắt AUTO)
            self.handle_pc_reply(cmd)
        elif cmd.startswith("PCWAIT "):
            self.pc_timeout_ms = max(100, to_int(cmd[7:]))
            print("[PC] Cho PC toi da %dms" % self.pc_timeout_ms)
        elif cu == "AUTO ON":
            self.auto_enabled = True
            self.run_pending = False
            self.reset_color_filter()
            self.wait_for(0)
            self.state = ST_WAIT_NEW_FRUIT
            arm.move_to_pose(POSE_HOME, 1500)
            print("[AUTO] BAT. Ve HOME va cho qua.")
        elif cu == "AUTO OFF":
            self.set_manual()
            print("[AUTO] Da tat.")
        elif cmd.startswith("RUN "):
            # RUN n: chạy 1 chu trình -> khay n không cần cảm biến. Về HOME, nghỉ 400ms rồi mới REQ.
            # Đang chạy chu trình / chờ PC -> bỏ qua (không cắt ngang khi đang kẹp quả).
            b = to_int(cmd[4:])
            idle = not self.auto_enabled or self.state in (ST_WAIT_NEW_FRUIT, ST_BLOCKED, ST_ERROR)
            if b not in BIN_DEG:
                print("[LOI] Cu phap: RUN <1|2|3>")
            elif not idle or self.run_pending:
                print("[RUN] Bo qua: dang chay chu trinh / cho PC (doi '[AUTO] Da ve HOME' roi gui lai)")
            else:
                self.bin_id, self.fruit = b, BIN_FRUIT[b]
                self.auto_enabled = self.run_pending = True
                self.wait_for(0)
                arm.move_to_pose(POSE_HOME, 1500)
                self.state = ST_RETURN_HOME
                print("[RUN] Ve HOME roi hoi duong -> Khay %d" % b)
        elif cu == "HOME":
            self.go_home()
            print("[EXEC] Ve HOME.")
        elif cu == "PICK":
            self.set_manual()
            arm.move_to_arc(P_PICK, GRIPPER_OPEN, 1800)
        elif cu in ("BIN1", "BIN2", "BIN3"):
            self.set_manual()
            arm.move_to_arc(at_bin(int(cu[3])), GRIPPER_CLOSE, 2000)
        elif cu in ("OPEN", "CLOSE"):
            self.set_manual()
            p = arm.pose()
            p[3] = GRIPPER_OPEN if cu == "OPEN" else GRIPPER_CLOSE
            arm.move_to_pose(p, 600)
        elif cu == "COLOR":
            print("[SENSOR] TCS3200: %s" % self.sensor.classify())
        elif cu == "FK":
            q = arm.pose()
            x, y, z = km.fk(q[0], q[1], q[2])
            print("[FK] Goc: (B=%.1f, S=%.1f, E=%.1f) -> (X=%.1f, Y=%.1f, Z=%.1f)mm | Cung: r=%.1fmm"
                  % (q[0], q[1], q[2], x, y, z, math.sqrt(x * x + y * y)))
        elif cu == "RO" or cmd.startswith("RO "):
            self.handle_backlash(cmd[2:])
        elif cu == "HELP":
            print("LENH: AUTO ON | AUTO OFF | RUN 1..3 | PCWAIT ms | CLEAR | BLOCKED | PATH n r g z ... [H]")
            print("      HOME | PICK | BIN1..3 | OPEN | CLOSE | COLOR | FK")
            print("      POL r goc z [kep] [ms] | POSE de vai khuyu kep [ms]")
            print("      RO | RO DE|VAI|KHUYU do (luu flash) | RO TAM DE|VAI|KHUYU do (chi RAM)")
        elif cmd.startswith("POSE "):  # POSE <de thuc> <vai> <khuyu> <kep> [ms]
            v = scan_numbers(cmd[5:], 4, 1)
            if len(v) < 4:
                print("[LOI] Cu phap: POSE <b> <s> <e> <g> [T_ms]")
            elif not all(math.isfinite(x) for x in v[:4]):
                print("[LOI] Goc khong hop le (nan/inf)")
            else:
                self.set_manual()
                if not arm.move_to_pose(v[:4], v[4] if len(v) > 4 else 1500):
                    print("[CANH BAO] Goc vuot gioi han -> da cat ve gioi han an toan!")
        elif cmd.startswith("POL "):  # POL <r> <goc thuc> <z> [kep] [ms]
            v = scan_numbers(cmd[4:], 4, 1)
            if len(v) < 3:
                print("[LOI] Cu phap: POL <r> <deg> <z> [grip] [T_ms]")
            else:
                self.set_manual()
                grip = v[3] if len(v) > 3 else GRIPPER_OPEN
                if not arm.move_to_arc(v[:3], grip, v[4] if len(v) > 4 else 1800):
                    print("[LOI] Diem ngoai vung lam viec cua IK!")
        else:
            print("[?] Lenh khong hop le. Go 'HELP' de xem danh sach lenh!")

    # ---------------- 1 vòng lặp ----------------
    def loop_once(self):
        now = time.ticks_ms()
        self.arm.update(now)
        try:
            self.step()
            line = self.serial_in.readline()
            if line:
                self.handle_command(line)
        except Exception as e:  # noqa: BLE001 - lỗi lạ: dừng chu trình nhưng vẫn nhận lệnh (gõ HOME)
            sys.print_exception(e)
            self.enter_error("Loi chuong trinh")
        busy = self.arm.moving
        if self.was_busy and not busy and not self.auto_enabled:
            print("[DONE]")  # Lệnh tay đã chạy xong
        self.was_busy = busy
        if not busy and time.ticks_diff(now, self.last_gc) > 1000:  # Dọn bộ nhớ lúc tay đứng yên
            self.last_gc = now
            gc.collect()


# =========================================================================
# 6. KHỞI ĐỘNG + VÒNG LẶP CHÍNH
# =========================================================================
def setup():
    print("\n=== ESP32 EEZYbotARM MK2 - FK/IK AUTO SORT ===")
    sensor = ColorSensor()
    store = BacklashStore()
    backlash = [store.load(BACKLASH_KEYS[0], BASE_BACKLASH_DEFAULT),
                store.load(BACKLASH_KEYS[1], 0.0), store.load(BACKLASH_KEYS[2], 0.0)]
    arm = RobotArm(backlash)
    sorter = Sorter(arm, sensor, SerialIn(), store)
    sorter.print_backlash()
    arm.init()
    arm.move_to_pose(POSE_HOME, 1500)
    print("-> San sang! Dat qua vao vi tri gap. Go HELP de xem lenh.")
    return sorter


def main():
    sorter = setup()
    try:
        while True:
            sorter.loop_once()
            time.sleep_ms(10)
    except KeyboardInterrupt:  # Thonny bấm Stop / Ctrl-C: servo giữ nguyên vị trí
        print("[DUNG] main.py da dung (Ctrl-C). Reset ESP32 de chay lai.")


if __name__ == "__main__":
    main()
