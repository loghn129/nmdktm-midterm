
import collections
import threading
import time
import unicodedata

from kinematics import (BLOCKED, CLEAR, DIRECT_HOME, MAX_WAYPOINTS, PC_TIMEOUT_MS,  # noqa: F401 - giao thức dùng chung
                        format_path, parse_path, parse_req, round_trip)

try:
    import serial
except ImportError:
    serial = None

# ============================================================
# KẾT NỐI SERIAL
# ============================================================
RESET_MARKERS = ("FK/IK AUTO SORT", "rst:", "ets ")
RESET_LINE = "[RESET] ESP32 khoi dong lai"
RESET_JUNK = "\x00� "  # Byte rác hay dính ở đầu dòng ROM lúc reset


def is_reset_line(line):
    return RESET_MARKERS[0] in line or line.lstrip(RESET_JUNK).startswith(RESET_MARKERS[1:])


def ascii_text(s):
    """Bỏ dấu tiếng Việt (putText của OpenCV chỉ vẽ được ASCII)."""
    s = unicodedata.normalize("NFKD", str(s).replace("đ", "d").replace("Đ", "D"))
    return s.encode("ascii", "ignore").decode("ascii")


class RobotLink:
    def __init__(self, port, baud=115200):
        self.port = port
        self.ser = None
        self.err = None
        self.lines = collections.deque(maxlen=60)
        self.n_lines = 0
        self.resets = 0
        self._lock = threading.Lock()
        self._running = False
        self._closed = False
        self._req_handler = None

        if serial is None:
            self.err = "Chua cai pyserial: pip install pyserial"
            return
        try:
            self.ser = serial.Serial(port, baud, timeout=0.05)
        except Exception as e:  # noqa: BLE001
            self.err = f"Khong mo duoc {port}: {ascii_text(e)}"
            self.ser = None
            return
        time.sleep(2.5)  # ESP32 tự reset khi mở cổng -> chờ khởi động xong rồi bỏ chữ in lúc boot
        try:
            self.ser.reset_input_buffer()
        except Exception as e:  # noqa: BLE001
            self.err = f"Mat ket noi: {ascii_text(e)}"
            return
        self._running = True
        threading.Thread(target=self._reader, daemon=True).start()

    @property
    def ok(self):
        return self.ser is not None and self.err is None and not self._closed

    def _add_line(self, line):
        self.lines.append(line)
        self.n_lines += 1

    def lines_since(self, mark):
        with self._lock:
            n = self.n_lines
            k = min(n - mark, len(self.lines))
            return (list(self.lines)[-k:] if k > 0 else []), n

    def _lost(self, why):
        """Ghi lỗi mất kết nối 1 lần (gọi khi đang giữ _lock hoặc từ luồng đọc)."""
        if self.err is None:
            self.err = f"Mat ket noi: {ascii_text(why)}"
            self._add_line(f"[MAT KET NOI] {ascii_text(why)}")
        self._running = False

    def _reader(self):
        while self._running:
            try:
                raw = self.ser.readline()
            except Exception as e:  # noqa: BLE001
                if self._running:
                    with self._lock:
                        self._lost(e)
                break
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").strip()
            if not line:
                continue
            print(f"[ESP32] {ascii(line)[1:-1]}")
            with self._lock:
                self._add_line(line)
                if is_reset_line(line):
                    self.resets += 1
                    self._add_line(RESET_LINE)
            bin_id = parse_req(line)
            if bin_id is not None and self._req_handler is not None:
                threading.Thread(target=self._req_handler, args=(bin_id,), daemon=True).start()

    def set_req_handler(self, fn):
        self._req_handler = fn

    def send(self, cmd):
        """Gửi 1 lệnh. Trả về False nếu không gửi được (chưa kết nối / rút cáp)."""
        with self._lock:
            if not self.ok:
                return False
            print(f"[PC -> ESP32] {cmd}")
            try:
                self.ser.write((cmd.strip() + "\n").encode("ascii", errors="replace"))
            except Exception as e:  # noqa: BLE001
                self._lost(e)
                return False
            return True

    def last_line(self):
        return self.lines[-1] if self.lines else ""

    def close(self):
        self._running = False
        self._closed = True
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:  # noqa: BLE001
                pass
