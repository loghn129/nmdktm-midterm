
import argparse
import collections
import csv
import json
import logging
import math
import os
import re
import threading
import time
import traceback

import cv2
import numpy as np
from flask import Flask, Response, render_template
from flask_socketio import SocketIO, emit

import esp32_link as proto
import planners
import robot_model as rm
from esp32_link import RESET_LINE, RobotLink, ascii_text
from vision_utils import (ColorObstacleDetector, draw_detection_zone, draw_polyline_mm, draw_robot_layout,
                          has_robot_calib, load_config, local_mm_per_px, open_camera, read_frame, update_config,
                          xy_to_uv)

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
TEACH_FILE = os.path.join(BASE_DIR, "teach_points.json")
HISTORY_FILE = os.path.join(BASE_DIR, "history.csv")

PLANNER_LABEL = {"co_dinh": "NE: CO DINH", "astar": "NE: A*"}
HEIGHT_OPTIONS = (60.0, 100.0, 150.0, 250.0, 9999.0)  # 9999 = coi vật cao vô hạn (chỉ né ngang)
PC_WAIT_MS = 1500
SAVE_KEYS = ("planner", "obs_height_mm", "serial_port")
BUSY_STALE_S = 40.0      # ESP32 im lặng lâu hơn chu trình dài nhất -> coi như rảnh
DONE_TIMEOUT_S = 8.0     # Chuỗi lệnh (Teach): quá lâu không thấy "[DONE]" -> hủy
ALERT_PREFIXES = ("[PC LOI]", "[PC] Bo qua", "[PC] Khong tra loi", "[AUTO LOI]", "[RUN] Bo qua", "[MAT KET NOI]",
                  "[LOI]", "[IK LOI]", "[CANH BAO]", "[TIMEOUT]", RESET_LINE)
JOINT_SEND_S = 0.125     # Kéo thanh trượt: gửi POSE tối đa 8 lần/giây
JOINT_MOVE_MS = 300
JOINT_KEYS = ("base", "shoulder", "elbow", "gripper")
JOINT_LIMITS = {"base": (0.0, 180.0), "shoulder": rm.LIM_SHOULDER, "elbow": rm.LIM_ELBOW, "gripper": rm.LIM_GRIPPER}
GROUP_TO_BIN = {"Qua chin": 1, "Qua vua": 2, "Qua xanh": 3}
BIN_LABEL = {1: "Quả chín", 2: "Quả vừa", 3: "Quả xanh"}
BIN_COLOR = {1: "#e5484d", 2: "#f5a524", 3: "#46a758"}
BLOCKED = proto.BLOCKED
CAM_VIEW_FPS = 10        # Ảnh camera gửi lên web (chỉ vẽ khi có người đang xem)
FK_RE = re.compile(r"^\[FK\] Goc: \(B=(-?[\d.]+), S=(-?[\d.]+), E=(-?[\d.]+)\)")
COLOR_RE = re.compile(r"^\[COLOR\] (.+?) \((\d)/(\d)\)")


# =========================================================================
# 1. PHẦN DÙNG CHUNG (chuyển từ main.py cũ)
# =========================================================================
class CycleTracker:
    """ESP32 đang chạy chu trình hay rảnh, theo các dòng nó gửi về (để không gửi RUN lúc đang bận).
      bận : "[RUN] Ve HOME", "REQ n", "[PC] CLEAR / PATH / Khong tra loi", "[RUN] Bo qua"
      rảnh: "[AUTO] Da ve HOME" (trừ lần về HOME ngay sau RUN), "[PC] BLOCKED", lỗi, AUTO bật/tắt, khởi động lại."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.busy = self.run_pending = False
        self.t_last = time.time()

    def sent_run(self):
        self.busy, self.run_pending, self.t_last = True, True, time.time()

    def feed(self, line):
        self.t_last = time.time()
        if line.startswith("[RUN] Ve HOME"):
            self.busy, self.run_pending = True, True
        elif proto.parse_req(line) is not None:
            self.busy, self.run_pending = True, False
        elif line.startswith("[RUN] Bo qua"):  # RUN bị bỏ qua -> "Da ve HOME" kế tiếp là của chu trình đang chạy
            self.busy, self.run_pending = True, False
        elif line.startswith(("[PC] CLEAR", "[PC] PATH", "[PC] Khong tra loi")):
            self.busy = True
        elif line.startswith("[AUTO] Da ve HOME"):
            if not self.run_pending:
                self.busy = False
        elif line.startswith(("[PC] BLOCKED", "[PC LOI]", "[AUTO LOI]", "[AUTO] TAT", "[AUTO] Da tat", "[AUTO] BAT",
                              "[EXEC] Ve HOME", "[MAT KET NOI]", RESET_LINE)):
            self.reset()

    def is_busy(self):
        if self.busy and time.time() - self.t_last > BUSY_STALE_S:
            self.reset()
        return self.busy


def answer_req(bin_id, obstacles, ready, why, cfg, margin, z_margin):
    """Câu trả lời cho "REQ <khay>". Không bao giờ ném lỗi: lỗi / camera "mù" -> BLOCKED."""
    t0 = time.perf_counter()

    def blocked(reason):
        return planners.Plan(proto.BLOCKED, proto.BLOCKED, [], reason, [], [], 0, (time.perf_counter() - t0) * 1000.0,
                             "PC")

    if not ready:
        return blocked(f"CAMERA CHUA SAN SANG: {why} -> BLOCKED")
    try:
        return planners.plan(cfg["planner"], bin_id, obstacles, cfg.get("obs_height_mm", 150.0), margin, z_margin)
    except Exception as e:  # noqa: BLE001 - không trả lời = ESP32 tự đi đường mặc định -> phải trả BLOCKED
        traceback.print_exc()
        return blocked(ascii_text(f"LOI PLANNER ({type(e).__name__}: {e}) -> BLOCKED"))


def list_ports():
    try:
        from serial.tools import list_ports as lp
        return [p.device for p in lp.comports()]
    except ImportError:
        return []


def servo_to_pose(key, servo_deg):
    """Góc servo trên thanh trượt -> (góc dùng trong POSE, góc servo sau khi cắt giới hạn, có bị cắt không).
    Đế: servo 0..180 -> góc THỰC qua bánh răng (mô hình 3 mốc); khớp khác: chính là góc servo."""
    lo, hi = JOINT_LIMITS[key]
    s = min(max(float(servo_deg), lo), hi)
    return (rm.base_servo_to_deg(s) if key == "base" else s), s, abs(s - float(servo_deg)) > 1e-6


def base_deg_to_servo(q1):
    """Nghịch đảo base_servo_to_deg (không tính bù rơ - firmware tự bù) - chỉ để hiển thị."""
    return rm.base_knots_to_servo(q1, rm.BASE_DEG_AT_SERVO0, rm.BASE_DEG_AT_SERVO180)


def clamp_pose(q4):
    lims = (rm.LIM_BASE, rm.LIM_SHOULDER, rm.LIM_ELBOW, rm.LIM_GRIPPER)
    return tuple(min(max(float(v), lo), hi) for v, (lo, hi) in zip(q4, lims))


def pose_cmd(q4, ms):
    return "POSE %.1f %.1f %.1f %.1f %d" % (tuple(q4) + (int(ms),))


# =========================================================================
# LỊCH SỬ HOẠT ĐỘNG (history.csv)
# =========================================================================
HISTORY_COLS = ("thoi_gian", "loai", "khay", "mau", "ket_qua", "duong_di", "thuat_toan", "so_vat_can", "lan_blocked",
                "thoi_luong_s", "chi_tiet")


class ActivityHistory:
    """LỊCH SỬ HOẠT ĐỘNG: mỗi chu trình phân loại (1 dòng lúc kết thúc) + lỗi / cảnh báo.
    Ghi nối vào file CSV (UTF-8 có BOM: Excel đọc đúng dấu), giữ qua các lần chạy; `keep` dòng mới nhất nằm
    trong RAM để gửi cho trình duyệt.
    Chu trình bắt đầu ở REQ đầu tiên của một quả; các REQ trả BLOCKED rồi hỏi lại được gộp (lan_blocked).
    Kết thúc: "[AUTO] Da ve HOME" -> HOAN TAT | "[AUTO LOI]" / mất kết nối / ESP32 khởi động lại / E-Stop -> LOI |
    tắt AUTO giữa chừng -> DUNG. Lỗi / cảnh báo giống hệt nhau trong DEDUP_S giây chỉ ghi 1 lần."""
    DEDUP_S = 10.0
    ERROR_PREFIXES = ("[AUTO LOI]", "[IK LOI]", "[LOI]", "[PC LOI]", "[MAT KET NOI]", "[TIMEOUT]")
    WARN_PREFIXES = ("[PC] Bo qua", "[PC] Khong tra loi", "[RUN] Bo qua", "[CANH BAO]", RESET_LINE)

    def __init__(self, path, keep=500):
        self.path = path
        self.lock = threading.RLock()
        self.rows = collections.deque(maxlen=keep)
        self.seq = 0
        self.gen = 0          # Tăng mỗi lần xóa -> trình duyệt tải lại cả bảng
        self.cur = None       # Chu trình đang chạy
        self._last_evt = {}
        try:
            with open(path, encoding="utf-8-sig", newline="") as f:
                for row in csv.DictReader(f):
                    self._push({c: row.get(c) or "" for c in HISTORY_COLS})
        except (OSError, csv.Error, UnicodeDecodeError):
            pass

    def _push(self, row):
        self.seq += 1
        row = dict(row, id=self.seq)
        self.rows.append(row)
        return row

    def _write(self, **kw):
        row = {c: "" for c in HISTORY_COLS}
        row.update({k: ("" if v is None else v) for k, v in kw.items()})
        row["thoi_gian"] = time.strftime("%Y-%m-%d %H:%M:%S")
        with self.lock:
            try:  # utf-8-sig chỉ ghi BOM khi file mới (vị trí 0)
                with open(self.path, "a", encoding="utf-8-sig", newline="") as f:
                    w = csv.DictWriter(f, fieldnames=HISTORY_COLS)
                    if f.tell() == 0:
                        w.writeheader()
                    w.writerow(row)
            except OSError as e:
                print(f"[LICH SU] Khong ghi duoc {self.path}: {e}")
            return self._push(row)

    def since(self, last_id):
        with self.lock:
            return [r for r in self.rows if r["id"] > last_id]

    def recent(self):
        with self.lock:
            return list(self.rows)

    def clear(self):
        with self.lock:
            self.rows.clear()
            self.gen += 1
            self._last_evt.clear()
            try:
                os.remove(self.path)
            except OSError:
                pass

    def csv_text(self):
        """Nội dung CSV để tải về (có BOM)."""
        with self.lock:
            try:
                with open(self.path, encoding="utf-8-sig", newline="") as f:
                    return "﻿" + f.read()
            except OSError:
                return "﻿" + ",".join(HISTORY_COLS) + "\r\n"

    def event(self, loai, text):
        """Lỗi ("LOI") / cảnh báo ("CANH BAO") lẻ, bỏ trùng trong DEDUP_S giây."""
        with self.lock:
            now = time.monotonic()
            if now - self._last_evt.get((loai, text), -1e9) < self.DEDUP_S:
                return None
            self._last_evt[(loai, text)] = now
            return self._write(loai=loai, chi_tiet=text)

    def on_req(self, bin_id, kind, algo, n_obs, reason):
        with self.lock:
            c = self.cur
            if c is not None and (c["bin"] != bin_id or c["kind"] != BLOCKED):
                self._finish("LOI", "Chu trinh truoc chua ket thuc (khong thay '[AUTO] Da ve HOME')")
                c = None
            if c is None:
                c = self.cur = {"t0": time.time(), "bin": bin_id, "blocked": 0}
            c.update(kind=kind, algo=algo, n_obs=n_obs, reason=reason)
            if kind == BLOCKED:
                c["blocked"] += 1

    def _finish(self, result, detail=""):
        c, self.cur = self.cur, None
        if c is None:
            return None
        return self._write(loai="CHU TRINH", khay=c["bin"], mau=BIN_LABEL.get(c["bin"], ""), ket_qua=result,
                           duong_di=c["kind"], thuat_toan=c["algo"], so_vat_can=c["n_obs"], lan_blocked=c["blocked"],
                           thoi_luong_s="%.1f" % (time.time() - c["t0"]), chi_tiet=detail or c["reason"])

    def finish(self, result, detail=""):
        with self.lock:
            return self._finish(result, detail)

    def on_line(self, line):
        """Dòng ESP32 gửi lên -> kết thúc chu trình / ghi lỗi, cảnh báo."""
        with self.lock:
            if line.startswith("[AUTO] Da ve HOME"):
                if self.cur is not None and self.cur["kind"] != BLOCKED:
                    self._finish("HOAN TAT")
            elif line.startswith(("[AUTO LOI]", "[MAT KET NOI]", RESET_LINE)):
                self._finish("LOI", line)
            elif line.startswith(("[AUTO] TAT", "[AUTO] Da tat")):
                self._finish("DUNG", "Tat AUTO giua chu trinh")
            if line.startswith(self.ERROR_PREFIXES):
                self.event("LOI", line)
            elif line.startswith(self.WARN_PREFIXES):
                self.event("CANH BAO", line)


# =========================================================================
# ẢNH CAMERA CHO WEB (MJPEG): ảnh thật + lớp vẽ giống cửa sổ 06
# =========================================================================
def jpeg(img, quality=75):
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, quality])
    return buf.tobytes() if ok else b""


def placeholder_jpeg(text, w=640, h=360):
    img = np.full((h, w, 3), 38, dtype=np.uint8)
    cv2.putText(img, text, (24, h // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (220, 220, 220), 2, cv2.LINE_AA)
    return jpeg(img)


def put_label(img, text, org, color, scale=0.55):
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)


# =========================================================================
# 2. BỘ ĐIỀU KHIỂN
# =========================================================================
class Controller:
    def __init__(self):
        self.cfg = load_config()
        self.cfg_start = dict(self.cfg)
        if self.cfg.get("planner") not in PLANNER_LABEL:
            self.cfg["planner"] = "co_dinh"
        self.margin = float(self.cfg.get("safe_margin_mm", 22.0))
        self.z_margin = float(self.cfg.get("safe_z_margin_mm", 15.0))
        self.lock = threading.RLock()
        self.plan_lock = threading.Lock()   # Cache của planner không an toàn đa luồng
        self.log = collections.deque(maxlen=300)
        self.log_seq = 0

        self.link, self.connecting, self.mark = None, False, 0
        self.resets, self.pc_wait_ms = 0, proto.PC_TIMEOUT_MS
        self.tracker = CycleTracker()
        self.q, self.grip = tuple(rm.POSE_HOME), rm.GRIPPER_OPEN  # Góc khớp đã biết (lệnh vừa gửi / FK)
        self.estop = self.auto = self.playing = False
        self.seq = collections.deque()      # Chuỗi lệnh chờ "[DONE]" (Teach)
        self.seq_t = 0.0
        self.counts = {1: 0, 2: 0, 3: 0}
        self.cycle_bin = None               # Khay của chu trình đang chạy
        self.color_bin, self.color_txt = None, ""
        self.plan, self.last_req = None, None  # Kế hoạch REQ gần nhất (vẽ lên ảnh camera)
        self.joint_target, self._joint_sent = None, 0.0
        self.teach = self._load_teach()
        self.history = ActivityHistory(HISTORY_FILE)

        self.detector = ColorObstacleDetector(self.cfg)
        self.obstacles, self.cam_ok = [], False
        self.ready = (False, "CHUA CO KHUNG HINH")
        self._want_lock = False
        self._running = True
        self._jpeg = None          # Ảnh camera + lớp vẽ mới nhất (JPEG)
        self._jpeg_until = 0.0     # Có người xem ảnh camera tới lúc này -> luồng camera mới vẽ
        self.use_camera = has_robot_calib(self.cfg)
        if self.use_camera:
            threading.Thread(target=self._camera_loop, daemon=True).start()
        else:
            self.add_log("calib_config.json chua co ma tran hieu chuan camera (M_px2mm) -> REQ luon tra BLOCKED", "err")
            self.ready = (False, "CHUA HIEU CHUAN CAMERA")

        print("[PLANNER] Dang tinh san cac phuong an ne (ca 2 thuat toan)...")
        planners.warm_up()
        self.connect()

    # ------------------------------------------------------------------ log
    def add_log(self, text, kind="sys"):
        with self.lock:
            self.log_seq += 1
            self.log.append({"id": self.log_seq, "t": time.strftime("%H:%M:%S"), "kind": kind, "text": text})
        print(f"[WEB] {ascii_text(text)}")

    def logs_since(self, last_id):
        with self.lock:
            return [e for e in self.log if e["id"] > last_id]

    # ------------------------------------------------------------------ kết nối
    def connect(self, port=None):
        if port:
            self.cfg["serial_port"] = port
        self.disconnect()
        port = self.cfg.get("serial_port", "COM3")
        with self.lock:
            self.connecting = True
        self.add_log(f"Dang mo {port} (ESP32 khoi dong lai ~2.5s)...")

        def work():  # RobotLink() chặn 2.5s -> không chạy ở luồng của socket
            link = RobotLink(port, int(self.cfg.get("baud", 115200)))
            with self.lock:
                self.connecting = False
                if not link.ok:
                    self.add_log(f"Khong ket noi duoc {port}: {link.err}", "err")
                    return
                self.link, self.mark, self.resets = link, link.n_lines, link.resets
                self.tracker.reset()
                self.auto, self.cycle_bin = False, None
                self.seq.clear()
            link.set_req_handler(lambda b, l=link: self._on_req(b, l))
            self._hello(link)
            self.add_log(f"Da ket noi {port}", "ok")

        threading.Thread(target=work, daemon=True).start()

    def _hello(self, link):
        """Giống main.py cũ: tắt AUTO, cho PC thời gian trả lời rộng hơn, hỏi tư thế hiện tại."""
        link.send("AUTO OFF")
        ok = bool(link.ok and link.send(f"PCWAIT {PC_WAIT_MS}"))
        self.pc_wait_ms = PC_WAIT_MS if ok else proto.PC_TIMEOUT_MS
        link.send("FK")

    def disconnect(self):
        link, self.link = self.link, None
        if link is not None:
            if link.ok:
                link.send(f"PCWAIT {proto.PC_TIMEOUT_MS}")  # Trả lại mặc định cho lúc chạy không có PC
            link.close()

    # ------------------------------------------------------------------ REQ (luồng riêng của link)
    def _on_req(self, bin_id, link):
        t_req = time.perf_counter()
        with self.lock:
            obstacles = list(self.obstacles)
            ready, why = self.ready
        with self.plan_lock:
            p = answer_req(bin_id, obstacles, ready, why, self.cfg, self.margin, self.z_margin)
        sent = link.send(p.reply)
        ms = (time.perf_counter() - t_req) * 1000.0
        note = "" if sent else " | KHONG GUI DUOC"
        if ms > self.pc_wait_ms:
            note += f" | TRA LOI MUON ({ms:.0f}ms > {self.pc_wait_ms}ms, ESP32 da tu di duong mac dinh)"
        with self.lock:
            self.plan = p
            self.last_req = {"bin": bin_id, "kind": p.kind, "reason": p.reason, "algo": p.algo, "ms": round(ms)}
            if p.kind != proto.BLOCKED and not self.estop and link is self.link:
                self.cycle_bin = bin_id  # Đếm quả khi thấy "[AUTO] Da ve HOME"
        self.add_log(f"REQ {bin_id} [{p.algo}] -> {p.reply[:48]} | {p.reason} | {ms:.0f}ms{note}",
                     "err" if (note or p.kind == proto.BLOCKED) else "ok")
        if link is self.link:
            self.history.on_req(bin_id, p.kind, p.algo, len(obstacles), p.reason)
            if note:
                self.history.event("CANH BAO", f"REQ {bin_id}:{note.replace(' | ', ' ')}")
            if "LOI PLANNER" in p.reason:
                self.history.event("LOI", p.reason)

    # ------------------------------------------------------------------ camera (luồng riêng)
    def _camera_loop(self):
        cap = open_camera(self.cfg)
        if not cap.isOpened():
            self.add_log(f"Khong mo duoc camera {self.cfg.get('cam_id', 0)} -> REQ tra BLOCKED", "err")
        was_ok, t_draw = None, 0.0
        while self._running:
            cam_ok, raw = read_frame(cap, self.cfg)
            if was_ok and not cam_ok:
                self.history.event("LOI", "Mat camera (khong doc duoc khung hinh)")
            was_ok = cam_ok
            if not cap.isOpened():
                time.sleep(0.2)
            if self._want_lock:
                self._want_lock = False
                if cap.isOpened():
                    self.detector.start_lock_camera(cap)
            obstacles = self.detector.update(raw, cam_ok)
            ready, why = self.detector.data_ok() if cam_ok else (False, "MAT CAMERA")
            ready = (bool(ready), ascii_text(why))
            with self.lock:
                self.cam_ok, self.obstacles, self.ready = cam_ok, obstacles, ready
            now = time.monotonic()
            if now < self._jpeg_until and now - t_draw >= 1.0 / CAM_VIEW_FPS:
                t_draw = now  # Vẽ ở luồng camera: bộ nhận diện chỉ bị đọc ở 1 luồng
                self._jpeg = self._render_camera(raw, cam_ok, obstacles, ready)
            time.sleep(0.02)
        cap.release()

    # ------------------------------------------------------------------ vòng cập nhật (~20 Hz)
    def tick(self):
        link = self.link
        if link is None:
            return
        new_lines, self.mark = link.lines_since(self.mark)
        for line in new_lines:
            self.tracker.feed(line)
            self._handle_line(link, line)
        if link.resets != self.resets:  # ESP32 khởi động lại -> AUTO bật lại, PCWAIT về 500ms -> gửi lại
            self.resets = link.resets
            self._hello(link)
            self.add_log(f"ESP32 khoi dong lai -> gui lai AUTO OFF / PCWAIT {PC_WAIT_MS}", "err")
        with self.lock:
            if self.joint_target is not None and time.monotonic() - self._joint_sent >= JOINT_SEND_S:
                self._send_pose(link, self.joint_target, JOINT_MOVE_MS)
                self.joint_target, self._joint_sent = None, time.monotonic()
            if self.playing and time.monotonic() - self.seq_t > DONE_TIMEOUT_S:
                self.seq.clear()
                self.playing = False
                self.add_log("[TIMEOUT] Khong nhan duoc [DONE] -> huy chuoi lenh", "err")

    def _send_pose(self, link, q4, ms):
        q4 = clamp_pose(q4)
        if link.send(pose_cmd(q4, ms)):
            self.q, self.grip = tuple(q4[:3]), q4[3]  # Góc đích; xong động tác web gửi FK để lấy góc thật
            self.auto = False  # Lệnh tay tắt AUTO trong firmware

    def _handle_line(self, link, line):
        self.add_log(line, "err" if line.startswith(ALERT_PREFIXES) else "rx")
        self.history.on_line(line)
        with self.lock:
            if line.startswith("[DONE]"):
                if self.seq:
                    self.seq_t = time.monotonic()
                    self._send_pose(link, *self.seq.popleft())
                else:
                    self.playing = False
                    link.send("FK")  # Lấy lại góc khớp thật
                return
            if line.startswith(("[LOI]", "[IK LOI]")):
                self.seq.clear()
                self.playing = False
            m = FK_RE.match(line)
            if m:
                self.q = tuple(float(v) for v in m.groups())
                return
            m = COLOR_RE.match(line)
            if m:
                self.color_bin = GROUP_TO_BIN.get(m.group(1))
                self.color_txt = f"{m.group(1)} ({m.group(2)}/{m.group(3)})"
                return
            if line.startswith("[AUTO] BAT"):
                self.auto = True
            elif line.startswith(("[AUTO] TAT", "[AUTO] Da tat")):
                self.auto = False
            if line.startswith(("[EXEC] Ve HOME", "[AUTO] BAT", "[RUN] Ve HOME", "[AUTO] Da ve HOME")):
                self.q, self.grip = tuple(rm.POSE_HOME), rm.GRIPPER_OPEN
            if line.startswith("[AUTO] Da ve HOME"):
                if self.cycle_bin is not None:
                    self.counts[self.cycle_bin] += 1
                    self.cycle_bin = None
                self.color_bin, self.color_txt = None, ""
            elif line.startswith(("[AUTO LOI]", RESET_LINE, "[MAT KET NOI]")):
                self.cycle_bin = None

    # ------------------------------------------------------------------ lệnh từ trình duyệt
    def command(self, msg):
        """msg = {"cmd": ...}. Trả về chuỗi cảnh báo ("" = không có gì)."""
        cmd = msg.get("cmd")
        handler = getattr(self, "_cmd_" + str(cmd), None)
        if handler is None:
            return f"Lenh khong ho tro: {cmd}"
        return handler(msg) or ""

    def _need_link(self):
        if self.connecting:
            return "Dang ket noi ESP32..."
        if self.link is None or not self.link.ok:
            return "Chua ket noi ESP32 (chon cong COM -> Ket noi)"
        if self.estop:
            return "E-Stop dang bat"
        return ""

    def _cmd_connect(self, msg):
        self.connect(port=msg.get("port"))

    def _cmd_disconnect(self, msg):
        self.disconnect()
        self.add_log("Da ngat ket noi")

    def _cmd_estop(self, msg):
        link = self.link
        with self.lock:
            self.estop, self.auto, self.playing, self.joint_target = True, False, False, None
            self.seq.clear()
        if link is not None:
            link.send("AUTO OFF")
        self.history.finish("LOI", "E-STOP giua chu trinh")
        self.history.event("CANH BAO", "E-STOP: da gui AUTO OFF")
        self.add_log("!!! EMERGENCY STOP: da gui AUTO OFF. Firmware khong co lenh dung khan: tay di not dong tac "
                     "dang chay -> nguy hiem thi NGAT NGUON SERVO.", "err")

    def _cmd_history_clear(self, msg):
        self.history.clear()
        self.add_log("Da xoa lich su hoat dong")

    def _cmd_estop_reset(self, msg):
        with self.lock:
            self.estop = False
        self.add_log("Da reset E-Stop. Nen bam Home truoc khi chay tiep.", "err")
        if self.link is not None:
            self.link.send("FK")

    def _cmd_joint(self, msg):
        err = self._need_link()
        if err or self.playing or self.auto:
            return err or "Dang khoa (AUTO / phat lai)"
        key = msg.get("key")
        if key not in JOINT_LIMITS:
            return "Khop khong hop le"
        try:
            servo = float(msg.get("servo"))
        except (TypeError, ValueError):
            return "Goc khong hop le"
        if not math.isfinite(servo):
            return "Goc khong hop le"
        val, s, clipped = servo_to_pose(key, servo)
        with self.lock:
            q4 = list(self.joint_target or (self.q + (self.grip,)))
            q4[JOINT_KEYS.index(key)] = val
            self.joint_target = tuple(q4)
        if clipped:
            lo, hi = JOINT_LIMITS[key]
            return f"Gioi han firmware {lo:.0f}-{hi:.0f} do -> cat ve {s:.1f}"

    def _cmd_home(self, msg):
        err = self._need_link()
        if err:
            return err
        with self.lock:
            self.seq.clear()
            self.playing = False
        self.link.send("HOME")

    def _cmd_auto(self, msg):
        err = self._need_link()
        if err:
            return err
        on = bool(msg.get("on"))
        self.link.send("AUTO ON" if on else "AUTO OFF")
        with self.lock:
            self.auto = on

    def _cmd_fruit(self, msg):
        err = self._need_link()
        if err:
            return err
        b = int(msg.get("bin", 1))
        if b not in rm.BINS:
            return "Khay 1..3"
        if self.tracker.is_busy():
            return "Robot dang chay chu trinh -> chua gui RUN (doi '[AUTO] Da ve HOME')"
        if self.link.send(f"RUN {b}"):
            self.tracker.sent_run()

    def _cmd_lock_camera(self, msg):
        if not self.use_camera:
            return "Camera chua hieu chuan"
        self._want_lock = True
        self.add_log("KHOA CAMERA: tay o HOME, ban da don (khoa phoi sang / mau de nhan mau vat on dinh)")

    def _cmd_toggle_planner(self, msg):
        with self.lock:
            self.cfg["planner"] = "astar" if self.cfg["planner"] == "co_dinh" else "co_dinh"

    def _cmd_height(self, msg):
        with self.lock:
            nxt = [h for h in HEIGHT_OPTIONS if h > self.cfg.get("obs_height_mm", 150.0)]
            self.cfg["obs_height_mm"] = nxt[0] if nxt else HEIGHT_OPTIONS[0]

    def _cmd_raw(self, msg):
        err = self._need_link()
        line = str(msg.get("line", "")).strip()
        if err:
            return err
        if line:
            self.link.send(line)

    # ---- Teach ----
    def _load_teach(self):
        try:
            with open(TEACH_FILE, encoding="utf-8") as f:
                data = json.load(f)
            return [q for q in (self._valid_point(p) for p in data.get("points", [])) if q]
        except (OSError, ValueError, AttributeError, TypeError):
            return []

    @staticmethod
    def _valid_point(p):
        try:
            q = clamp_pose([p[k] for k in JOINT_KEYS])
        except (KeyError, TypeError, ValueError):
            return None
        return dict(zip(JOINT_KEYS, (round(v, 1) for v in q))) if all(math.isfinite(v) for v in q) else None

    def _save_teach(self):
        try:
            with open(TEACH_FILE, "w", encoding="utf-8") as f:
                json.dump({"note": "base = goc THUC (do), shoulder/elbow/gripper = goc servo", "points": self.teach},
                          f, indent=2)
        except OSError as e:
            self.add_log(f"Khong luu duoc {TEACH_FILE}: {e}", "err")

    def _cmd_teach_add(self, msg):
        with self.lock:
            p = self._valid_point(dict(zip(JOINT_KEYS, self.q + (self.grip,))))
            self.teach.append(p)
            self._save_teach()
            n = len(self.teach)
        self.add_log(f"[TEACH] Luu P{n}: de {p['base']} vai {p['shoulder']} khuyu {p['elbow']} kep {p['gripper']}")

    def _cmd_teach_del(self, msg):
        with self.lock:
            i = int(msg.get("index", -1))
            if 0 <= i < len(self.teach):
                self.teach.pop(i)
                self._save_teach()

    def _cmd_teach_clear(self, msg):
        with self.lock:
            self.teach = []
            self._save_teach()

    def _cmd_teach_set(self, msg):
        pts = [q for q in (self._valid_point(p) for p in (msg.get("points") or [])) if q]
        with self.lock:
            self.teach = pts
            self._save_teach()
        return f"Da nhap {len(pts)} diem"

    def _cmd_teach_go(self, msg):
        err = self._need_link()
        if err:
            return err
        i = int(msg.get("index", -1))
        if not 0 <= i < len(self.teach):
            return "Khong co diem nay"
        with self.lock:
            self._send_pose(self.link, [self.teach[i][k] for k in JOINT_KEYS], int(msg.get("ms", 1200)))

    def _cmd_teach_play(self, msg):
        err = self._need_link()
        if err or self.auto:
            return err or "Tat AUTO truoc"
        ms = max(200, min(10000, int(msg.get("ms", 1200))))
        with self.lock:
            if not self.teach:
                return "Chua co diem nao"
            self.seq = collections.deque(([p[k] for k in JOINT_KEYS], ms) for p in self.teach)
            self.playing, self.seq_t = True, time.monotonic()
            self._send_pose(self.link, *self.seq.popleft())  # Lệnh sau gửi khi thấy "[DONE]"
        self.add_log(f"Phat lai {len(self.teach)} diem")

    # ------------------------------------------------------------------ trạng thái cho trình duyệt
    @staticmethod
    def layout():
        return {
            "limits": {k: list(v) for k, v in JOINT_LIMITS.items()},
            "bins": {b: {"deg": rm.BINS[b]["deg"], "name": BIN_LABEL[b], "color": BIN_COLOR[b]} for b in rm.BINS},
        }

    def snapshot(self):
        with self.lock:
            link = self.link
            q, grip = self.q, self.grip
            tip = rm.fk(*q)
            h = float(self.cfg.get("obs_height_mm", 150.0))
            ready, why = self.ready
            return {
                "connected": bool(link is not None and link.ok), "connecting": self.connecting,
                "port": self.cfg.get("serial_port"), "err": getattr(link, "err", None),
                "q": [round(float(v), 2) for v in q], "grip": round(float(grip), 1),
                "servo": [round(base_deg_to_servo(q[0]), 1), round(q[1], 1), round(q[2], 1), round(float(grip), 1)],
                "tip": [round(v, 1) for v in tip], "r": round(math.hypot(tip[0], tip[1]), 1),
                "busy": self.tracker.is_busy(), "auto": self.auto, "estop": self.estop, "playing": self.playing,
                "color_bin": self.color_bin, "color_txt": self.color_txt, "counts": dict(self.counts),
                "obstacles": [{"x": round(float(o["x_mm"]), 1), "y": round(float(o["y_mm"]), 1),
                               "r": round(float(o["r_mm"]), 1)} for o in self.obstacles],
                "obs_h": h, "obs_h_label": "2D (vo han)" if h >= 9999 else f"{h:.0f}mm", "margin": self.margin,
                "use_camera": self.use_camera, "cam_ok": self.cam_ok, "cam_ready": bool(ready), "cam_why": why,
                "locking": self.detector.is_capturing(),
                "planner": self.cfg["planner"], "last_req": self.last_req,
                "teach": [dict(p, base_servo=round(base_deg_to_servo(p["base"]), 1)) for p in self.teach],
            }

    # ------------------------------------------------------------------ ảnh camera cho web
    def camera_jpeg(self):
        """Luồng MJPEG gọi liên tục: báo 'đang có người xem' trong 2s tới, trả ảnh JPEG mới nhất."""
        self._jpeg_until = time.monotonic() + 2.0
        if not self.use_camera:
            return placeholder_jpeg("CHUA HIEU CHUAN CAMERA (calib_config.json)")
        return self._jpeg or placeholder_jpeg("DANG MO CAMERA...")

    def _render_camera(self, raw, cam_ok, obstacles, ready):
        """Ảnh thật + vùng quét, cung R_PATH / khay, đường đi của kế hoạch REQ gần nhất, vật cản + lề an toàn."""
        cfg = self.cfg
        frame = raw.copy()
        with self.lock:
            plan = self.plan
        try:
            draw_detection_zone(frame, self.detector)
            if plan is not None:
                for res in plan.results:
                    col = (0, 0, 255) if res["hit"] else ((0, 255, 0) if res["move"].carrying else (0, 200, 120))
                    draw_polyline_mm(frame, cfg, res["tip_path"], col, 2)
                for r, deg, _z in plan.waypoints:
                    cv2.drawMarker(frame, xy_to_uv(*rm.polar_to_xyz(r, deg, 0)[:2], cfg), (255, 0, 255),
                                   cv2.MARKER_DIAMOND, 14, 2)
            draw_robot_layout(frame, cfg)
            for ob in obstacles:
                u, v = ob["u"], ob["v"]
                safe_px = int(round((ob["r_mm"] + self.margin) / local_mm_per_px(cfg, u, v)))
                cv2.circle(frame, (u, v), ob["r_px"], (0, 200, 255), 2, cv2.LINE_AA)
                cv2.circle(frame, (u, v), safe_px, (0, 200, 255), 1, cv2.LINE_AA)
        except Exception as e:  # noqa: BLE001 - lỗi vẽ không được làm dừng luồng camera
            put_label(frame, f"Loi ve: {type(e).__name__}", (10, 48), (0, 0, 255))
        if not cam_ok:
            txt, col = "MAT CAMERA", (0, 0, 255)
        elif not ready[0]:
            txt, col = f"CAMERA CHUA SAN SANG: {ready[1]}", (0, 220, 255)
        else:
            txt, col = f"{len(obstacles)} vat can camera thay", (0, 255, 0)
        put_label(frame, txt, (10, 24), col)
        return jpeg(frame)

    def close(self, save=True):
        self._running = False
        self.disconnect()
        changes = {k: self.cfg[k] for k in SAVE_KEYS if k in self.cfg and self.cfg.get(k) != self.cfg_start.get(k)}
        if save and changes:
            update_config(changes)  # Chỉ lưu khóa đã đổi bằng nút, đọc lại file trước khi ghi
        return changes if save else {}


# =========================================================================
# 3. SERVER WEB
# =========================================================================
app = Flask(__name__, template_folder=os.path.join(BASE_DIR, "web", "templates"),
            static_folder=os.path.join(BASE_DIR, "web", "static"))
socketio = SocketIO(app, async_mode="threading")
ctrl = None


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/camera.mjpg")
def camera_mjpg():
    """Ảnh camera thật + lớp vẽ, dạng MJPEG (thẻ <img> của trình duyệt tự hiển thị). Đóng tab là dừng."""
    def frames():
        last = None
        while True:
            jpg = ctrl.camera_jpeg()
            if jpg is not last:
                last = jpg
                yield (b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(jpg)).encode()
                       + b"\r\n\r\n" + jpg + b"\r\n")
            time.sleep(1.0 / CAM_VIEW_FPS)

    return Response(frames(), mimetype="multipart/x-mixed-replace; boundary=frame",
                    headers={"Cache-Control": "no-store"})


@app.route("/history.csv")
def history_csv():
    return Response(ctrl.history.csv_text(), mimetype="text/csv; charset=utf-8",
                    headers={"Content-Disposition": "attachment; filename=lich_su_hoat_dong.csv"})


@socketio.on("connect")
def on_connect():
    emit("layout", ctrl.layout())
    emit("ports", list_ports())
    emit("log", ctrl.logs_since(0)[-120:])
    emit("history", {"reset": True, "rows": ctrl.history.recent()})
    emit("state", ctrl.snapshot())


@socketio.on("cmd")
def on_cmd(msg):
    if not isinstance(msg, dict):
        return
    try:
        warn = ctrl.command(msg)
    except Exception as e:  # noqa: BLE001 - lỗi 1 lệnh không được làm sập server
        warn = f"Loi xu ly lenh: {type(e).__name__}: {e}"
        ctrl.add_log(warn, "err")
    emit("ack", {"cmd": msg.get("cmd"), "warn": warn})


@socketio.on("ports")
def on_ports():
    emit("ports", list_ports())


def push_loop(hz=20):
    last_log = ctrl.log_seq
    hist = ctrl.history
    last_hist, hist_gen = hist.seq, hist.gen
    while True:
        socketio.sleep(1.0 / hz)
        try:
            ctrl.tick()
            socketio.emit("state", ctrl.snapshot())
            new = ctrl.logs_since(last_log)
            if new:
                last_log = new[-1]["id"]
                socketio.emit("log", new)
            if hist.gen != hist_gen:  # Đã xóa lịch sử -> gửi lại cả bảng
                hist_gen, last_hist = hist.gen, hist.seq
                socketio.emit("history", {"reset": True, "rows": hist.recent()})
            else:
                rows = hist.since(last_hist)
                if rows:
                    last_hist = rows[-1]["id"]
                    socketio.emit("history", {"reset": False, "rows": rows})
        except Exception as e:  # noqa: BLE001
            print(f"[WEB] Loi vong cap nhat: {type(e).__name__}: {e}")


def main():
    global ctrl
    ap = argparse.ArgumentParser(description="Web dieu khien EEZYbotARM MK2 + ne vat can")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5000)
    args = ap.parse_args()
    ctrl = Controller()
    logging.getLogger("werkzeug").setLevel(logging.ERROR)  # Không in mỗi lần trình duyệt hỏi server
    socketio.start_background_task(push_loop)
    print(f"[WEB] Mo trinh duyet: http://localhost:{args.port}")
    try:
        socketio.run(app, host=args.host, port=args.port, allow_unsafe_werkzeug=True, use_reloader=False,
                     log_output=False)
    except KeyboardInterrupt:
        pass
    finally:
        changes = ctrl.close()
        if changes:
            print(f"[WEB] Da luu vao calib_config.json: {changes}")


if __name__ == "__main__":
    main()
