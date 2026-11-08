// app.js - phần trình duyệt của web.py: CHỈ vẽ và gửi nút bấm.
// Mọi tính toán (FK, góc đế thực, chu trình, tránh vật cản) nằm ở web.py; server đẩy "state" ~20 lần/giây.
const $ = id => document.getElementById(id);
const clamp = (v, a, b) => Math.min(Math.max(v, a), b);
const socket = io();
let L = null;          // layout: giới hạn khớp + khay từ server (robot_model.py)
let S = null;          // state mới nhất

// =========================================================================
// 1. TIỆN ÍCH GIAO DIỆN
// =========================================================================
function send(cmd, extra = {}) { socket.emit('cmd', { cmd, ...extra }); }
let toastTimer = 0;
function toast(msg, isErr = false) {
  const t = $('toast');
  t.textContent = msg; t.className = 'show' + (isErr ? ' err' : '');
  clearTimeout(toastTimer); toastTimer = setTimeout(() => t.className = '', 2800);
}
function setPill(id, text, cls) { const p = $(id); p.className = 'pill ' + cls; p.lastElementChild.textContent = text; }
function addLog(entries) {
  const box = $('log');
  const atEnd = box.scrollTop + box.clientHeight >= box.scrollHeight - 30;
  for (const e of entries) {
    const d = document.createElement('div');
    d.className = e.kind;
    d.textContent = `${e.t}  ${e.text}`;
    box.appendChild(d);
  }
  while (box.childElementCount > 400) box.firstChild.remove();
  if (atEnd) box.scrollTop = box.scrollHeight;
}

// =========================================================================
// 2. THANH TRƯỢT KHỚP
// =========================================================================
const JOINTS = [
  { key: 'base', name: 'Base', vn: 'Đế xoay · qua bánh răng' },
  { key: 'shoulder', name: 'Shoulder', vn: 'Vai' },
  { key: 'elbow', name: 'Elbow', vn: 'Khuỷu' },
  { key: 'gripper', name: 'Gripper', vn: 'Kẹp · 40 mở · 140 đóng' },
];
const dragging = {};
const warnTimers = {};
let lastJoint = null;
function buildJoints() {
  const box = $('joints');
  box.innerHTML = '';
  for (const j of JOINTS) {
    const [lo, hi] = L.limits[j.key];
    const track = `linear-gradient(90deg, #f6c9c9 0 ${lo / 1.8}%, #cfe0fb ${lo / 1.8}% ${hi / 1.8}%, #f6c9c9 ${hi / 1.8}% 100%)`;
    const el = document.createElement('div');
    el.className = 'joint';
    el.innerHTML = `
      <div class="top"><span class="name">${j.name}</span><span class="sub">${j.vn}</span><span class="sub" id="info_${j.key}"></span></div>
      <div class="ctl">
        <button data-k="${j.key}" data-d="-1" title="-1°">−</button>
        <input type="range" id="sl_${j.key}" min="0" max="180" step="0.5" style="--track:${track}">
        <button data-k="${j.key}" data-d="1" title="+1°">+</button>
        <input type="number" id="nb_${j.key}" min="0" max="180" step="0.5">
      </div>
      <div class="warn" id="warn_${j.key}"></div>`;
    box.appendChild(el);
    const sl = el.querySelector('input[type=range]'), nb = el.querySelector('input[type=number]');
    sl.addEventListener('pointerdown', () => dragging[j.key] = true);
    window.addEventListener('pointerup', () => { if (dragging[j.key]) setTimeout(() => dragging[j.key] = false, 400); });
    sl.addEventListener('input', () => { nb.value = sl.value; sendJoint(j.key, +sl.value); });
    nb.addEventListener('change', () => sendJoint(j.key, +nb.value));
    el.querySelectorAll('button').forEach(b => b.addEventListener('click', () => {
      const v = clamp(+sl.value + +b.dataset.d, 0, 180);
      sl.value = v; nb.value = v; sendJoint(j.key, v);
      dragging[j.key] = true; setTimeout(() => dragging[j.key] = false, 600);
    }));
  }
}
function sendJoint(key, servo) { lastJoint = key; send('joint', { key, servo }); }
function showJointWarn(key, msg) {
  const el = $('warn_' + key); if (!el) return;
  el.textContent = msg;
  clearTimeout(warnTimers[key]); warnTimers[key] = setTimeout(() => el.textContent = '', 2500);
}
function renderJoints() {
  JOINTS.forEach((j, i) => {
    const sl = $('sl_' + j.key), nb = $('nb_' + j.key);
    if (!dragging[j.key]) sl.value = S.servo[i];
    if (!dragging[j.key] && document.activeElement !== nb) nb.value = S.servo[i].toFixed(1);
    const info = j.key === 'base' ? `thực ${S.q[0].toFixed(1)}°` : j.key === 'gripper' ? (S.grip < 90 ? 'đang mở' : 'đang kẹp') : '';
    $('info_' + j.key).textContent = info;
  });
}

// =========================================================================
// 3. CẬP NHẬT THEO STATE
// =========================================================================
let teachKey = '';
function render() {
  if (!S || !L) return;
  renderJoints();
  const [x, y, z] = S.tip;
  $('tcp').innerHTML = `<b>Tâm kẹp (FK)</b><br>X ${x.toFixed(1).padStart(7)} mm<br>Y ${y.toFixed(1).padStart(7)} mm<br>` +
    `Z ${z.toFixed(1).padStart(7)} mm<br>r ${S.r.toFixed(1).padStart(7)} mm<br>θ ${S.q[0].toFixed(1).padStart(7)} °`;
  $('fsmTxt').textContent = S.estop ? 'E-STOP' : (!S.connected ? 'Chưa kết nối ESP32' : (S.busy ? 'Đang chạy chu trình' : 'Rảnh'));
  document.body.classList.toggle('estop', S.estop);

  // Pill + kết nối
  setPill('pillRobot', S.connecting ? 'ESP32: đang mở cổng...' : `ESP32: ${S.port}${S.connected ? '' : ' (chưa kết nối)'}`,
    S.connecting ? 'warn' : (S.connected ? 'ok' : 'danger'));
  setPill('pillAuto', S.auto ? 'AUTO: đang chạy' : 'AUTO: tắt', S.auto ? 'ok' : 'off');
  $('btnDisconnect').disabled = !S.connected;

  // Nút
  const lockJ = S.estop || S.auto || S.playing || !S.connected;
  JOINTS.forEach(j => {
    $('sl_' + j.key).disabled = lockJ; $('nb_' + j.key).disabled = lockJ;
    document.querySelectorAll(`button[data-k=${j.key}]`).forEach(b => b.disabled = lockJ);
  });
  $('btnHome').disabled = S.estop || !S.connected;
  const ba = $('btnAuto'); ba.disabled = S.estop || S.playing || !S.connected; ba.classList.toggle('on', S.auto);
  ba.querySelector('span').textContent = S.auto ? 'Dừng Auto' : 'Auto Cycle';
  $('btnTeach').disabled = S.estop;
  $('btnPlay').disabled = S.estop || S.auto || S.playing || !S.teach.length || !S.connected;
  $('btnPlay').querySelector('span').textContent = S.playing ? 'Đang phát...' : 'Phát lại điểm';
  document.querySelectorAll('.fruitBtn').forEach(b => b.disabled = S.estop || !S.connected || S.busy);

  // Cảm biến
  const cb = S.color_bin;
  $('swatch').style.background = cb ? L.bins[cb].color : '#e9edf3';
  $('colorName').textContent = cb ? `${L.bins[cb].name} → Khay ${cb}` : 'Chưa có quả';
  $('colorRead').textContent = S.color_txt || '—';
  for (const b of [1, 2, 3]) $('cnt' + b).textContent = S.counts[b];
  $('cntT').textContent = S.counts[1] + S.counts[2] + S.counts[3];

  // Tránh vật cản
  $('btnPlanner').textContent = S.planner === 'astar' ? 'NÉ: A*' : 'NÉ: CỐ ĐỊNH';
  $('btnHeight').textContent = `CAO: ${S.obs_h_label}`;
  $('btnCapture').disabled = !S.use_camera || S.locking;
  $('btnCapture').textContent = S.locking ? 'Đang khóa camera...' : 'Khóa camera';
  let camTxt, camCls;
  if (!S.use_camera) { camTxt = 'Chưa hiệu chuẩn'; camCls = 'danger'; }
  else if (!S.cam_ok) { camTxt = 'Mất camera'; camCls = 'danger'; }
  else if (S.cam_ready) { camTxt = 'Sẵn sàng'; camCls = 'ok'; }
  else { camTxt = 'Chưa sẵn sàng'; camCls = 'warn'; }
  setPill('pillCam', `Camera: ${camTxt}`, camCls);
  let hint = S.cam_ready ? 'Camera sẵn sàng: REQ sẽ dùng vật cản camera thấy.' : `Camera: ${S.cam_why} → REQ trả BLOCKED.`;
  if (S.use_camera) hint += ' Nhận vật cản theo màu; tay ở HOME, bàn đã dọn rồi bấm Khóa camera.';
  $('camHint').textContent = hint;
  $('obsCount').textContent = `${S.obstacles.length} vật cản camera thấy · cao ${S.obs_h_label} · lề ${S.margin} mm`;
  const rq = S.last_req;
  const box = $('reqBox');
  box.className = 'reqBox' + (rq ? ' ' + rq.kind.toLowerCase() : '');
  $('reqKind').textContent = rq ? `REQ ${rq.bin} → ${rq.kind}` : 'Chưa có';
  $('reqMeta').textContent = rq ? `${rq.algo} · ${rq.ms} ms` : '';
  $('reqReason').textContent = rq ? rq.reason : 'ESP32 gửi REQ khi có quả; server tính đường né rồi trả CLEAR / PATH / BLOCKED.';

  // Teach
  const key = JSON.stringify(S.teach);
  if (key !== teachKey) { teachKey = key; renderTeach(); }
  document.querySelectorAll('#teachBody button').forEach(b => b.disabled = S.estop || S.auto || S.playing);
}
function renderTeach() {
  const tb = $('teachBody');
  tb.innerHTML = '';
  if (!S.teach.length) tb.innerHTML = '<tr><td colspan="6" class="empty">Chưa có điểm. Chỉnh tay tới vị trí rồi bấm Teach / Record.</td></tr>';
  S.teach.forEach((p, i) => {
    const tr = document.createElement('tr');
    tr.innerHTML = `<td>P${i + 1}</td><td>${p.base_servo.toFixed(1)}</td><td>${p.shoulder.toFixed(1)}</td><td>${p.elbow.toFixed(1)}</td>` +
      `<td>${p.gripper.toFixed(0)}</td><td class="act"><button class="small" data-go="${i}">Đi tới</button> <button class="small" data-del="${i}">✕</button></td>`;
    tb.appendChild(tr);
  });
  $('teachCount').textContent = `${S.teach.length} điểm`;
  tb.querySelectorAll('[data-go]').forEach(b => b.onclick = () => send('teach_go', { index: +b.dataset.go, ms: teachMs() }));
  tb.querySelectorAll('[data-del]').forEach(b => b.onclick = () => send('teach_del', { index: +b.dataset.del }));
}
const teachMs = () => clamp(+$('teachMs').value || 1200, 200, 10000);

// =========================================================================
// 4. SOCKET
// =========================================================================
socket.on('connect', () => { setPill('pillServer', 'Server: đã nối', 'ok'); showCamera(); });
socket.on('disconnect', () => setPill('pillServer', 'Server: mất kết nối', 'danger'));
socket.on('layout', lay => { L = lay; buildJoints(); });
socket.on('state', st => { S = st; render(); });
socket.on('log', addLog);
socket.on('ports', ports => {
  const sel = $('portSel'), cur = S?.port || sel.value;
  sel.innerHTML = '';
  const list = ports.length ? ports : [cur || 'COM3'];
  if (cur && !list.includes(cur)) list.unshift(cur);
  for (const p of list) { const o = document.createElement('option'); o.textContent = p; sel.appendChild(o); }
  if (cur) sel.value = cur;
});
socket.on('ack', a => {
  if (!a.warn) return;
  if (a.cmd === 'joint' && lastJoint) showJointWarn(lastJoint, a.warn);
  else toast(a.warn, /^(Tu choi|Khong|Loi|Dang khoa|E-Stop)/.test(a.warn));
});

// =========================================================================
// 4b. ẢNH CAMERA THẬT (MJPEG do server vẽ sẵn vùng quét, vật cản, đường né)
// =========================================================================
function showCamera() { $('camImg').src = '/camera.mjpg?t=' + Date.now(); }
$('camImg').onerror = () => setTimeout(showCamera, 2000);  // Server khởi động lại / mất luồng -> mở lại

// =========================================================================
// 4c. LỊCH SỬ HOẠT ĐỘNG
// =========================================================================
let HIST = [];
const RESULT_CLS = { 'HOAN TAT': 'ok', 'DUNG': 'warn', 'LOI': 'err' };
const TYPE_CLS = { 'CHU TRINH': 'info', 'CANH BAO': 'warn', 'LOI': 'err' };
const TYPE_TXT = { 'CHU TRINH': 'Chu trình', 'CANH BAO': 'Cảnh báo', 'LOI': 'Lỗi' };
const RESULT_TXT = { 'HOAN TAT': 'Hoàn tất', 'DUNG': 'Dừng', 'LOI': 'Lỗi' };
const esc = t => String(t ?? '').replace(/[&<>"]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
socket.on('history', h => {
  HIST = h.reset ? h.rows : HIST.concat(h.rows);
  if (HIST.length > 1000) HIST = HIST.slice(-1000);
  renderHistory();
});
function renderHistory() {
  const type = $('histType').value, day = $('histDate').value;
  const ofDay = HIST.filter(r => !day || String(r.thoi_gian).startsWith(day));
  const rows = ofDay.filter(r => !type || (type === 'LOI' ? r.loai !== 'CHU TRINH' : r.loai === type));
  // Thống kê theo ngày đang lọc
  const cyc = ofDay.filter(r => r.loai === 'CHU TRINH');
  const done = cyc.filter(r => r.ket_qua === 'HOAN TAT');
  const perBin = [1, 2, 3].map(b => done.filter(r => +r.khay === b).length);
  const nPath = done.filter(r => r.duong_di === 'PATH').length;
  const nBlocked = cyc.reduce((a, r) => a + (+r.lan_blocked || 0), 0);
  const avg = done.length ? done.reduce((a, r) => a + (+r.thoi_luong_s || 0), 0) / done.length : 0;
  const nErr = ofDay.filter(r => r.loai === 'LOI').length, nWarn = ofDay.filter(r => r.loai === 'CANH BAO').length;
  $('histStats').innerHTML =
    `<span class="chip">Hoàn tất <b>${done.length}</b>/${cyc.length}</span>` +
    `<span class="chip">Khay 1 <b>${perBin[0]}</b> · Khay 2 <b>${perBin[1]}</b> · Khay 3 <b>${perBin[2]}</b></span>` +
    `<span class="chip">Đi vòng né <b>${nPath}</b></span><span class="chip">BLOCKED <b>${nBlocked}</b> lần</span>` +
    `<span class="chip">TB <b>${avg.toFixed(1)}</b> s/quả</span>` +
    `<span class="chip">Lỗi <b>${nErr}</b> · Cảnh báo <b>${nWarn}</b></span>`;
  $('histCount').textContent = `${rows.length} dòng${day ? ' · ' + day : ''}`;
  const show = rows.slice(-200).reverse();  // mới nhất ở trên
  $('histBody').innerHTML = show.length ? show.map(r => `<tr>
      <td>${esc(String(r.thoi_gian).slice(day ? 11 : 5))}</td>
      <td><span class="badge ${TYPE_CLS[r.loai] || ''}">${esc(TYPE_TXT[r.loai] || r.loai)}</span></td>
      <td>${esc(r.khay)}</td>
      <td>${r.ket_qua ? `<span class="badge ${RESULT_CLS[r.ket_qua] || ''}">${esc(RESULT_TXT[r.ket_qua] || r.ket_qua)}</span>` : ''}</td>
      <td class="hideS">${esc(r.duong_di)}${r.thuat_toan ? ' · ' + esc(r.thuat_toan) : ''}</td>
      <td class="hideS">${esc(r.so_vat_can)}</td>
      <td>${r.thoi_luong_s ? esc(r.thoi_luong_s) + ' s' : ''}</td>
      <td class="det">${esc(r.chi_tiet)}${+r.lan_blocked ? ` (BLOCKED ${esc(r.lan_blocked)} lần)` : ''}</td></tr>`).join('')
    : '<tr><td colspan="8" class="empty">Chưa có hoạt động nào.</td></tr>';
}
const today = () => { const d = new Date(); d.setMinutes(d.getMinutes() - d.getTimezoneOffset()); return d.toISOString().slice(0, 10); };
$('histType').onchange = renderHistory;
$('histDate').onchange = renderHistory;
$('btnHistToday').onclick = () => { $('histDate').value = today(); renderHistory(); };
$('btnHistAll').onclick = () => { $('histDate').value = ''; renderHistory(); };
$('btnHistClear').onclick = () => { if (confirm('Xóa toàn bộ lịch sử hoạt động (cả file history.csv)? Không khôi phục được.')) send('history_clear'); };
$('histDate').value = today();

// ---- Nút ----
$('btnHome').onclick = () => send('home');
$('btnAuto').onclick = () => send('auto', { on: !S?.auto });
$('btnTeach').onclick = () => { send('teach_add'); toast('Đã lưu điểm hiện tại'); };
$('btnPlay').onclick = () => send('teach_play', { ms: teachMs() });
$('btnEstop').onclick = () => send('estop');
$('btnEstopReset').onclick = () => send('estop_reset');
document.querySelectorAll('.fruitBtn').forEach(b => b.onclick = () => send('fruit', { bin: +b.dataset.bin }));
$('btnCapture').onclick = () => send('lock_camera');
$('btnPlanner').onclick = () => send('toggle_planner');
$('btnHeight').onclick = () => send('height');
$('btnConnect').onclick = () => send('connect', { port: $('portSel').value });
$('btnDisconnect').onclick = () => send('disconnect');
$('btnPorts').onclick = () => socket.emit('ports');
$('btnClearLog').onclick = () => $('log').innerHTML = '';
const sendRaw = () => { const v = $('cmdInput').value.trim(); if (v) { send('raw', { line: v }); $('cmdInput').value = ''; } };
$('btnSendCmd').onclick = sendRaw;
$('cmdInput').addEventListener('keydown', e => { if (e.key === 'Enter') sendRaw(); });
$('btnClearTeach').onclick = () => { if (S?.teach.length && confirm('Xóa tất cả điểm đã lưu?')) send('teach_clear'); };
$('btnExport').onclick = () => {
  const blob = new Blob([JSON.stringify({ note: 'base = goc THUC (do), shoulder/elbow/gripper = goc servo', points: S?.teach || [] }, null, 2)], { type: 'application/json' });
  const a = document.createElement('a'); a.href = URL.createObjectURL(blob); a.download = 'mk2_teach_points.json'; a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 1000);
};
$('btnImport').onclick = () => $('fileImport').click();
$('fileImport').onchange = async e => {
  const file = e.target.files[0]; if (!file) return;
  try {
    const j = JSON.parse(await file.text());
    send('teach_set', { points: Array.isArray(j) ? j : (j.points || []) });
  } catch (err) { toast('File JSON không hợp lệ', true); }
  e.target.value = '';
};
