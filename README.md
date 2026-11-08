# Bản toàn Python: EEZYbotARM MK2 phân loại quả + né vật cản
## Các file

| File | Dòng | Chạy ở | Vai trò |
|---|---|---|---|
| `kinematics.py` | ~300 | **PC + ESP32** | Dùng chung: hằng số hình học / giới hạn, đế 3 mốc + bù rơ, FK/IK, cung đường (gắp, 3 khay), `leg_ms`, giao thức `REQ` / `CLEAR` / `PATH ... [H]` / `BLOCKED` |
| `esp32/main.py` | ~760 | ESP32 | Firmware MicroPython (thay `KINEMATICS.ino`): servo PWM, máy trạng thái gắp - thả, TCS3200, lệnh Serial, độ rơ lưu NVS |
| `web.py` | ~920 | PC | Web điều khiển (Flask): camera -> vật cản -> trả lời `REQ`, điều khiển khớp, Teach, E-Stop, lịch sử (thay `main.py` cũ) |
| `web/templates`, `web/static` | | Trình duyệt | Giao diện: chỉ vẽ và gửi nút bấm, không tính toán |
| `robot_model.py` | ~270 | PC | Các đoạn đi của chu trình (giống FSM firmware) + kiểm tra va chạm (numpy) |
| `planners.py` | ~570 | PC | Hai thuật toán né: "cố định" và A* |
| `esp32_link.py` | ~150 | PC | Kết nối Serial với ESP32 (luồng đọc, phát hiện ESP32 khởi động lại) |
| `vision_utils.py` | ~630 | PC | Camera, đổi pixel <-> mm, nhận vật cản theo màu, hàm vẽ |
| `calib_config.json` | | PC | Hiệu chuẩn camera + màu vật cản + cổng COM (chép từ `newcode`, 08/11/2026) |

## Chạy phía PC

1. `pip install -r requirements.txt`
2. - Thẻ *Kết nối ESP32*: chọn cổng COM -> **Kết nối** (web gửi `AUTO OFF`, `PCWAIT 1500`, `FK`).
   - Tay ở HOME, bàn đã dọn, bấm **Khóa camera**; chờ *Camera: Sẵn sàng* (chưa sẵn sàng -> REQ trả BLOCKED).
   - Đặt quả vào điểm gắp (bật **Auto Cycle**), hoặc bấm **Khay 1/2/3** (= `RUN n`: robot thật chạy 1 chu trình, không cần quả).
   - **NÉ** đổi CO DINH <-> A*, **CAO** là chiều cao giả định của vật cản.
   - Thanh trượt khớp gửi `POSE`; **Teach** lưu điểm vào `teach_points.json`.
   - **E-Stop** chỉ gửi `AUTO OFF` (firmware chưa có lệnh dừng khẩn): nguy hiểm thì ngắt nguồn servo.
   - Góc khớp hiển thị là góc ESP32 báo qua `FK` (web gửi `FK` sau mỗi động tác tay); lúc AUTO đang chạy chưa cập nhật.
   - Thẻ **Lịch sử hoạt động**: mỗi quả đã phân loại (khay, CLEAR/PATH/BLOCKED, số vật cản, thời gian) + lỗi / cảnh báo; lưu nối vào `history.csv` (mở bằng Excel), lọc theo loại / ngày, nút *Tải CSV* / *Xóa lịch sử*.
   - Tắt bằng Ctrl+C: NÉ / CAO / cổng COM được lưu vào `calib_config.json`.

Khi dời camera, đổi màu vật cản hoặc đổi hằng số đế: làm lại ở bản đầy đủ (`newcode`), chép `calib_config.json` sang đây, và sửa hằng số trong `kinematics.py` (rồi upload lại `kinematics.py` lên ESP32).