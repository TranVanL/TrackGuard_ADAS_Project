# 📡 03a — Sensors & Perception (Detection Model, Noise, Synthetic Generation)

> Đọc trước Phase 1-2 ([00_MASTER_PLAN.md](../00_MASTER_PLAN.md) P1-P2). Mục tiêu: hiểu detection thô từ sensor trông như thế nào, sai số ở đâu, và cách sinh dữ liệu radar synthetic hợp lý khi không có radar raw thật (KITTI không có).

---

## 1. Camera detection model

Camera cho ra **2D bounding box + class + confidence** trên ảnh, KHÔNG cho trực tiếp vị trí 3D/khoảng cách. Để có vị trí xấp xỉ trong world frame, cần:
- Biết chiều cao thật trung bình của class (vd xe con ~1.5m) + focal length + kích thước bbox trên ảnh → ước lượng khoảng cách (monocular depth xấp xỉ), HOẶC
- Dùng ground-truth 3D location có sẵn trong dataset (KITTI cung cấp `location` 3D thật) làm "đầu ra camera" xấp xỉ + cộng noise — đây là cách TrackGuard dùng (đơn giản hoá hợp lý, không lạc đề sang depth estimation)

**Đặc điểm camera cần nhớ:**
| Thuộc tính | Giá trị điển hình |
|---|---|
| Tần số | 10 Hz (giới hạn dataset KITTI) |
| Sai số vị trí ngang (lateral) | Thấp (~0.1-0.3m) — camera rất chính xác về góc |
| Sai số khoảng cách (longitudinal) | Cao hơn khi xa (monocular depth kém chính xác theo khoảng cách) |
| Vận tốc | KHÔNG đo trực tiếp — phải suy ra qua tracking (sai phân vị trí giữa các frame) |
| FOV | Hẹp hơn radar, thường ~60-120° |
| Class + confidence | Có — quan trọng cho gating/fusion (không match Vehicle-track với Pedestrian-detection) |

---

## 2. Radar detection model

Radar đo trực tiếp: **range** (khoảng cách), **azimuth** (góc), **range-rate** (vận tốc theo hướng xuyên tâm, qua hiệu ứng Doppler) — đây là điểm mạnh lớn nhất của radar so với camera.

| Thuộc tính | Giá trị điển hình |
|---|---|
| Tần số | 20 Hz |
| Sai số range | Thấp, ổn định theo khoảng cách (~±0.1-0.2m) |
| Sai số azimuth | Cao hơn camera (góc phân giải kém hơn, ~±0.5-1°) |
| Range-rate (vận tốc xuyên tâm) | Đo trực tiếp, rất chính xác (±0.1-0.2 m/s) — đây là lý do radar quan trọng cho TTC |
| FOV | Rộng hơn nhưng độ phân giải thấp hơn theo góc |
| Class | Thường KHÔNG có (radar không phân biệt tốt xe/người) |

---

## 3. Vì sao TTC nên ưu tiên dữ liệu radar

$$TTC = \frac{\text{range}}{\text{closing\_speed}}$$

`closing_speed` (range-rate) camera phải suy ra gián tiếp qua sai phân vị trí (nhiễu cao, trễ 1 chu kỳ), radar đo trực tiếp bằng Doppler → chính xác hơn nhiều. Đây là lý do thực tế nhiều hệ thống FCW/AEB production dựa nhiều vào radar cho quyết định longitudinal, và dùng camera cho classification + lateral precision + lane context.

---

## 4. Noise model — cách sinh nhiễu hợp lý cho synthetic sensor

Với mỗi detection thật (ground-truth), cộng nhiễu Gaussian độc lập theo trục:

```cpp
struct NoiseModel {
    double sigma_range;      // camera: lớn hơn theo khoảng cách; radar: gần const
    double sigma_azimuth;
    double sigma_range_rate; // chỉ radar có ý nghĩa trực tiếp
};

// Camera noise — tăng theo khoảng cách (mô phỏng depth estimation kém xa)
double sigma_range_camera(double range_m) {
    return 0.05 * range_m; // 5% khoảng cách, ví dụ 40m → sigma 2m
}

// Radar noise — gần như hằng số theo khoảng cách
constexpr double kRadarSigmaRange     = 0.15; // m
constexpr double kRadarSigmaAzimuthDeg = 0.6; // độ
constexpr double kRadarSigmaRangeRate = 0.15; // m/s
```

> **Seed cố định (`std::mt19937` với seed cụ thể)** để đảm bảo replay deterministic — bắt buộc cho scenario test tin cậy được (xem [03g_TESTING_VALIDATION.md](03g_TESTING_VALIDATION.md)).

---

## 5. Sinh Radar synthetic detection từ ground-truth KITTI (kỹ thuật dùng trong P2 Day 8)

KITTI không có radar raw, nên TrackGuard sinh "radar ảo" từ ground-truth 3D location + vận tốc suy ra:

```
1. Lấy ground-truth location(t), location(t-1) từ label file
2. velocity_gt = (location(t) - location(t-1)) / dt
3. range = |location(t) - ego_position(t)|
4. azimuth = atan2(lateral, longitudinal)
5. range_rate = -d(range)/dt  (chiếu velocity tương đối lên trục range)
6. Cộng noise theo NoiseModel radar ở mục 4
7. Output RawDetection { pos (từ range+azimuth), vel (từ range_rate), confidence≈0.9 }
```

Đây là kỹ thuật **sensor model synthesis** chuẩn, dùng phổ biến khi test ADAS software không có sensor thật — nói thẳng điều này trong interview là điểm cộng (thể hiện hiểu methodology test ADAS), không phải điểm trừ vì "không có radar thật".

---

## 6. FOV & Confidence Gating

- **FOV Gate**: loại detection ngoài góc nhìn hợp lệ của sensor (vd radar FOV ±60°, camera FOV ±40°) — object ngoài FOV không thể là detection thật, phải là artifact/nhiễu
- **Confidence Gate**: loại detection có confidence < ngưỡng (vd < 0.3) — giảm false-positive sớm từ Stage 3, trước khi vào Kalman Filter

## 7. Tài liệu tham khảo (đọc thêm, không bắt buộc)
- KITTI Tracking Benchmark — devkit README (định dạng label_02, oxts)
- "Radar and Camera Sensor Fusion" tổng quan (bất kỳ survey paper ADAS sensor fusion nào để hiểu đặc tính bổ sung 2 sensor)
