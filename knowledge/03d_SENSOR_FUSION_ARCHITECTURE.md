# 🔀 03d — Sensor Fusion Architecture (Time Sync, Ego-Motion, Calibration, Early vs Late Fusion)

> Đọc trước Phase 3 và Phase 7 ([00_MASTER_PLAN.md](../00_MASTER_PLAN.md) P3, P7).

---

## 1. Vì sao cần Time Synchronization

Camera (10Hz) và Radar (20Hz) **không bao giờ đo cùng 1 thời điểm tuyệt đối** — chúng có chu kỳ khác nhau và có thể lệch pha. Nếu dùng trực tiếp detection "gần nhất" mà không nội suy, sai số vị trí có thể lên tới vài chục cm với object đang di chuyển nhanh (vd 20m/s × 25ms lệch = 0.5m sai số).

### 1.1 Giải pháp: Interpolation Buffer
Giữ 2 detection gần nhất (trước và sau) mỗi sensor, nội suy tuyến tính về đúng "tick" thời gian chung:
$$z_{sync}(t) = z_1 + \frac{t - t_1}{t_2 - t_1}(z_2 - z_1), \quad t_1 \le t \le t_2$$

**Common tick strategy:** TrackGuard chọn tick cố định (vd mỗi 50ms = 20Hz — nhịp của pipeline decision, xem [03f](03f_REALTIME_CPP_LINUX.md)), mọi sensor được nội suy về đúng các tick này trước khi vào Stage 4.

---

## 2. Ego-motion Compensation — vì sao cần

Xe ego (bản thân) cũng di chuyển giữa các frame. Nếu không bù chuyển động ego, vị trí "tương đối" của object đo được ở `t-1` sẽ SAI khi so với frame ego tại `t` — vì frame tham chiếu đã dịch chuyển.

### 2.1 Dead-reckoning từ odometry/IMU
Với vận tốc ego `v_ego` và tốc độ quay `ψ̇_ego` biết được (từ oxts trong KITTI, hoặc IMU thật), tính độ dịch chuyển ego giữa `t-1` và `t`:
$$\Delta x = v_{ego} \cos(\bar\psi) \Delta t \qquad \Delta y = v_{ego} \sin(\bar\psi) \Delta t \qquad \Delta \psi = \dot\psi_{ego} \Delta t$$

Sau đó áp dụng phép biến đổi ngược (xoay + tịnh tiến) lên detection cũ để đưa nó về frame ego hiện tại:
$$\begin{bmatrix}x' \\ y'\end{bmatrix} = R(-\Delta\psi)\begin{bmatrix}x - \Delta x \\ y - \Delta y\end{bmatrix}$$

> **Test tỉnh táo (sanity check):** nếu ego đứng yên (`v_ego=0, ψ̇_ego=0`), compensation phải là identity (không đổi gì). Nếu ego rẽ 90°, detection phải xoay đúng 90° theo hướng ngược lại trong frame ego.

---

## 3. Coordinate Transform & Extrinsic Calibration

Mỗi sensor gắn ở vị trí khác nhau trên xe (camera trên kính chắn gió, radar ở cản trước) → có **frame riêng**. Extrinsic calibration là ma trận biến đổi (rotation + translation) từ sensor frame sang **vehicle frame chung** (thường gốc toạ độ đặt tại tâm trục sau hoặc tâm xe).

$$\begin{bmatrix}x_{vehicle} \\ y_{vehicle} \\ 1\end{bmatrix} = T_{sensor \to vehicle} \begin{bmatrix}x_{sensor} \\ y_{sensor} \\ 1\end{bmatrix}, \quad T = \begin{bmatrix}R & t \\ 0 & 1\end{bmatrix}$$

**Sai calibration → sai fusion:** nếu camera calibrate lệch 0.5° hoặc offset vị trí sai vài cm, object ở xa (40-50m) sẽ bị lệch cả mét trong vehicle frame — đây là lý do calibration luôn là bước đầu tiên trong bất kỳ hệ thống fusion thật nào, không phải chi tiết phụ.

---

## 4. Early/Low-level Fusion vs Late/Track-level Fusion

| | **Early Fusion** | **Late Fusion (TrackGuard chọn)** |
|---|---|---|
| Hợp nhất ở đâu | Ngay tại raw data/detection, trước khi track | Sau khi mỗi sensor đã ra track riêng |
| Cần gì | Đồng bộ pixel-level/point-level rất chính xác | Chỉ cần track state (position, velocity, covariance) |
| Thêm sensor mới | Sửa lại toàn bộ core fusion algorithm | Cắm thêm 1 track pipeline + gating rule trong `TrackFuser` |
| Chịu lỗi 1 sensor hỏng | Cả hệ thống ảnh hưởng ngay | Track sensor còn lại vẫn hoạt động độc lập (graceful degrade) |
| Độ chính xác lý thuyết | Cao hơn (giữ nhiều thông tin hơn) | Thấp hơn 1 chút, chấp nhận trade-off |
| Độ phức tạp cho portfolio 60 ngày | Không khả thi | **Khả thi, đúng trọng tâm** |

---

## 5. Covariance Intersection — công thức fusion track-to-track

Khi 2 track (từ 2 sensor khác nhau) được xác định là cùng 1 object thật (qua gating tương tự Stage 5), cần hợp nhất state của chúng. Vấn đề: **không biết mức độ tương quan (correlation)** giữa 2 ước lượng (chúng có thể chia sẻ nhiễu ego-motion chung). Covariance Intersection (CI) là công thức fusion **an toàn** không cần biết correlation:

$$P_{fused}^{-1} = \omega P_1^{-1} + (1-\omega) P_2^{-1}$$
$$x_{fused} = P_{fused}(\omega P_1^{-1} x_1 + (1-\omega) P_2^{-1} x_2)$$

`ω ∈ [0,1]` chọn để **tối thiểu hoá trace(P_fused)** hoặc `det(P_fused)` (thường dò tuyến tính `ω` từ 0 đến 1, chọn giá trị tốt nhất — đơn giản và đủ tốt cho portfolio).

**Tính chất quan trọng:** CI **luôn** cho `P_fused` "an toàn" (không đánh giá thấp uncertainty), khác với công thức fusion Kalman-optimal (giả định độc lập tuyệt đối) có thể quá lạc quan nếu 2 nguồn thực ra có tương quan ẩn.

---

## 6. Trade-off khi track chỉ có 1 sensor

Không phải lúc nào camera-track và radar-track cũng match được (vd radar không phát hiện người đi bộ tốt, camera bị che khuất). `TrackFuser` phải xử lý **graceful fallback**: track chỉ từ 1 sensor vẫn output bình thường (không bắt buộc phải có đủ 2 sensor mới có track) — chỉ đánh dấu `fusion_confidence` thấp hơn để downstream (TTC/decision) biết mà thận trọng hơn nếu cần.

## 7. Checklist tự-kiểm tra trước khi qua Phase 4 và Phase 8
- [ ] Tôi giải thích được vì sao lệch tần số sensor cần interpolation, không chỉ "lấy detection gần nhất"
- [ ] Tôi giải thích được ego-motion compensation bằng ví dụ cụ thể (ego rẽ 90°)
- [ ] Tôi nêu được 3 điểm khác biệt chính giữa early và late fusion, và lý do chọn late fusion
- [ ] Tôi hiểu Covariance Intersection khác gì so với fusion Kalman-optimal thông thường
