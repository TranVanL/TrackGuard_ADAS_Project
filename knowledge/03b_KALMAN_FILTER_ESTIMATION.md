# 🧮 03b — Kalman Filter & State Estimation (CV / CTRV / EKF)

> Đọc trước Phase 4-5 ([00_MASTER_PLAN.md](../00_MASTER_PLAN.md) P4-P5). Đây là file quan trọng nhất — sai công thức ở đây lan lỗi ra toàn bộ pipeline. Viết tay từng công thức trước khi code.

---

## 1. Vì sao cần Kalman Filter (trực giác)

Ta có 2 nguồn thông tin về vị trí object, cả 2 đều không hoàn hảo:
1. **Dự đoán** (từ mô hình chuyển động: "object đang chạy đều thì frame sau chắc ở đây")
2. **Đo lường** (từ sensor: "radar đo được nó ở đây")

Kalman Filter là cách **kết hợp tối ưu** 2 nguồn này theo tỉ lệ độ tin cậy (covariance) của từng nguồn — tin đo lường nhiều hơn nếu sensor rất chính xác, tin dự đoán nhiều hơn nếu sensor nhiễu.

---

## 2. Kalman Filter tuyến tính (Linear KF) — dùng cho CV model

### 2.1 State vector (CV — Constant Velocity)
$$x = \begin{bmatrix} p_x \\ p_y \\ v_x \\ v_y \end{bmatrix}$$

### 2.2 Predict step
$$x' = Fx \qquad P' = FPF^T + Q$$

Với CV model, ma trận chuyển trạng thái:
$$F = \begin{bmatrix} 1 & 0 & \Delta t & 0 \\ 0 & 1 & 0 & \Delta t \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

`Q` (process noise covariance) mô hình hoá việc "vận tốc không thực sự hằng số" — thường dùng discretized white-noise acceleration model:
$$Q = \sigma_a^2 \begin{bmatrix} \Delta t^4/4 & 0 & \Delta t^3/2 & 0 \\ 0 & \Delta t^4/4 & 0 & \Delta t^3/2 \\ \Delta t^3/2 & 0 & \Delta t^2 & 0 \\ 0 & \Delta t^3/2 & 0 & \Delta t^2 \end{bmatrix}$$

`σ_a` (process noise acceleration std) là **tham số cần tune** — quá nhỏ → filter "cứng đầu" không theo kịp thay đổi thật; quá lớn → filter nhiễu theo đo lường quá nhiều. Giá trị khởi điểm hợp lý cho xe: `σ_a ≈ 2-4 m/s²`.

### 2.3 Update step
$$y = z - Hx \qquad S = HPH^T + R \qquad K = PH^TS^{-1}$$
$$x = x + Ky \qquad P = (I-KH)P$$

`H` (measurement matrix) ánh xạ state → không gian đo. Nếu sensor chỉ đo vị trí (không đo vận tốc trực tiếp), với state `[px,py,vx,vy]`:
$$H = \begin{bmatrix} 1 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 \end{bmatrix}$$

`R` (measurement noise covariance) lấy trực tiếp từ noise model sensor (xem [03a §4](03a_SENSORS_PERCEPTION.md)).

### 2.4 Joseph form (BẮT BUỘC dùng, không dùng công thức rút gọn `(I-KH)P`)

Công thức rút gọn `P = (I-KH)P` **nhạy cảm với sai số số học** (rounding error có thể làm P mất tính đối xứng hoặc âm). Joseph form ổn định hơn:
$$P = (I-KH)P(I-KH)^T + KRK^T$$

**Bắt buộc** assert trong debug build:
```cpp
assert((P - P.transpose()).norm() < 1e-6);   // symmetric
assert(P.eigenvalues().real().minCoeff() >= -1e-9); // PSD (cho phép sai số số học nhỏ)
```

---

## 3. Kalman Gain — ý nghĩa trực giác

$$K = PH^T S^{-1}$$

- `K → 0`: tin dự đoán hơn (measurement rất nhiễu, `R` lớn)
- `K → 1` (giới hạn tối đa): tin đo lường hơn (sensor rất chính xác, `R` nhỏ; hoặc dự đoán đang rất bất định, `P` lớn)
- Sau mỗi update, `P` luôn **giảm** (uncertainty giảm) — nếu `P` không giảm dần qua các bước, có bug trong công thức

---

## 4. Extended Kalman Filter (EKF) — dùng cho CTRV model

### 4.1 Vì sao cần EKF
Khi object rẽ cua (yaw_rate ≠ 0), chuyển động **không còn tuyến tính** — không thể biểu diễn bằng ma trận `F` hằng số. EKF giải quyết bằng cách **tuyến tính hoá cục bộ** quanh điểm ước lượng hiện tại bằng Jacobian.

### 4.2 State vector (CTRV — Constant Turn-Rate and Velocity)
$$x = \begin{bmatrix} p_x \\ p_y \\ v \\ \psi \\ \dot\psi \end{bmatrix} \quad (\psi = \text{yaw}, \ \dot\psi = \text{yaw rate})$$

### 4.3 Non-linear transition function (khi `ψ̇ ≠ 0`)
$$p_x' = p_x + \frac{v}{\dot\psi}\left[\sin(\psi + \dot\psi \Delta t) - \sin\psi\right]$$
$$p_y' = p_y + \frac{v}{\dot\psi}\left[-\cos(\psi + \dot\psi \Delta t) + \cos\psi\right]$$
$$v' = v \qquad \psi' = \psi + \dot\psi \Delta t \qquad \dot\psi' = \dot\psi$$

**Trường hợp đặc biệt `ψ̇ ≈ 0`** (đi thẳng, không cua) — công thức trên chia cho 0, phải xử lý riêng bằng xấp xỉ CV:
$$p_x' = p_x + v\cos(\psi)\Delta t \qquad p_y' = p_y + v\sin(\psi)\Delta t$$
Ngưỡng thực dụng: `|ψ̇| < 1e-3 rad/s` → dùng nhánh xấp xỉ này.

### 4.4 Jacobian của hàm chuyển động (`Fj`, 5×5)
Tính đạo hàm riêng từng phần tử state mới theo state cũ — đây là bước hay sai nhất, **phải viết tay từng phần tử trước khi code**:
$$F_j = \frac{\partial f}{\partial x}\Big|_{x=\hat{x}}$$

Predict với EKF dùng `Fj` thay cho `F`:
$$P' = F_j P F_j^T + Q$$
nhưng state `x' = f(x)` dùng **hàm phi tuyến thật** (không phải `Fj x`).

### 4.5 Update step EKF — vẫn tuyến tính nếu đo trực tiếp vị trí
Nếu sensor vẫn đo `[px, py]` trực tiếp, `H` giữ tuyến tính như CV model → update step giống hệt §2.3 (Joseph form áp dụng y hệt).

---

## 5. CV vs CTRV — khi nào dùng model nào

| Tiêu chí | CV | CTRV |
|---|---|---|
| Object đi thẳng đều | Rất tốt, đơn giản, tính toán nhanh | Tốt nhưng dư thừa tham số |
| Object rẽ cua | RMSE cao (model sai) | Chính xác hơn nhiều |
| Độ phức tạp tính toán | Thấp (tuyến tính thuần) | Cao hơn (Jacobian mỗi bước) |
| Ổn định số học | Rất ổn định | Cần xử lý case `ψ̇≈0` cẩn thận |

> **Nâng cao (không bắt buộc code trong 60 ngày, nhưng nên biết để trả lời phỏng vấn):** Interacting Multiple Model (IMM) filter chạy song song nhiều model (CV + CTRV) và trộn theo xác suất — đây là hướng production thật hay dùng. TrackGuard chọn CTRV đơn model có chủ đích để giữ scope trong 60 ngày, và có thể nói rõ "biết IMM là bước tiếp theo hợp lý" khi được hỏi.

---

## 6. Tuning Q và R — quy trình thực dụng

1. Bắt đầu với `R` lấy đúng từ sensor noise model đã biết (không phải tune, là số đo thật)
2. Tune `σ_a` (process noise) bằng cách chạy trên scenario có ground-truth, thử vài giá trị (0.5, 2, 5, 10 m/s²), đo RMSE, chọn giá trị RMSE thấp nhất
3. Ghi lại bảng "RMSE theo σ_a" vào docs — đây là bằng chứng hiểu tuning, không phải hard-code số ngẫu nhiên

## 7. Checklist tự-kiểm tra trước khi qua Phase 6
- [ ] Tôi có thể viết tay công thức predict/update KF không nhìn tài liệu
- [ ] Tôi hiểu tại sao Joseph form ổn định hơn công thức rút gọn
- [ ] Tôi có thể giải thích Jacobian trong EKF dùng để làm gì
- [ ] Tôi biết case đặc biệt `ψ̇≈0` cần xử lý riêng, và tại sao
