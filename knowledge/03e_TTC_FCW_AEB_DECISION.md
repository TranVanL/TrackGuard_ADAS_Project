# 🚨 03e — TTC, FCW & AEB Decision Logic

> Đọc trước Phase 8-10 ([00_MASTER_PLAN.md](../00_MASTER_PLAN.md) P8-P10). **Safety-honest note:** mọi ngưỡng số (2.6s, 1.6s, 0.9s...) trong file này là giá trị **tham khảo tự chọn** dựa trên hiểu biết chung về NCAP/AEB testing concept, KHÔNG phải số chính thức của bất kỳ hãng xe/tiêu chuẩn cụ thể nào. Khi phỏng vấn, nói rõ: *"Ngưỡng tôi chọn dựa trên nghiên cứu chung về FCW/AEB timing, có thể tune lại theo yêu cầu cụ thể của từng hệ thống thật."*

---

## 1. Time-To-Collision (TTC) — công thức và giới hạn

### 1.1 Công thức cơ bản
$$TTC = \frac{d_{range}}{v_{closing}}, \quad v_{closing} > 0 \text{ (đang tiến lại gần)}$$

Nếu `v_closing ≤ 0` (object đang xa dần hoặc đứng yên tương đối) → `TTC = ∞` (không có nguy cơ va chạm theo động học tuyến tính).

**Ví dụ tính tay:** lead vehicle cách 40m, closing speed 10 m/s → `TTC = 40/10 = 4.0s`.

### 1.2 Giới hạn của công thức TTC tuyến tính
Công thức trên giả định **closing speed không đổi** — thực tế nếu lead vehicle đang phanh (gia tốc âm), TTC thật sẽ **nhỏ hơn** giá trị tính theo công thức tuyến tính (va chạm đến sớm hơn dự đoán). Phiên bản nâng cao hơn (không bắt buộc code, nên biết để trả lời phỏng vấn) dùng closing acceleration:
$$TTC_{quadratic}: \quad \frac{1}{2}a t^2 + v_{closing}\,t - d_{range} = 0 \Rightarrow t = \frac{-v_{closing} + \sqrt{v_{closing}^2 + 2 a\, d_{range}}}{a}$$

TrackGuard dùng **TTC tuyến tính đơn giản** có chủ đích (đủ để chứng minh hiểu nguyên lý + dễ validate bằng tay), và ghi rõ hướng nâng cao trong docs.

### 1.3 Path relevance — vì sao cần `PathPredictor`
TTC chỉ có ý nghĩa nếu object **đang ở trong đường đi dự kiến** của ego. Object ở làn bên cạnh có closing speed dương (do góc nhìn) nhưng KHÔNG va chạm — đây là nguồn false-positive lớn nếu bỏ qua bước lọc `in_ego_path`.

---

## 2. FCW (Forward Collision Warning) — thiết kế state machine chống chattering

### 2.1 Vấn đề chattering
Nếu chỉ so sánh `TTC < threshold` mỗi frame độc lập, khi TTC dao động sát quanh ngưỡng (nhiễu đo lường), hệ thống sẽ bật/tắt cảnh báo liên tục → gây khó chịu và mất tin cậy với tài xế ("cry wolf effect" — tài xế sẽ bỏ qua cảnh báo nếu nó kêu quá thường xuyên).

### 2.2 Giải pháp: N-frame confirm + Hysteresis margin
```
State Idle:
  nếu TTC < T_warn trong N_confirm frame LIÊN TIẾP → chuyển Warn
State Warn:
  nếu TTC > (T_warn + margin) → chuyển về Idle
  (margin > 0 tạo "vùng đệm" chống dao động ngay tại ranh giới)
```

Giá trị tham khảo thường dùng trong thiết kế FCW (KHÔNG phải số chuẩn chính thức):
- `T_warn ≈ 2.6s`
- `margin ≈ 0.4s` (nghĩa là phải TTC > 3.0s mới về Idle)
- `N_confirm ≈ 3 frame` (@ 20Hz ≈ 150ms) — đủ nhanh để cảnh báo kịp thời, đủ chậm để lọc nhiễu 1 frame

---

## 3. AEB (Autonomous Emergency Braking) — Staged Braking Logic

### 3.1 Vì sao cần nhiều stage thay vì 1 ngưỡng phanh gấp
Phanh gấp đột ngột (full brake) khi chưa thực sự cần thiết gây nguy hiểm (xe sau tông vào) và giảm tin cậy hệ thống. Thiết kế thực tế dùng **staged response** — cảnh báo mạnh dần rồi mới phanh toàn lực:

```
Idle ──TTC < 1.6s──▶ Stage1 (partial brake, vd -3 m/s², kèm cảnh báo âm thanh mạnh)
Stage1 ──TTC < 0.9s──▶ Stage2 (full brake, vd -8 đến -10 m/s²)
Stage1/Stage2 ──TTC hồi phục > threshold + margin──▶ Idle
```

(Giá trị 1.6s/0.9s tham khảo tự chọn, tương tự tinh thần AEB test protocol công khai — không phải số chính thức 1 hãng cụ thể.)

### 3.2 Suppression Rules — vì sao cần
AEB không nên can thiệp nếu tài xế **đã đang phản ứng đúng**:
- **Driver braking override**: nếu `driver_brake_input > threshold` (tài xế đã đạp phanh mạnh) → suppress Stage2 (không cần phanh tự động full lực chồng lên phanh tài xế, tránh phanh quá mức gây mất kiểm soát)
- **Driver steering override**: nếu tài xế đánh lái tránh rõ ràng (steering angle rate vượt ngưỡng) → suppress AEB (tài xế đã chủ động tránh, object có thể không còn trong path)

### 3.3 Liên hệ khái niệm Functional Safety (ISO 26262) — mức hiểu, không phải certified
AEB là hệ thống có thể gây hại nếu **kích hoạt sai** (phanh gấp không cần thiết → tai nạn từ xe sau) hoặc **không kích hoạt khi cần** (miss detection → va chạm). Đây chính là lý do ISO 26262 (functional safety) tồn tại — thiết kế phải cân bằng 2 loại lỗi này (false positive vs false negative), có cơ chế giám sát/suppression, và phải được validate trên **bộ scenario rộng** trước khi coi là an toàn.

> **Honesty framing bắt buộc nói khi phỏng vấn:** *"Tôi hiểu khái niệm và thiết kế theo tinh thần ISO 26262/NCAP ở mức kiến trúc — state machine, suppression, staged response, validation bằng scenario library. Tôi KHÔNG claim hệ thống này đã qua quy trình certification thật, vì điều đó cần cả 1 team functional safety chuyên trách và test bench phần cứng thật."*

## 4. Checklist tự-kiểm tra trước khi qua Phase 11
- [ ] Tôi giải thích được công thức TTC và giới hạn của nó (giả định closing speed const)
- [ ] Tôi giải thích được vì sao cần N-frame confirm + margin cho FCW (chống chattering)
- [ ] Tôi giải thích được vì sao AEB cần staged thay vì 1 ngưỡng phanh gấp
- [ ] Tôi nói được rõ ràng ranh giới "hiểu khái niệm" vs "certified production system"
