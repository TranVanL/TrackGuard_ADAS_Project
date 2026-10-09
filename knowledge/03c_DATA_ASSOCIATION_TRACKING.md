# 🔗 03c — Data Association & Multi-Object Tracking (Hungarian, Gating, Track Lifecycle, MOTA/MOTP)

> Đọc trước Phase 6 ([00_MASTER_PLAN.md](../00_MASTER_PLAN.md) P6). Đây là "core story" quan trọng nhất của TrackGuard — chứng minh hiểu multi-object tracking thật, không chỉ single-object demo.

---

## 1. Bài toán Data Association là gì

Sau khi có N track đang theo dõi và M detection mới ở frame hiện tại, câu hỏi là: **detection nào khớp với track nào?** Đây là bài toán **bipartite assignment** — mỗi track ghép tối đa 1 detection, mỗi detection ghép tối đa 1 track, tối thiểu hoá tổng chi phí (cost).

```
Track 1 ──cost(1,A)── Detection A
   ╲──cost(1,B)──────╱
Track 2 ──cost(2,A)── Detection B
   ╲──cost(2,B)──────╱
```

---

## 2. Cost function

Hai lựa chọn phổ biến:

**(a) Mahalanobis distance** (dùng khi có covariance từ Kalman Filter):
$$d^2 = (z - H\hat x)^T S^{-1} (z - H\hat x)$$
trong đó `S = HPH^T + R` (innovation covariance). Khoảng cách này **tính đến độ bất định** — track có covariance lớn (mới, ít chắc chắn) sẽ "dễ chấp nhận" detection ở xa hơn track có covariance nhỏ (đã ổn định).

**(b) IoU (Intersection over Union)** — dùng khi có bounding box (camera):
$$IoU = \frac{Area(box_{track} \cap box_{det})}{Area(box_{track} \cup box_{det})} \quad cost = 1 - IoU$$

> TrackGuard dùng **Mahalanobis** làm cost chính (phù hợp với Kalman Filter state đã có), IoU dùng bổ sung cho camera detection có bbox rõ (tăng độ chắc chắn association khi cả 2 metric đồng thuận).

---

## 3. Gating — TẠI SAO cần trước khi assignment

Nếu để nguyên toàn bộ N×M cặp vào Hungarian, thuật toán sẽ **luôn** tìm ra 1 assignment (kể cả khi tất cả cặp đều vô lý — vì Hungarian tối ưu hoá tổng chi phí thấp nhất, không có khái niệm "từ chối"). Gating là bước lọc **trước**, loại cặp có cost vượt ngưỡng, đặt cost = ∞ (hoặc loại khỏi ma trận) để Hungarian không bao giờ chọn cặp đó.

### 3.1 Chi-square gating table (Mahalanobis, dùng phổ biến trong tracking)

Với `d²` tuân theo phân phối chi-square bậc tự do = số chiều đo (`dim(z)`):

| Bậc tự do (dim z) | Ngưỡng 95% | Ngưỡng 99% |
|---|---|---|
| 1 | 3.84 | 6.63 |
| 2 (vị trí x,y) | 5.99 | 9.21 |
| 3 | 7.81 | 11.34 |
| 4 | 9.49 | 13.28 |

TrackGuard đo `[px, py]` (dim=2) → dùng ngưỡng **5.99** (95%) làm mặc định — cặp có `d² > 5.99` bị gate ra.

---

## 4. Hungarian Algorithm (Kuhn-Munkres)

**Input:** ma trận chi phí `C[N][M]` (N track × M detection, pad ma trận vuông nếu N≠M bằng cost giả rất lớn cho phần dummy)
**Output:** assignment tối ưu (tổng chi phí nhỏ nhất), độ phức tạp **O(n³)**

Ý tưởng thuật toán (mức hiểu để implement, không cần chứng minh toán học sâu):
1. Trừ min mỗi hàng, rồi trừ min mỗi cột → tạo số 0 "ứng viên"
2. Tìm bộ ghép cực đại chỉ dùng ô = 0 (dùng đường tăng — augmenting path)
3. Nếu chưa ghép hết → vẽ đường phủ tối thiểu qua các số 0, điều chỉnh ma trận (cộng/trừ theo min phần chưa phủ), lặp lại bước 2
4. Dừng khi số cặp ghép = min(N,M)

> Có thể tự viết O(n³) (Kuhn-Munkres classic) hoặc dùng thư viện nhỏ open-source (header-only) — miễn là **hiểu và giải thích được thuật toán**, không chỉ gọi hàm.

**Unit test bắt buộc:** dùng ví dụ sách giáo khoa có lời giải biết trước (vd ma trận 3×3 cost cụ thể → assignment tối ưu đã biết) để verify implementation đúng trước khi tích hợp vào pipeline.

---

## 5. Track Lifecycle State Machine

```
        ┌─────────────┐  matched N_confirm frame liên tiếp   ┌─────────────┐
  new ─▶│  Tentative  │──────────────────────────────────────▶│  Confirmed  │
        └─────────────┘                                       └─────────────┘
               │ unmatched                                       │  unmatched
               ▼                                                 ▼
        ┌─────────────┐                                    ┌─────────────┐
        │   Deleted   │◀───── unmatched > K_max frame ─────│  Coasting   │
        └─────────────┘                                    └─────────────┘
                                                                   │ matched lại
                                                                   ▼
                                                            (quay lại Confirmed)
```

| State | Ý nghĩa | Hành vi |
|---|---|---|
| **Tentative** | Track mới tạo (từ detection chưa match track nào) | Chỉ cần 1 lần unmatched → xoá ngay (tránh track "ma" từ nhiễu 1 frame) |
| **Confirmed** | Đã matched đủ `N_confirm` (vd 3) frame liên tiếp | Track "thật", dùng cho fusion/threat assessment |
| **Coasting** | Confirmed nhưng frame này không matched (occlusion tạm) | Chỉ chạy `predict()`, không `update()`. Giữ ID |
| **Deleted** | Unmatched liên tục > `K_max` frame (vd 5) khi đang Coasting | Xoá khỏi track list |

**Vì sao cần Coasting thay vì xoá ngay khi unmatched:** occlusion (bị xe khác che khuất tạm thời) là hiện tượng bình thường, không phải track sai. Xoá ngay sẽ làm track "chết rồi sống lại" với ID mới → phá vỡ tính liên tục cần thiết cho fusion và threat assessment (TTC cần lịch sử track liên tục).

---

## 6. MOTA / MOTP — chỉ số đánh giá chuẩn ngành

**MOTP (Multi-Object Tracking Precision)** — độ chính xác vị trí trung bình của các track đã matched đúng:
$$MOTP = \frac{\sum_{t,i} d_{t,i}}{\sum_t c_t}$$
(`d_{t,i}` = sai số vị trí matched tại frame t object i, `c_t` = số match tại frame t)

**MOTA (Multi-Object Tracking Accuracy)** — độ chính xác tổng thể, phạt lỗi (miss, false positive, ID switch):
$$MOTA = 1 - \frac{\sum_t (m_t + fp_t + idsw_t)}{\sum_t g_t}$$
- `m_t` = số object bị miss (có ground-truth nhưng không track được)
- `fp_t` = số false positive (track ra nhưng không có ground-truth tương ứng)
- `idsw_t` = số lần ID switch (track nhảy ID giữa 2 object)
- `g_t` = tổng số ground-truth object tại frame t

> **ID switch là chỉ số quan trọng nhất để chứng minh chất lượng tracking** — nếu Hungarian + gating + track lifecycle đúng, ID switch phải rất thấp qua occlusion ngắn.

## 7. Checklist tự-kiểm tra trước khi qua Phase 7
- [ ] Tôi giải thích được vì sao cần gate trước khi Hungarian
- [ ] Tôi hiểu ý nghĩa chi-square threshold, không chỉ copy số 5.99
- [ ] Tôi giải thích được vì sao cần state Coasting (không xoá ngay khi unmatched)
- [ ] Tôi biết công thức MOTA/MOTP và ý nghĩa ID switch
