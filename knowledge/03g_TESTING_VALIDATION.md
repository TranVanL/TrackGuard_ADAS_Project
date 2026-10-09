# ✅ 03g — Testing & Validation Strategy (Unit, Scenario, Metrics, CI)

> Đọc trước Phase 12 ([00_MASTER_PLAN.md](../00_MASTER_PLAN.md) P12), nhưng áp dụng nguyên tắc **ngay từ Phase 1** — mỗi phase trước đó đều đã yêu cầu unit test đi kèm.

---

## 1. Testing Pyramid cho TrackGuard

```
        ▲
       ╱ ╲        Scenario Integration Tests (~15-20)
      ╱   ╲       — cut-in, hard-brake, pedestrian, FP-traps
     ╱─────╲      — chạy full pipeline, check output cuối (FCW/AEB trigger đúng/sai)
    ╱       ╲
   ╱ Unit    ╲    Unit Tests (~90-100)
  ╱  Tests    ╲   — mỗi công thức toán (KF predict/update, Hungarian, TTC, gating)
 ╱─────────────╲  — test độc lập từng class, không phụ thuộc pipeline đầy đủ
```

**Nguyên tắc:** unit test nhiều hơn scenario test rất nhiều (đáy tháp rộng) — vì mỗi công thức sai sẽ dễ debug hơn NHIỀU nếu bắt được ở unit test (biết chính xác class nào sai), so với chỉ biết "scenario integration fail" (phải debug ngược qua cả pipeline).

---

## 2. Nguyên tắc viết Unit Test (áp dụng toàn bộ 03b, 03c)

### 2.1 Test phải verify TOÁN HỌC, không phải "chạy không crash"
❌ Sai: `EXPECT_NO_THROW(kf.predict(0.1));`
✅ Đúng: `EXPECT_NEAR(kf.state().x(), expected_x, 1e-6);` — so khớp với **closed-form solution** đã tính tay trước.

### 2.2 Test case biết trước lời giải (textbook case)
Với Hungarian algorithm, dùng ma trận cost cụ thể có lời giải tối ưu đã biết (tính tay hoặc tra cứu) — KHÔNG dùng ma trận ngẫu nhiên rồi tự tin "chắc đúng".

### 2.3 Test edge case, không chỉ happy path
| Class | Edge case bắt buộc test |
|---|---|
| `KalmanFilterCV`/`EKF` | Covariance ban đầu rất lớn/nhỏ, đo liên tiếp giống hệt nhau, `dt=0` |
| `HungarianAssigner` | N≠M (không vuông), ma trận toàn cost=∞ (không match được cặp nào) |
| `MahalanobisGate` | Detection đúng tại biên ngưỡng chi-square (test cả 2 phía > và <) |
| `TrackManager` | Unmatched đúng tại `K_max` frame (biên xoá/giữ track) |
| `TTCEstimator` | `closing_speed = 0` (phải trả `TTC = ∞`, không chia 0/NaN) |
| `FcwDecision` | TTC dao động đúng tại ranh giới threshold nhiều lần liên tiếp (anti-chattering) |
| `AebDecision` | `driver_brake_input` đúng tại ngưỡng suppression |

---

## 3. Scenario-based Integration Testing

### 3.1 Cấu trúc 1 scenario (YAML) kèm expected outcome
```yaml
name: hard_brake_lead_vehicle
duration_s: 8
ego: {v0: 22.0}
objects:
  - id: lead1
    type: vehicle
    start_range_m: 35
    v0: 20.0
    brake_at_s: 2.0
    brake_decel: -6.0
expected:
  fcw_triggers: true
  fcw_trigger_time_s_range: [2.5, 4.0]   # khoảng thời gian hợp lý để FCW trigger
  aeb_stage_reached: "Stage2"
```

### 3.2 Danh sách scenario tối thiểu (20+ case cho P12, chia tuning set / holdout set)

> **Phương pháp chống overfitting:** chia scenario thành **tuning set** (dùng để chọn threshold FCW/AEB ở P9-P10) và **holdout set** (chỉ chạy để báo cáo cuối cùng ở P12, KHÔNG dùng để tune) — giống train/test split, chứng minh threshold generalize chứ không chỉ đúng với đúng scenario đã viết.

**Tuning set (14 case):**

| # | Scenario | Mục đích |
|---|---|---|
| 1 | `straight_lead_vehicle` | Baseline — không nguy hiểm, không trigger gì |
| 2 | `hard_brake_lead_vehicle` | FCW + AEB Stage1+Stage2 phải trigger đúng thứ tự |
| 3 | `cut_in_adjacent_lane` | Object từ làn bên cắt vào — track phải bắt kịp nhanh |
| 4 | `pedestrian_crossing` | Object nhỏ, chuyển động ngang — test model CV phù hợp hơn CTRV |
| 5 | `false_positive_adjacent_lane` | Xe làn bên chạy song song — `PathPredictor` phải lọc, KHÔNG trigger FCW |
| 6 | `occlusion_short` | Object bị che khuất 3-4 frame rồi xuất hiện lại — ID phải giữ nguyên |
| 7 | `borderline_ttc_oscillation` | TTC dao động quanh threshold — FCW không được chattering |
| 8 | `driver_brake_override` | Driver đã phanh trước — AEB Stage2 phải bị suppress |
| 9 | `curved_road_lead_vehicle` | Ego + lead vehicle cùng rẽ cua — CTRV phải track đúng |
| 10 | `multi_object_dense` | 5+ object cùng lúc, một số cắt nhau — MOTA/ID switch phải thấp |
| 11 | `radar_only_dropout` | Camera mất tín hiệu tạm thời — fusion fallback radar-only vẫn output |
| 12 | `camera_only_dropout` | Radar mất tín hiệu tạm thời — fusion fallback camera-only vẫn output |
| 13 | `low_confidence_flicker` | Detection confidence dao động quanh threshold — không được tạo track giả |
| 14 | `stop_and_go_traffic` | Ego + lead vehicle tăng/giảm tốc liên tục — FCW không được spam cảnh báo |

**Holdout set (6+ case, chỉ dùng để báo cáo cuối ở P12, không dùng để tune threshold):**

| # | Scenario | Mục đích |
|---|---|---|
| 15 | `holdout_hard_brake_wet_road` | Biến thể phanh gấp với profile giảm tốc khác — kiểm tra threshold có generalize không |
| 16 | `holdout_cut_in_fast` | Cut-in tốc độ cao hơn tuning set — test biên |
| 17 | `holdout_pedestrian_occluded` | Kết hợp pedestrian + occlusion — case chưa từng thấy riêng lẻ |
| 18 | `holdout_dense_intersection` | Giao lộ nhiều object đổi hướng — stress test association |
| 19 | `holdout_long_range_approach` | Vật cản xuất hiện từ rất xa, TTC giảm dần đều — test độ ổn định TTC dài hạn |
| 20 | `holdout_false_positive_curve` | Xe làn bên trên đường cong — test `PathPredictor` ở case khó hơn tuning set |

### 3.3 So khớp expected vs actual
Mỗi scenario test đọc `expected` từ YAML, chạy full pipeline, so khớp:
- `fcw_triggers` (bool) khớp
- Nếu có trigger, thời điểm trigger nằm trong khoảng hợp lý (không yêu cầu chính xác tuyệt đối vì có random noise, chỉ cần trong range)
- `aeb_stage_reached` khớp giá trị cao nhất đạt được

---

## 4. Metrics cuối cùng (P12 Day 56-57)

| Metric | Công thức/nguồn | Mục đích |
|---|---|---|
| MOTA/MOTP | Xem [03c §6](03c_DATA_ASSOCIATION_TRACKING.md) | Đánh giá chất lượng tracking tổng thể |
| ID switch count | Đếm số lần track đổi ID sai | Chất lượng association/lifecycle |
| FCW Precision/Recall/F1 | So khớp `expected.fcw_triggers` toàn bộ scenario | Chất lượng decision layer |
| AEB Precision/Recall/F1 | Tương tự, theo stage | Chất lượng decision layer |
| RMSE vị trí (theo model CV vs CTRV) | So ground-truth | Chất lượng estimation |
| p50/p95/p99 cycle time | Xem [03f §4](03f_REALTIME_CPP_LINUX.md) | Chất lượng real-time engine |

> **Báo cáo trung thực:** không cần số liệu "đẹp tuyệt đối" — quan trọng là **phương pháp đo đúng chuẩn ngành** (MOTA/MOTP là chỉ số thật dùng trong research/production tracking, không phải tự chế). Nếu recall FCW thấp ở 1 scenario cụ thể, ghi rõ nguyên nhân (vd threshold chưa tune tối ưu) — đây là dấu hiệu **hiểu sâu**, không phải điểm yếu.

---

## 5. CI Pipeline (GitHub Actions)

```yaml
name: ci
on: [push, pull_request]
jobs:
  build-test:
    strategy:
      matrix:
        compiler: [gcc, clang]
        build_type: [Debug, Release]
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - name: Install deps
        run: sudo apt-get install -y libeigen3-dev cmake ${{ matrix.compiler }}
      - name: Configure (Debug = ASan+UBSan+TSan on)
        run: cmake -B build -DCMAKE_BUILD_TYPE=${{ matrix.build_type }} -DENABLE_SANITIZERS=${{ matrix.build_type == 'Debug' }}
      - name: Build
        run: cmake --build build -j
      - name: Test
        run: ctest --test-dir build --output-on-failure
      - name: Coverage (gcc Debug only)
        if: matrix.compiler == 'gcc' && matrix.build_type == 'Debug'
        run: |
          gcovr -r . --xml -o coverage.xml
```

## 6. Checklist tự-kiểm tra trước khi ship v1.0
- [ ] Mọi công thức toán trong 03b/03c/03e đều có unit test tương ứng
- [ ] 20+ scenario integration test pass (14 tuning + 6 holdout), có scenario "chống false-positive" rõ ràng
- [ ] MOTA/MOTP + FCW/AEB precision-recall có số liệu thật trên CẢ tuning và holdout set trong README
- [ ] CI matrix xanh, sanitizers (ASan/UBSan/TSan) clean trên build Debug
- [ ] Có ít nhất 1 scenario mà kết quả "không hoàn hảo" được note rõ nguyên nhân trong docs (bằng chứng honesty + hiểu sâu)
