# 📅 TrackGuard — Plan chi tiết 12 tuần (60 ngày code)

> **Quy tắc bất biến:**
> - Mỗi tối **2-3h thuần code** + 30' đọc/note
> - **1 commit có ý nghĩa** mỗi ngày
> - **Cuối mỗi tuần**: push + tag `wXX-done` + 5 dòng "what I learned" trong `docs/lessons.md`
> - **MUST** = bắt buộc, **NICE** = cut nếu trễ (xem [00_MASTER_PLAN.md §10](00_MASTER_PLAN.md))
> - Trước mỗi Phase, đọc file kiến thức tương ứng trong `knowledge/` (link ở đầu mỗi Phase)

## 🖥️ Môi trường thực tế

| Thành phần | Chạy ở đâu | Cần gì? |
|---|---|---|
| Toàn bộ pipeline C++17 | **Linux host** | GCC/Clang + CMake + Eigen3 (linear algebra) |
| Dataset | **KITTI Tracking subset** (2-3 sequence, ~vài trăm MB) | Tải thủ công 1 lần, để trong `data/kitti_subset/` (gitignored) |
| Synthetic scenario | **Tự sinh, không cần dataset** | Python script hoặc C++ generator → YAML |
| Visualization (NICE) | **Linux host** | SDL2 hoặc OpenCV (`libopencv-dev`) |
| CI | **GitHub Actions** | ubuntu-latest runner, matrix: gcc/clang × Debug(sanitizer)/Release |

Setup 1 lần trước Day 1:
```bash
sudo apt install libeigen3-dev libopencv-dev cmake g++ clang
mkdir -p data/kitti_subset && cd data/kitti_subset
# Tải 2-3 sequence nhỏ từ KITTI tracking benchmark (label_02 + oxts), giải nén vào đây
```

---

## 📅 PHASE 1 / TUẦN 1 — Setup + Dataset/Scenario Replay Harness (8%)

> 📖 Đọc trước: [knowledge/03a_SENSORS_PERCEPTION.md](knowledge/03a_SENSORS_PERCEPTION.md) §1-2, [knowledge/03d_SENSOR_FUSION_ARCHITECTURE.md](knowledge/03d_SENSOR_FUSION_ARCHITECTURE.md) §1
> **Goal:** Replay 1 sequence KITTI → console in detection đúng thứ tự, đúng timestamp, deterministic.

### Day 1 (T2) — Đọc pipeline ADAS + format KITTI, KHÔNG code (1.5h đọc + 30' note)
**Đọc:**
1. KITTI tracking benchmark README (label format: frame, track_id, type, bbox, location, rotation_y)
2. Khái quát pipeline ADAS L2: Sensor → Perception → Fusion/Tracking → Threat → Decision
3. `knowledge/03a_SENSORS_PERCEPTION.md` §1-2 (camera/radar detection model)

**Note (1 trang):**
- KITTI label_02 format: mỗi dòng = 1 detection tại 1 frame, cột nào là gì
- oxts format: GPS/IMU → dùng để tính ego-motion (velocity, yaw rate)
- 5 khối pipeline chính + 2 khối hạ tầng (time sync, real-time engine)

**Commit:** *(không cần — chỉ note giấy)*

### Day 2 (T3) — Project skeleton (2h)
```
TrackGuard/
├── CMakeLists.txt        (C++17, find_package(Eigen3), option(BUILD_TESTS))
├── config/                (default.yaml — seed, thresholds, RunConfig defaults)
├── include/trackguard/   (empty dirs: config/ sensors/ sync/ transform/ estimation/
│                          association/ tracking/ fusion/ threat/ decision/
│                          engine/ replay/)
├── src/
├── tests/
├── data/kitti_subset/    (.gitignore)
├── docs/lessons.md
└── .github/workflows/ci.yml   (chỉ build, chưa test)
```
`git init && git add -A && git commit -m "[W1D2] chore: project skeleton"`

**Output/DoD:** `cmake .. && make` chạy được (dù chưa có code thật), CI shell green.

### Day 3 (T4) — Tải KITTI subset + nghiên cứu format thật + `RunConfig`/seeded RNG (2h)
- Tải 2-3 sequence nhỏ (vd `0000`, `0001`) label_02 + oxts vào `data/kitti_subset/`
- Viết script Python nhỏ (`scripts/inspect_kitti.py`) in ra 20 dòng đầu label file để hiểu cột thật
- Viết `config/default.yaml` (rng_seed, thresholds) + `RunConfig` loader + `std::mt19937_64` seeded từ config — toàn bộ randomness trong project (noise injector, scenario generator) đều phải đi qua RNG này, không được dùng `rand()`/`std::random_device` rải rác
- Note lại mapping cột → struct field cho `RawDetection`

**Commit:** `[W1D3] chore: KITTI subset + inspect script + RunConfig/seeded RNG`

### Day 4 (T5) — `KittiTrackingLoader` implementation (2h)
`include/trackguard/replay/kitti_loader.h` + `src/replay/kitti_loader.cpp`:
- Parse label_02 file → `std::vector<RawDetection>` group theo frame
- Parse oxts → ego pose/velocity per frame
- gtest: parse 5 dòng mẫu hard-coded → đúng field

**Commit:** `[W1D4] feat(replay): KittiTrackingLoader + parser unit test`

### Day 5 (T6) — `ReplayClock` + replay loop cơ bản (1.5h) + push
`include/trackguard/replay/replay_clock.h`:
- Virtual time, tick theo frame index của KITTI (10Hz)
- `main.cpp`: load sequence → loop theo `ReplayClock` → in detection ra console theo đúng thứ tự thời gian

Push GitHub → tag `w01-done`. Viết 5 dòng "what I learned" (`docs/lessons.md`).

**Commit:** `[W1D5] feat(replay): ReplayClock + basic replay loop demo`

### ✅ Checkpoint W1 (MUST)
- [ ] `KittiTrackingLoader` parse đúng 1 sequence, unit test pass
- [ ] Replay loop in detection đúng thứ tự frame, đúng timestamp
- [ ] Repo push GitHub + tag `w01-done`

---

## 📅 PHASE 2 / TUẦN 2 — Sensor Adapters + Determinism (8%)

> 📖 Đọc trước: [knowledge/03a_SENSORS_PERCEPTION.md](knowledge/03a_SENSORS_PERCEPTION.md) §3-5 (noise model, radar synth)

### Day 6 (T2) — `ISensorSource` interface + `RawDetection` struct final (1.5h)
`include/trackguard/sensors/isensor_source.h`:
```cpp
class ISensorSource {
public:
  virtual std::vector<RawDetection> poll(Timestamp t) = 0;
  virtual SensorType type() const = 0;
  virtual ~ISensorSource() = default;
};
```
**Commit:** `[W2D6] feat(sensors): ISensorSource interface + RawDetection struct`

### Day 7 (T3) — `CameraDetectionSource` (2h)
- Wrap `KittiTrackingLoader`, lọc field cần cho "camera" (bbox center → pos xấp xỉ, class, confidence giả lập 0.7-0.95)
- Unit test: poll(t) trả đúng số detection tại frame t

**Commit:** `[W2D7] feat(sensors): CameraDetectionSource`

### Day 8 (T4) — `RadarDetectionSource` synthetic (2h)
> KITTI không có radar raw → sinh **synthetic radar detection** từ ground-truth 3D location + velocity (từ vị trí sai phân giữa 2 frame) + noise Gaussian (range ±0.1m, azimuth ±0.5°, range-rate ±0.2m/s) — đây chính là kỹ thuật "sensor model" chuẩn dùng trong test ADAS.
- `include/trackguard/sensors/radar_detection_source.h` + noise injector (seeded RNG cho reproducibility)

**Commit:** `[W2D8] feat(sensors): RadarDetectionSource — synthetic from ground-truth + noise`

### Day 9 (T5) — Determinism test (1.5h)
- gtest: chạy replay 2 lần với cùng seed → output bit-identical (so sánh từng `RawDetection`)
- Đây là property quan trọng: pipeline phải reproducible để debug và để scenario test tin cậy được

**Commit:** `[W2D9] test(sensors): determinism — same seed = identical replay output`

### Day 10 (T6) — Synthetic Scenario DSL skeleton + push (2h)
`scenarios/straight_lead_vehicle.yaml`:
```yaml
name: straight_lead_vehicle
duration_s: 10
ego: {v0: 20.0, a: 0.0}
objects:
  - id: lead1
    type: vehicle
    start_range_m: 40
    v0: 15.0
    a: 0.0
```
`ScenarioDsl` parser → sinh `RawDetection` trực tiếp (bypass KITTI) cho edge case tự thiết kế.

Push → tag `w02-done`, note lessons.

**Commit:** `[W2D10] feat(replay): ScenarioDsl YAML parser + first synthetic scenario`

### ✅ Checkpoint W2 (MUST)
- [ ] 2 `ISensorSource` impl (Camera thật từ KITTI, Radar synthetic)
- [ ] Determinism test pass
- [ ] Scenario DSL sinh được 1 scenario cơ bản
- [ ] Tag `w02-done`

---

## 📅 PHASE 3 / TUẦN 3 — Time Sync + Ego-motion + Coordinate Transform (8%)

> 📖 Đọc trước: [knowledge/03d_SENSOR_FUSION_ARCHITECTURE.md](knowledge/03d_SENSOR_FUSION_ARCHITECTURE.md) §2-4

### Day 11 (T2) — Đọc time sync & ego-motion, KHÔNG code nhiều (1h đọc + 1h code khung) (2h)
- Đọc §2-3 file kiến thức: vì sao camera 10Hz / radar 20Hz lệch pha cần interpolate
- Tạo skeleton `TimeSyncBuffer` header (chưa implement logic)

**Commit:** `[W3D11] chore(sync): TimeSyncBuffer skeleton + design note`

### Day 12 (T3) — `TimeSyncBuffer` implementation (2h)
- Ring buffer per-sensor, `getInterpolated(SensorType, Timestamp target)` → linear interpolate 2 detection gần nhất
- Unit test: 2 điểm biết trước → nội suy đúng công thức

**Commit:** `[W3D12] feat(sync): TimeSyncBuffer linear interpolation + test`

### Day 13 (T4) — `EgoMotionCompensator` (2h)
- Đọc oxts (velocity, yaw_rate) → dead-reckoning: `Δx, Δy, Δyaw` giữa 2 timestamp
- Áp dụng lên detection cũ để "kéo" nó về frame ego hiện tại trước khi dùng
- Unit test: ego đứng yên → compensation = identity; ego rẽ 90° → detection xoay đúng

**Commit:** `[W3D13] feat(sync): EgoMotionCompensator dead-reckoning + test`

### Day 14 (T5) — `ExtrinsicCalibration` + `FovGate`/`ConfidenceGate` (2h)
- Ma trận 4x4 (hoặc 2D affine đơn giản hoá) sensor→vehicle frame, hard-code từ KITTI calib file thật (`calib_velo_to_cam` tham khảo cấu trúc, đơn giản hoá 2D)
- `FovGate`: loại detection ngoài góc/range hợp lệ; `ConfidenceGate`: ngưỡng confidence

**Commit:** `[W3D14] feat(transform): ExtrinsicCalibration + FovGate + ConfidenceGate`

### Day 15 (T6) — Integration test Stage 1-3 + push (2h)
- Test end-to-end: 2 sensor → sync → transform → cùng vehicle frame, cùng timestamp
- So sánh vị trí detection sau transform với ground-truth (sai số < threshold)

Push → tag `w03-done`.

**Commit:** `[W3D15] test(integration): sensors→sync→transform pipeline end-to-end`

### ✅ Checkpoint W3 (MUST)
- [ ] Camera + Radar detection cùng timestamp sau interpolation
- [ ] Ego-motion compensation đúng khi ego rẽ
- [ ] Integration test Stage 1→3 pass
- [ ] Tag `w03-done`

---

## 📅 PHASE 4 / TUẦN 4 — Kalman Filter Core, CV Model (9%)

> 📖 Đọc trước: [knowledge/03b_KALMAN_FILTER_ESTIMATION.md](knowledge/03b_KALMAN_FILTER_ESTIMATION.md) §1-3 (bắt buộc hiểu công thức trước khi code)

### Day 16 (T2) — Học lý thuyết KF, viết derivation tay (1.5h đọc + 30' note)
- Viết ra giấy: predict step, update step, ý nghĩa Q (process noise), R (measurement noise), K (Kalman gain)
- Không code — hiểu sai công thức ở đây sẽ lan ra cả pipeline

**Commit:** *(không cần)*

### Day 17 (T3) — Linear algebra utils + `KalmanFilterCV` state (2h)
- Dùng Eigen: `VectorXd x(4)` = [x,y,vx,vy], `MatrixXd P(4,4)`
- Constructor khởi tạo từ detection đầu tiên (position từ đo, velocity = 0, P lớn ban đầu cho velocity)

**Commit:** `[W4D17] feat(estimation): KalmanFilterCV state + Eigen setup`

### Day 18 (T4) — `predict(dt)` implementation + test (2h)
```cpp
// F = [[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]]
x = F * x;
P = F * P * F.transpose() + Q;
```
- Unit test: object velocity const → predict N bước → vị trí khớp closed-form `x0 + v*t`

**Commit:** `[W4D18] feat(estimation): KalmanFilterCV::predict + closed-form test`

### Day 19 (T5) — `update(z)` + Joseph form + PSD assertion (2h)
```cpp
y = z - H * x;
S = H * P * H.transpose() + R;
K = P * H.transpose() * S.inverse();
x = x + K * y;
// Joseph form (numerically stable, giữ P symmetric + PSD):
P = (I - K*H) * P * (I - K*H).transpose() + K * R * K.transpose();
```
- Debug-build assert: `P` symmetric (`(P - P.transpose()).norm() < eps`) và eigenvalue ≥ 0
- Unit test: đo lặp lại nhiều lần → P hội tụ nhỏ dần (uncertainty giảm)

**Commit:** `[W4D19] feat(estimation): KalmanFilterCV::update — Joseph form + PSD assert`

### Day 20 (T6) — Demo end-to-end CV tracking 1 object + push (2h)
- Wire: sensor → sync → transform → KF (1 object thẳng) → in RMSE so với ground-truth
- `ctest` toàn bộ unit test W4 phải xanh (mục tiêu 20/20)

Push → tag `w04-done`.

**Commit:** `[W4D20] feat(demo): 1-object CV tracking end-to-end demo`

### ✅ Checkpoint W4 (MUST)
- [ ] `ctest` 20/20 pass cho KF math
- [ ] `P` luôn symmetric + PSD (assert không fail qua nhiều update)
- [ ] Demo track 1 object đường thẳng, RMSE thấp
- [ ] Tag `w04-done`

---

## 📅 PHASE 5 / TUẦN 5 — EKF / CTRV Model (9%)

> 📖 Đọc trước: [knowledge/03b_KALMAN_FILTER_ESTIMATION.md](knowledge/03b_KALMAN_FILTER_ESTIMATION.md) §4-5 (CTRV + Jacobian)

### Day 21 (T2) — Học CTRV model + Jacobian, viết tay derivation (1.5h đọc + 30' note)
- State `[x,y,v,yaw,yaw_rate]`, non-linear transition khi `yaw_rate ≠ 0`
- Linearization: Jacobian `Fj` của hàm chuyển động — viết công thức ra giấy trước

**Commit:** *(không cần)*

### Day 22 (T3) — `ExtendedKalmanFilterCTRV` state + Jacobian computation (2h)
- `computeJacobian(x, dt)` trả `Fj` 5x5, xử lý case đặc biệt `|yaw_rate| < eps` (tránh chia 0 — dùng xấp xỉ CV cho case này)

**Commit:** `[W5D22] feat(estimation): EKF CTRV state + Jacobian (yaw_rate≈0 special case)`

### Day 23 (T4) — `predict()`/`update()` cho CTRV + test scenario cua (2h)
- Unit test: object rẽ cua đều (constant yaw_rate) → predict N bước → quỹ đạo khớp closed-form vòng tròn

**Commit:** `[W5D23] feat(estimation): EKF CTRV predict/update + curved-path test`

### Day 24 (T5) — Numerical stability tests (2h)
- Test case: Jacobian gần suy biến, yaw_rate đổi dấu đột ngột, detection noise lớn
- Assert P luôn PSD qua toàn bộ scenario dài (1000+ step) — chống drift số học

**Commit:** `[W5D24] test(estimation): EKF numerical stability — near-singular Jacobian`

### Day 25 (T6) — So sánh CV vs CTRV RMSE trên scenario cua + push (2h)
- Chạy cùng 1 scenario cua qua cả 2 model → in bảng RMSE so sánh → note vào docs (bằng chứng "tôi hiểu khi nào dùng model nào")

Push → tag `w05-done`.

**Commit:** `[W5D25] docs+test: CV vs CTRV RMSE comparison on curved scenario`

### ✅ Checkpoint W5 (MUST)
- [ ] EKF CTRV track đúng quỹ đạo cua
- [ ] Numerical stability test pass (1000+ step, P luôn hợp lệ)
- [ ] Bảng so sánh CV vs CTRV ghi vào docs
- [ ] Tag `w05-done`

---

## 📅 PHASE 6 / TUẦN 6 — Data Association + Track Lifecycle (9%)

> 📖 Đọc trước: [knowledge/03c_DATA_ASSOCIATION_TRACKING.md](knowledge/03c_DATA_ASSOCIATION_TRACKING.md) §1-4

### Day 26 (T2) — Học Hungarian algorithm + gating theory (1.5h đọc + 30' note)
- Hiểu complexity O(n³), vì sao cần gate trước (giảm kích thước cost matrix, loại cặp vô lý)
- Chi-square table cho Mahalanobis gating (df=2 → threshold ~5.99 ở 95%)

**Commit:** *(không cần)*

### Day 27 (T3) — `MahalanobisGate` implementation (2h)
```cpp
double d2 = (z - Hx).transpose() * S.inverse() * (z - Hx);
bool gated_out = d2 > chi2_threshold; // df=2, 95% → 5.99
```
- Unit test: detection xa track → bị gate; gần → không bị gate

**Commit:** `[W6D27] feat(association): MahalanobisGate — chi-square threshold`

### Day 28 (T4) — `HungarianAssigner` implementation (2h)
- Implement Hungarian (Kuhn-Munkres) hoặc dùng thư viện nhỏ tự viết O(n³)
- Unit test: cost matrix biết trước kết quả tối ưu (case sách giáo khoa) → assigner ra đúng

**Commit:** `[W6D28] feat(association): HungarianAssigner O(n³) + textbook test`

### Day 29 (T5) — `TrackManager` state machine (2h)
```
Tentative --matched N_confirm frame--> Confirmed
Tentative --unmatched--> Deleted
Confirmed --unmatched--> Coasting (predict-only)
Coasting --matched--> Confirmed
Coasting --unmatched > K_max frame--> Deleted
```
- Unit test: track qua occlusion ngắn (< K_max frame) → không bị xoá, ID giữ nguyên

**Commit:** `[W6D29] feat(tracking): TrackManager lifecycle state machine + occlusion test`

### Day 30 (T6) — Multi-object integration test + push (2h)
- Scenario 5 object, có occlusion chéo nhau → track ID consistency check qua toàn scenario
- Đây là milestone quan trọng: nếu ID nhảy lung tung → phải quay lại debug gating/assignment

Push → tag `w06-done`.

**Commit:** `[W6D30] test(integration): 5-object tracking with occlusion — ID consistency`

### ✅ Checkpoint W6 (MUST — đây là "core story" quan trọng nhất, không được cắt)
- [ ] Hungarian + Mahalanobis gating hoạt động đúng trên cost matrix test
- [ ] Track sống qua occlusion ngắn, ID không đổi
- [ ] 5-object scenario integration test pass
- [ ] Tag `w06-done`

---

## 📅 PHASE 7 / TUẦN 7 — Multi-Sensor Fusion (8%)

> 📖 Đọc trước: [knowledge/03d_SENSOR_FUSION_ARCHITECTURE.md](knowledge/03d_SENSOR_FUSION_ARCHITECTURE.md) §5-6 (track-to-track fusion, covariance intersection)

### Day 31 (T2) — Học covariance intersection + track-to-track fusion (1.5h đọc + 30' note)
- Công thức Covariance Intersection: `P_fused⁻¹ = w·P1⁻¹ + (1-w)·P2⁻¹`, chọn `w` tối ưu hoá trace(P_fused)

**Commit:** *(không cần)*

### Day 32 (T3) — Chạy song song 2 pipeline track (Camera-only, Radar-only) (2h)
- Refactor: mỗi sensor chạy full Stage 4-6 riêng (KF + association + track manager riêng) → 2 danh sách track độc lập

**Commit:** `[W7D32] refactor(fusion): parallel camera-track and radar-track pipelines`

### Day 33 (T4) — `TrackFuser` gating (2h)
- Match camera-track ↔ radar-track (Mahalanobis tương tự Stage 5, nhưng giữa 2 track thay vì track-detection)
- Case không match: track chỉ 1 sensor vẫn output riêng (graceful, không bắt buộc phải có cả 2)

**Commit:** `[W7D33] feat(fusion): TrackFuser gating — match camera-track & radar-track`

### Day 34 (T5) — Covariance Intersection fusion implementation (2h)
- Implement công thức trên, unit test: 2 track cùng object, độ tin cậy khác nhau → fused P nhỏ hơn cả 2 input

**Commit:** `[W7D34] feat(fusion): covariance intersection fusion + test`

### Day 35 (T6) — Integration test: fused RMSE < single-sensor RMSE + push (2h)
- Scenario 2-sensor: so sánh RMSE camera-only, radar-only, fused → fused phải thấp nhất (bằng chứng fusion thực sự có giá trị, không phải chỉ "chạy được")

Push → tag `w07-done`.

**Commit:** `[W7D35] test(integration): fusion RMSE improvement vs single-sensor`

### ✅ Checkpoint W7 (MUST)
- [ ] `TrackFuser` match đúng track cùng object giữa 2 sensor
- [ ] Fused RMSE thấp hơn rõ rệt so với từng sensor riêng
- [ ] Tag `w07-done`

---

## 📅 PHASE 8 / TUẦN 8 — TTC + Threat Assessment (8%)

> 📖 Đọc trước: [knowledge/03e_TTC_FCW_AEB_DECISION.md](knowledge/03e_TTC_FCW_AEB_DECISION.md) §1-2

### Day 36 (T2) — Học công thức TTC + closing-speed kinematics (1.5h đọc + 30' note)
- `TTC = range / closing_speed` khi `closing_speed > 0` (đang tiến lại gần), else `TTC = ∞`
- Viết tay ví dụ số: range=40m, closing_speed=10m/s → TTC=4s

**Commit:** *(không cần)*

### Day 37 (T3) — `TTCEstimator` implementation cơ bản (2h)
- Lấy `range`, `closing_speed` từ fused track (chiếu relative velocity lên trục dọc ego)
- Unit test: input số cụ thể → TTC đúng công thức

**Commit:** `[W8D37] feat(threat): TTCEstimator — closing speed kinematics`

### Day 38 (T4) — `PathPredictor` — lọc object không cùng làn (2h)
- Dự đoán quỹ đạo ego (constant curvature từ yaw_rate hiện tại) trong N giây tới
- Object có vị trí lệch quá `lane_width/2` so với quỹ đạo dự đoán → đánh dấu `in_ego_path = false` (giảm false positive FCW/AEB cho xe làn bên)

**Commit:** `[W8D38] feat(threat): PathPredictor — ego path curvature + in-path filter`

### Day 39 (T5) — Unit test TTC trên scenario hard-brake (2h)
- Scenario: lead vehicle phanh gấp, ego giữ tốc độ → TTC giảm dần theo thời gian, so khớp tính tay ±5%

**Commit:** `[W8D39] test(threat): TTC accuracy on hard-brake scenario ±5%`

### Day 40 (T6) — Integration TTC toàn scenario library subset + push (2h)
- Chạy TTC qua 5+ scenario đã có, log kết quả, note case nào TTC nhảy bất thường (do track chưa ổn định) → note để fix ở Phase sau nếu cần

Push → tag `w08-done`.

**Commit:** `[W8D40] test(integration): TTC across scenario library subset`

### ✅ Checkpoint W8 (MUST)
- [ ] TTC khớp công thức tay trên scenario hard-brake
- [ ] PathPredictor lọc đúng object khác làn
- [ ] Tag `w08-done`

---

## 📅 PHASE 9 / TUẦN 9 — FCW Decision (8%)

> 📖 Đọc trước: [knowledge/03e_TTC_FCW_AEB_DECISION.md](knowledge/03e_TTC_FCW_AEB_DECISION.md) §3

### Day 41 (T2) — Học FCW threshold/hysteresis design (1.5h đọc + 30' note)
- Vì sao cần hysteresis: TTC dao động quanh threshold → không có margin sẽ "chattering" (bật/tắt liên tục, gây phiền tài xế)
- Tham khảo giá trị NCAP-like: `T_warn ≈ 2.6s`, margin reset `+0.4s`

**Commit:** *(không cần)*

### Day 42 (T3) — `FcwDecision` state machine skeleton (2h)
```
Idle --TTC < T_warn, N_confirm frame liên tiếp--> Warn
Warn --TTC > T_warn + margin--> Idle
```
**Commit:** `[W9D42] feat(decision): FcwDecision state machine skeleton`

### Day 43 (T4) — Hysteresis logic implementation (2h)
- N-frame confirm counter (chống 1 frame nhiễu trigger nhầm)
- Unit test: TTC dao động quanh threshold không confirm đủ N frame → không trigger

**Commit:** `[W9D43] feat(decision): FCW hysteresis — N-frame confirm + margin reset`

### Day 44 (T5) — Borderline scenario test chống chattering (2h)
- Scenario riêng: TTC dao động sát ngưỡng nhiều lần → đếm số lần state đổi, phải ít hơn threshold hợp lý (vd < 2 lần đổi trong 5s)

**Commit:** `[W9D44] test(decision): FCW anti-chattering on borderline scenario`

### Day 45 (T6) — Full FCW scenario suite pass + push (2h)
- Chạy toàn bộ scenario library qua FCW → note case nào trigger đúng/sai, tính precision sơ bộ

Push → tag `w09-done`.

**Commit:** `[W9D45] test(integration): FCW full scenario suite`

### ✅ Checkpoint W9 (MUST)
- [ ] FCW trigger đúng threshold trên scenario rõ ràng
- [ ] Không chattering trên scenario borderline
- [ ] Tag `w09-done`

---

## 📅 PHASE 10 / TUẦN 10 — AEB Decision (8%)

> 📖 Đọc trước: [knowledge/03e_TTC_FCW_AEB_DECISION.md](knowledge/03e_TTC_FCW_AEB_DECISION.md) §4-5

### Day 46 (T2) — Học AEB staged braking + suppression rules (1.5h đọc + 30' note)
- Staged: Stage1 (partial brake, cảnh báo mạnh hơn) khi `TTC < 1.6s`; Stage2 (full brake) khi `TTC < 0.9s`
- Suppression: nếu driver đã đạp phanh mạnh hoặc đánh lái tránh → suppress AEB (tránh can thiệp thừa)

**Commit:** *(không cần)*

### Day 47 (T3) — `AebDecision` state machine (2h)
```
Idle --TTC < 1.6s--> Stage1 (partial)
Stage1 --TTC < 0.9s--> Stage2 (full)
Stage1/Stage2 --TTC hồi phục > threshold+margin--> Idle
```
**Commit:** `[W10D47] feat(decision): AebDecision staged state machine`

### Day 48 (T4) — Suppression rule implementation (2h)
- Input giả lập `driver_brake_input`, `driver_steer_input` (từ scenario config)
- Rule: `driver_brake_input > threshold` → suppress Stage2 (driver đã phản ứng, không cần full brake tự động)

**Commit:** `[W10D48] feat(decision): AEB suppression — driver override detection`

### Day 49 (T5) — Scenario test staged trigger order + suppression (2h)
- Test: TTC giảm dần → phải qua Stage1 trước Stage2, không nhảy cóc
- Test: driver brake sớm → Stage2 bị suppress đúng

**Commit:** `[W10D49] test(decision): AEB staged order + suppression correctness`

### Day 50 (T6) — Full AEB scenario suite pass + push (2h)
Push → tag `w10-done`.

**Commit:** `[W10D50] test(integration): AEB full scenario suite`

### ✅ Checkpoint W10 (MUST)
- [ ] AEB staged đúng thứ tự Stage1→Stage2
- [ ] Suppression hoạt động khi driver override
- [ ] Tag `w10-done`

---

## 📅 PHASE 11 / TUẦN 11 — Real-time Engine + Visualization (9%)

> 📖 Đọc trước: [knowledge/03f_REALTIME_CPP_LINUX.md](knowledge/03f_REALTIME_CPP_LINUX.md) toàn bộ

### Day 51 (T2) — Học fixed-rate scheduler + lock-free SPSC queue (1.5h đọc + 30' note)
- Ôn lại pattern từ EventStreamCore (SPSC ring buffer) — tái sử dụng tinh thần, không copy code
- Hiểu budget-per-cycle (50ms @ 20Hz) và ý nghĩa jitter

**Commit:** *(không cần)*

### Day 52 (T3) — `FixedRateScheduler` implementation (2h)
- Loop `sleep_until(next_tick)`, đo thời gian mỗi cycle, log nếu vượt budget (overrun counter)
- Unit test: 100 cycle @ 20Hz → jitter trung bình < 1ms (trên Linux host không real-time kernel, ghi rõ note "best-effort, không phải RTOS hard real-time")

**Commit:** `[W11D52] feat(engine): FixedRateScheduler 20Hz + overrun tracking`

### Day 53 (T4) — `SpscQueue` giữa các stage + wire toàn bộ pipeline (2h)
- Bounded lock-free SPSC queue (tự viết hoặc port pattern từ EventStreamCore, ghi rõ trong docs đây là điểm chung 2 project)
- Wire: Stage 1→2→...→9 qua queue, mỗi stage 1 "tick" trong `FixedRateScheduler`

**Commit:** `[W11D53] feat(engine): SpscQueue inter-stage + wire full pipeline`

### Day 54 (T5) — Profiler p50/p95/p99 (2h)
- Đo cycle time toàn pipeline, in báo cáo p50/p95/p99 sau N cycle
- Test: p99 < budget (50ms) trên toàn bộ scenario library replay tốc độ thật

**Commit:** `[W11D54] feat(engine): cycle-time profiler p50/p95/p99 + budget test`

### Day 55 (T6) — 2D top-down visualization (NICE nhưng nên cố làm) + push (2h)
- SDL2/OpenCV: vẽ ego (tam giác), detection thô (chấm nhạt), fused track (hình chữ nhật + ID), TTC/FCW/AEB indicator (màu đổi theo state)
- Export video 90s từ 1 scenario tiêu biểu (hard-brake) cho `04_PROJECT_SCRIPT.md`

Push → tag `w11-done`.

**Commit:** `[W11D55] feat(viz): 2D top-down renderer + demo scenario capture`

### ✅ Checkpoint W11 (MUST core; visualization NICE)
- [ ] Pipeline chạy fixed-rate 20Hz, p99 jitter đo được và trong budget
- [ ] (NICE) Video demo 2D top-down 90s
- [ ] Tag `w11-done`

---

## 📅 PHASE 12 / TUẦN 12 — Validation Harness + CI + Ship (8%)

> 📖 Đọc trước: [knowledge/03g_TESTING_VALIDATION.md](knowledge/03g_TESTING_VALIDATION.md) toàn bộ

### Day 56 (T2) — MOTA/MOTP calculator + tuning/holdout split (2h)
- `metrics/mota_motp.cpp` (hoặc Python post-process từ log CSV): tính Multi-Object Tracking Accuracy/Precision trên scenario có ground-truth
- Chia 20+ scenario thành **tuning set** (~14 case, dùng để chọn threshold FCW/AEB ở Phase 9-10) và **holdout set** (~6+ case, chưa từng dùng để tune) — chạy MOTA/MOTP trên CẢ hai tập, báo cáo riêng
- Chạy trên 3-5 scenario, in bảng kết quả

**Commit:** `[W12D56] feat(metrics): MOTA/MOTP calculator + tuning/holdout split report`

### Day 57 (T3) — FCW/AEB precision-recall calculator (2h)
- So khớp trigger thực tế vs "nên trigger" (ground-truth label trong scenario config) → precision/recall/F1 toàn bộ scenario library

**Commit:** `[W12D57] feat(metrics): FCW/AEB precision-recall across scenario library`

### Day 58 (T4) — CI pipeline hoàn chỉnh (2h)
- GitHub Actions matrix: gcc + clang, Debug (ASan+UBSan+TSan) + Release
- Chạy `ctest` toàn bộ + upload coverage badge (gcov/lcov, mục tiêu 80%+)
- (Stretch, nếu còn thời gian) job riêng `ci-arm64.yml` cross-compile + chạy qua QEMU, không block merge nếu fail (allow-failure)

**Commit:** `[W12D58] ci: full matrix build + sanitizers (ASan/UBSan/TSan) + coverage badge`

### Day 59 (T5) — README + docs polish (2h)
- README theo format AutoDiag/EventStreamCore: pitch, architecture diagram rút gọn, quickstart build, demo GIF/video link, kill-shot numbers (LOC, test count, MOTA/MOTP, precision/recall)
- Hoàn thiện `04_PROJECT_SCRIPT.md` (nếu chưa xong)

**Commit:** `[W12D59] docs: README polish — pitch, quickstart, kill-shot numbers`

### Day 60 (T6) — Final polish + v1.0 ship + push (2h)
- Review toàn bộ checklist milestone M1-M12
- Squash/cleanup commit lộn xộn nếu cần (không rewrite history đã push public trừ khi chắc chắn an toàn)
- Tag `v1.0`, viết "what I learned" tổng kết 12 tuần

**Commit:** `[W12D60] chore: v1.0 ship — TrackGuard complete`

### ✅ Checkpoint W12 — FINAL (MUST)
- [ ] MOTA/MOTP report có số liệu thật trên scenario library
- [ ] FCW/AEB precision-recall report có số liệu thật
- [ ] CI matrix xanh (gcc+clang × Debug/Release, sanitizers clean)
- [ ] README + demo video/GIF hoàn chỉnh
- [ ] Tag `v1.0`

---

## 📊 Tổng kết KPI mục tiêu sau 60 ngày

| Metric | Target |
|---|---|
| LOC | ~9,850 core (~11,500 với stretch S1-S6) |
| Test count (unit + scenario integration) | 150+ |
| Scenario library | 20+ case (cut-in, hard-brake, pedestrian, false-positive traps, weather/occlusion, ...) — chia tuning set / holdout set |
| MOTA (trên scenario có ground-truth) | Báo cáo thật trên CẢ tuning + holdout set — quan trọng là đúng phương pháp đo, không chỉ đẹp số |
| FCW/AEB precision/recall | Báo cáo thật trên scenario library (tuning + holdout riêng biệt) |
| CI | Green, matrix gcc/clang × Debug(ASan/UBSan/TSan)/Release |
| Commits | 60 (1/ngày) |
| Tags | 12 (`w01-done` → `w12-done`) + `v1.0` (+ `v1.1` nếu hoàn thành stretch) |

> Sau khi đạt KPI này, chuyển sang [05_INTERVIEW_PREP.md](05_INTERVIEW_PREP.md) để ôn phỏng vấn, và cân nhắc Phase 13 (Stretch Week) bên dưới nếu còn thời gian.

---

## 🎁 PHASE 13 / STRETCH WEEK (Bonus — chỉ bắt đầu sau khi v1.0 xanh)

> 📖 Đọc trước: [knowledge/03b_KALMAN_FILTER_ESTIMATION.md](knowledge/03b_KALMAN_FILTER_ESTIMATION.md) phần UKF/IMM (nếu đã bổ sung), [00_MASTER_PLAN.md §11](00_MASTER_PLAN.md)
> **Nguyên tắc:** không đụng vào core P1-P12 nữa — chỉ thêm module mới, không sửa lại API cũ (trừ khi bug thật).

### Day 61-62 — Unscented Kalman Filter (UKF) cho CTRV (4h)
- Sigma-point generation (2n+1 điểm), unscented transform cho predict/update, không cần Jacobian
- Unit test: so sánh RMSE UKF vs EKF trên scenario cua gấp (yaw_rate lớn) — kỳ vọng UKF RMSE thấp hơn hoặc bằng EKF

**Commit:** `[STRETCH-D61] feat(estimation_stretch): UnscentedKalmanFilterCTRV + RMSE comparison`

### Day 63-64 — IMM (Interacting Multiple Model) blend CV + CTRV (4h)
- Model probability mixing (Markov transition matrix giữa CV/CTRV), combined state estimate theo trọng số xác suất
- Unit test: object đổi từ đi thẳng sang rẽ đột ngột → IMM thích nghi nhanh hơn either single model

**Commit:** `[STRETCH-D63] feat(estimation_stretch): ImmEstimator — CV/CTRV probability blending`

### Day 65 — ARM64 cross-compile + QEMU CI (2h)
- `toolchains/arm64-toolchain.cmake`, build thử local qua `qemu-aarch64`
- `.github/workflows/ci-arm64.yml` (allow-failure job, không block merge)

**Commit:** `[STRETCH-D65] ci: ARM64 cross-compile + QEMU smoke test`

### Day 66 — ROS2 publisher wrapper (2h)
- `include/trackguard/interop/ros2_publisher.h`, publish `FusedTrack`/`DecisionOutput` dạng message tối giản, config-gated (`enable_ros2_publisher: false` mặc định)

**Commit:** `[STRETCH-D66] feat(interop): minimal ROS2 publisher wrapper (config-gated)`

### Day 67-68 — Mở rộng scenario library lên 25 case + báo cáo so sánh cuối (4h)
- Thêm scenario weather/occlusion dài/multi-lane phức tạp
- Viết báo cáo so sánh: CV vs CTRV vs UKF vs IMM (RMSE table), tag `v1.1`

**Commit:** `[STRETCH-D68] docs+test: extended scenario library + CV/CTRV/UKF/IMM comparison report — v1.1`

### ✅ Checkpoint Stretch (NICE, không bắt buộc cho portfolio-ready)
- [ ] UKF + IMM implemented, RMSE comparison report có số liệu thật
- [ ] ARM64 CI chạy được qua QEMU (allow-failure OK)
- [ ] ROS2 wrapper build được, config-gated đúng (mặc định tắt không ảnh hưởng core)
- [ ] Tag `v1.1`
