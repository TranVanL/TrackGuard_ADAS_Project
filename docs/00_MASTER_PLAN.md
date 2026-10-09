# 🚘 TrackGuard — Master Plan (12 tuần / 60 ngày, mỗi tối 2-3h)

> **Tagline:** *Perception + Decision plane cho ADAS — C++17/Linux end-to-end: sensor replay → time sync/ego-motion → coordinate transform → Kalman/EKF state estimation → data association (Hungarian) → multi-object track management → multi-sensor (camera+radar) track fusion → TTC threat assessment → FCW/AEB decision. Primary target: ADAS Software Engineer / Sensor Fusion Engineer / Perception (classical) Engineer / Autonomous Driving C++ Engineer. Đứng cạnh [EventStreamCore](https://github.com/TranVanL/eventstreamCore) (real-time data plane, C++ depth) và [AutoDiag/VDiag](../AutoDiag/00_MASTER_PLAN.md) (Android control-plane/HAL) — TrackGuard là project thứ 3, chứng minh hiểu ADAS software toàn trình từ sensor → fusion → tracking → decision, không phải chỉ 1 thuật toán rời rạc.*

---

## 0. Triết lý design

| Nguyên tắc | Cụ thể |
|---|---|
| **Algorithm-honest, không phải demo-only** | Mỗi stage phải đúng toán học (KF derivation, Hungarian complexity, TTC kinematics) VÀ có test chứng minh đúng — không phải "chạy được là xong" |
| **Dataset-driven, không cần xe/sensor thật** | Replay từ **KITTI tracking subset** (public, free) + **synthetic scenario generator** tự viết cho edge case (cut-in, hard-brake lead vehicle, pedestrian crossing, false-positive traps) |
| **Classical perception, KHÔNG train model riêng** | Dùng detection có sẵn từ dataset (hoặc synthetic ground-truth + noise injection) — trọng tâm là **fusion/tracking/decision C++**, không lạc đề sang deep learning/CV training |
| **Track-level (late) fusion — chọn có lý do** | Camera track + Radar track → fuse ở track level (confidence-weighted covariance) — dễ chứng minh, dễ mở rộng thêm sensor, nói rõ tradeoff với low-level/early fusion trong interview |
| **Real-time pipeline, tinh thần EventStreamCore** | Staged pipeline + deterministic fixed-rate scheduler (20Hz) + lock-free/bounded queue giữa stage — không phải "for loop chạy 1 lần rồi print" |
| **Safety-honest framing** | KHÔNG claim production/certified AEB. Chứng minh hiểu **kiến trúc + thuật toán + validation methodology** (ISO 26262 ở mức concept, NCAP test protocol ở mức tham khảo) |
| **Requirements-traceable & safety-concept-inspired** | Mọi feature quan trọng có FR/NFR ID trace từ [06_REQUIREMENTS_TRACEABILITY.md](06_REQUIREMENTS_TRACEABILITY.md) tới design tới test; FCW/AEB có HARA-style hazard analysis + safety goals ở [07_SAFETY_HAZARD_ANALYSIS.md](07_SAFETY_HAZARD_ANALYSIS.md) — **portfolio-quality, không certified**, nhưng chứng minh đúng tư duy quy trình automotive |
| **Coding standard & static analysis như production repo thật** | Subset MISRA C++/AUTOSAR C++14-inspired (24 rule, [08_CODING_STANDARDS_STATIC_ANALYSIS.md](08_CODING_STANDARDS_STATIC_ANALYSIS.md)) + clang-tidy/cppcheck/Coverity làm required CI gate, không chỉ sanitizer |
| **Buổi tối 2-3h/ngày** | Mỗi day-task ≤ 2h code + 30' đọc/note, giống format AutoDiag |
| **Cắt sớm nếu trễ** | Mỗi Phase có MUST (must-have) và NICE (cut nếu trễ) |
| **Hết tuần phải push** | Tag `wXX-done`, viết 5 dòng "what I learned" |

---

## 1. Cốt lõi: 12 boundary, 12 câu chuyện kể

```
[Stage -1] Configuration & Determinism Manager     ← seeded RNG, YAML config, reproducible build hash
   ── Boundary Z: Config/Determinism ──
[Stage 0] Scenario/Dataset Replay Harness         ← KITTI tracking subset + synthetic scenario DSL
   ── Boundary A: Sensor Ingest (ISensorSource) ──
[Stage 1] CameraDetectionSource | RadarDetectionSource   (deterministic replay, timestamped)
   ── Boundary B: Time Sync + Ego-motion Compensation ──
[Stage 2] TimeSyncBuffer (interpolation) + EgoMotionCompensator (odometry/IMU dead-reckoning)
   ── Boundary C: Coordinate Transform + Calibration ──
[Stage 3] Extrinsic calibration (sensor→vehicle frame) + FOV/confidence gating
   ── Boundary D: State Estimation ──
[Stage 4] KalmanFilter (CV model) + ExtendedKalmanFilter (CTRV model) per-track (+ optional UKF comparison)
   ── Boundary E: Data Association ──
[Stage 5] Hungarian Algorithm (O(n³)) + Mahalanobis/IoU gating
   ── Boundary F: Track Lifecycle Management ──
[Stage 6] TrackManager: Tentative → Confirmed → Coasting → Deleted (occlusion-aware)
   ── Boundary G: Multi-sensor Fusion ──
[Stage 7] TrackFuser: camera+radar track-to-track fusion (confidence-weighted covariance intersection)
   ── Boundary H: Threat Assessment ──
[Stage 8] TTCEstimator: closing kinematics + path/curvature prediction
   ── Boundary I: Decision Layer ──
[Stage 9] FCW (hysteresis alert state machine) + AEB (staged braking + suppression rules)
   ── Boundary J: Real-time Engine ──
[Engine]  Fixed-rate 20Hz scheduler + lock-free inter-stage queues + p50/p95/p99 profiling
   ── Boundary K: Validation & Metrics ──
[QA]      ScenarioRunner (20+ case) + MOTA/MOTP + FCW/AEB precision-recall + gtest + ASan/UBSan/TSan + CI matrix
   ── Boundary L: Portability & Interop (stretch) ──
[Bonus]   ARM64 cross-compile + QEMU CI + minimal ROS2 publisher wrapper (config-gated, off by default)
```

**12 câu chuyện = 12 deep technical stories kể trong interview:**
1. **Config & determinism** → seeded RNG toàn hệ thống, config hash ghi vào mọi report — chứng minh kết quả reproducible, không phải "may mắn chạy 1 lần"
2. **Sensor ingest & replay** → `ISensorSource` factory, deterministic replay từ log file, no-flakiness testing
3. **Time sync & ego-motion** → interpolation buffer cho lệch timestamp camera/radar + dead-reckoning bù chuyển động ego giữa 2 frame
4. **Coordinate transform & calibration** → extrinsic matrix sensor→vehicle, tại sao sai calibration làm fusion lệch
5. **Kalman Filter state estimation** → CV vs CTRV (vs UKF stretch) model, predict/update, tại sao cần Joseph form giữ covariance PSD
6. **Data association** → Hungarian algorithm assignment, Mahalanobis gating (chi-square test) chống mis-association
7. **Track lifecycle** → state machine Tentative/Confirmed/Coasting/Deleted, xử lý occlusion không xoá track quá sớm
8. **Multi-sensor fusion** → track-to-track fusion, covariance intersection, vì sao chọn late fusion thay vì early fusion
9. **Threat assessment (TTC)** → công thức TTC từ closing speed, path prediction khi xe cua
10. **Decision (FCW/AEB)** → hysteresis chống chattering, staged braking, false-positive suppression — thiết kế giống tinh thần ISO 26262 (không certified)
11. **Real-time engine & validation** → fixed-rate scheduler + lock-free queue (reuse pattern EventStreamCore) + MOTA/MOTP + precision/recall trên scenario library
12. **Portability & interop (stretch)** → ARM64 + QEMU CI (giống AutoDiag P19) + ROS2 publisher tối giản — chứng minh core logic không khoá vào 1 nền tảng

---

## 2. Code budget (~11,500 LOC source — vượt tầm 2 project kia về chiều sâu thuật toán)

| Layer | Files | LOC | Lang |
|---|---|---|---|
| Config & Determinism Manager (seeded RNG, YAML config, hash) | 3 | 250 | C++ |
| Dataset loader (KITTI tracking parser) + replay harness | 6 | 700 | C++ |
| Synthetic scenario generator (DSL + scenario lib: 20+ case) | 6 | 800 | C++/YAML |
| Time sync buffer + ego-motion compensation | 3 | 350 | C++ |
| Coordinate transform + extrinsic calibration | 3 | 300 | C++ |
| Kalman Filter core: linear algebra + CV + CTRV/EKF | 6 | 900 | C++ |
| **(stretch)** Unscented Kalman Filter (UKF) comparison module | 3 | 400 | C++ |
| Data association: Hungarian + Mahalanobis/IoU gating | 4 | 650 | C++ |
| Track manager (lifecycle state machine) | 3 | 400 | C++ |
| Multi-sensor fusion (TrackFuser + covariance intersection) | 3 | 400 | C++ |
| **(stretch)** IMM (Interacting Multiple Model) filter blend CV/CTRV | 3 | 450 | C++ |
| TTC + threat assessment (path/curvature prediction) | 3 | 350 | C++ |
| FCW/AEB decision logic + suppression rules | 4 | 500 | C++ |
| Real-time engine (fixed-rate scheduler, lock-free queues, profiling) | 5 | 700 | C++ |
| Visualization (2D top-down renderer, SDL2/OpenCV + python export) | 4 | 500 | C++/Python |
| Metrics (MOTA/MOTP, FCW/AEB precision-recall calculators) | 3 | 350 | C++/Python |
| Tests: gtest unit (KF/EKF/UKF/IMM, association, TTC, decision) + scenario integration | 28 | 2700 | C++ |
| CMake + CI (matrix build, sanitizers, ARM64 + QEMU) | 7 | 350 | CMake/YAML |
| **(stretch)** ROS2 publisher wrapper (config-gated) | 2 | 200 | C++ |
| Docs (planning + knowledge + interview prep, ~20 files) | 20 | — | MD |
| **Total (core, no stretch)** | **~95** | **~9,850** | |
| **Total (with stretch modules)** | **~103** | **~11,500** | |

> **Chiến lược LOC:** core (P1-P12, không stretch) đã ~9,850 LOC — ngang AutoDiag/EventStreamCore. Stretch modules (UKF, IMM, ROS2) đẩy lên ~11,500 LOC **chỉ khi core đã xong và test xanh** — không bao giờ đánh đổi chất lượng core lấy thêm LOC stretch (xem [§11 Stretch Goals](#11-stretch-goals-nếu-còn-dư-thời-gian-sau-day-60)).

---

## 3. Lộ trình 12 tuần (60 ngày code, mỗi tối 2-3h)

> **v1.0 (Days 1-30):** Perception core — replay, sync, transform, state estimation, association, track lifecycle
> **v2.0 (Days 31-45):** Fusion + decision — multi-sensor fusion, TTC, FCW, AEB
> **v3.0 (Days 46-60):** Real-time engine + validation + ship — scheduler, visualization, metrics, CI, polish

| Phase | Tuần | Tên | Cuối tuần demo | Effort |
|---|---|---|---|---|
| **P1** | W1 (D1-5) | Setup + dataset/scenario replay harness | Replay KITTI clip → console log detections theo timestamp | 8% |
| **P2** | W2 (D6-10) | Sensor adapters (Camera/Radar) + determinism tests | 2 `ISensorSource` impl, replay bit-identical 2 lần | 8% |
| **P3** | W3 (D11-15) | Time sync + ego-motion compensation + coordinate transform | Camera/radar detection cùng frame vehicle, cùng timestamp | 8% |
| **P4** | W4 (D16-20) | Kalman Filter core (CV model) + linear algebra + gtest | `ctest` 20/20 KF math pass, 1D/2D CV demo track thẳng | 9% |
| **P5** | W5 (D21-25) | EKF/CTRV model + per-track filter + numerical stability | Track xe cua đúng quỹ đạo, covariance luôn PSD (test) | 9% |
| **P6** | W6 (D26-30) | Data association (Hungarian + gating) + Track lifecycle | Multi-object track ổn định qua occlusion ngắn, ID không đổi | 9% |
| **P7** | W7 (D31-35) | Multi-sensor fusion (TrackFuser) | Camera+radar → 1 fused track, sai số giảm so với 1 sensor | 8% |
| **P8** | W8 (D36-40) | TTC + threat assessment | TTC đúng công thức trên scenario hard-brake (so với ground-truth) | 8% |
| **P9** | W9 (D41-45) | FCW decision (hysteresis alert state machine) | FCW trigger đúng threshold, không chatter trên scenario borderline | 8% |
| **P10** | W10 (D46-50) | AEB decision (staged braking + suppression) | AEB trigger đúng staged logic + suppression driver-override | 8% |
| **P11** | W11 (D51-55) | Real-time engine (scheduler + lock-free queue) + visualization | Pipeline chạy fixed-rate 20Hz, jitter < 1ms; 2D top-down demo video | 9% |
| **P12** | W12 (D56-60) | Validation harness (MOTA/MOTP, precision/recall) + CI + v1.0 ship | Scenario library 10+ case, metrics report, CI green, tag v1.0 | 8% |

---

## 4. Daily commitment

- **Thứ 2-6**: 2-3h/tối (chính)
- **Thứ 7**: optional 1-2h catch-up
- **Chủ nhật**: nghỉ
- **Mỗi day-task**: ≤ 2h code + ≤ 30' note → 1 commit mỗi ngày
- **Cuối tuần**: push + tag `wXX-done` + 5 dòng "what I learned" trong `docs/lessons.md`

---

## 5. Milestones

| Milestone | Sau Phase | Demo 60-90s |
|---|---|---|
| **M1 — Replay alive** | P1 (W1) | `./replay kitti_0001.txt` → detections in đúng thứ tự, đúng timestamp |
| **M2 — Sensors abstracted** | P2 (W2) | Camera + Radar adapter, swap qua factory, replay deterministic |
| **M3 — Frames aligned** | P3 (W3) | Detection 2 sensor cùng vehicle-frame, cùng timestamp sau interpolation |
| **M4 — KF alive** | P4 (W4) | `ctest` 20/20; track 1 object đường thẳng, RMSE < ground-truth threshold |
| **M5 — EKF cua tốt** | P5 (W5) | Track object rẽ cua, quỹ đạo khớp ground-truth, covariance luôn hợp lệ |
| **M6 — Multi-object tracking ổn định** | P6 (W6) | 5+ object cùng lúc, ID consistency qua occlusion 5 frame |
| **M7 — Fusion giảm sai số** | P7 (W7) | Fused track RMSE thấp hơn camera-only và radar-only |
| **M8 — TTC đúng công thức** | P8 (W8) | TTC scenario hard-brake khớp tính tay ±5% |
| **M9 — FCW không chatter** | P9 (W9) | Scenario borderline TTC dao động quanh threshold → FCW không nhấp nháy |
| **M10 — AEB staged đúng logic** | P10 (W10) | 3-stage braking trigger đúng thứ tự, suppression khi driver brake |
| **M11 — Real-time xanh + video demo** | P11 (W11) | Jitter < 1ms @20Hz; video 2D top-down 90s |
| **M12 — v1.0 ship** | P12 (W12) | 20+ scenario, MOTA/MOTP report, CI matrix green (ASan/UBSan/TSan), README + demo video, tag `v1.0` |
| **M13 — v1.1 stretch (nếu còn thời gian)** | Post-P12 | UKF/IMM comparison report + ARM64 QEMU CI xanh + ROS2 publisher demo + interview prep 150+ Q&A hoàn chỉnh |

---

## 6. JD Coverage Matrix — ADAS / Perception / Autonomous Driving

| JD Theme | Phase | Proof in project |
|---|---|---|
| Sensor fusion (camera + radar) | P3, P7 | Coordinate transform + `TrackFuser` covariance-weighted fusion |
| State estimation / Kalman filter | P4-P5 | CV + CTRV/EKF, Joseph form, numerical stability tests |
| Advanced estimation (UKF/IMM) | Stretch | Unscented transform + Interacting Multiple Model blend, RMSE comparison report |
| Multi-object tracking (MOT) | P6 | Hungarian assignment + gating + track lifecycle state machine |
| Data association algorithms | P6 | Mahalanobis (chi-square gating) + IoU, complexity analysis |
| Collision warning / TTC | P8-P9 | `TTCEstimator` + FCW hysteresis state machine |
| AEB / decision logic | P10 | Staged braking + suppression rules, safety-inspired design |
| Functional-safety-adjacent validation | P12 | Scenario-based validation methodology, precision/recall, honesty framing tinh thần ISO 26262 |
| Real-time C++ systems | P11 | Fixed-rate scheduler + lock-free queues (shared DNA với EventStreamCore) |
| Validation / metrics (MOTA/MOTP) | P12 | Scenario library + metrics report + CI |
| Robotics / perception generalist | P1-P8 | Sensor abstraction + estimation + tracking transfer trực tiếp sang robot perception stack |
| Test/Validation Engineer (ADAS) | P12 | ScenarioRunner framework + metrics + CI — trực tiếp áp dụng cho vai trò QA/validation ADAS |
| Functional Safety Engineer (junior/mid) | Toàn bộ | HARA-style hazard analysis + safety goals + functional safety concept ([07](07_SAFETY_HAZARD_ANALYSIS.md)), rõ ràng "reference, not certified" |
| Requirements/Systems Engineer | Toàn bộ | SRS + traceability matrix FR/NFR→design→test ([06](06_REQUIREMENTS_TRACEABILITY.md)) |
| Automotive Software Quality / Static Analysis | Toàn bộ | MISRA/AUTOSAR-inspired coding standard + clang-tidy/cppcheck/Coverity CI gate ([08](08_CODING_STANDARDS_STATIC_ANALYSIS.md)) |
| Embedded/portability | Stretch | ARM64 cross-compile + QEMU CI giống AutoDiag P19 |
| ROS/robotics interop | Stretch | Minimal ROS2 publisher wrapper quanh core pipeline (config-gated) |
| C++17/Linux depth | Toàn bộ | ~9.8-11.5K LOC C++17, 150+ test, sanitizers, CMake matrix |

---

## 7. Vị trí trong bộ 3 project — chiến lược "mỗi JD chọn 2/3"

| Aspect | EventStreamCore | AutoDiag (VDiag) | **TrackGuard** |
|---|---|---|---|
| **Plane** | Data plane / streaming | Control plane / Android system | **Perception + Decision plane / ADAS** |
| **Strength shown** | Modern C++17, lock-free, p99 < 2µs | Android framework (AAOS + modern arch) | **Sensor fusion, estimation theory, tracking, decision logic** |
| **Primary JD target** | C++/POSIX/Real-time | Senior Android (Automotive/Framework) | **ADAS / Sensor Fusion / Perception (classical) / AD C++** |
| **"Wow" factor** | 10M ev/s, cache-line, SPSC ring | AAOS 1-1 + MVVM + Security + Testing | **KF math đúng + Hungarian + TTC + FCW/AEB scenario-validated** |
| **LOC** | ~10K | ~10K | **~9.8K core / ~11.5K với stretch** |
| **Tests** | 200+ | 140+ | **150+ (unit + scenario, gtest)** |
| **Interview Q&A prepared** | — (phỏng vấn C++/real-time chung) | 170+ (4 file) | **150+ (5 file, xem [05_INTERVIEW_PREP.md](05_INTERVIEW_PREP.md))** |
| **CI matrix** | gcc/clang | gcc/clang + ARM64 | gcc/clang + ARM64 (stretch) + ASan/UBSan/TSan |
| **Demo** | Throughput chart | 3-min video: app → Binder → JNI → HAL | **2D top-down replay video: sensor dots → fused track → FCW/AEB trigger** |

> **Kết luận số liệu:** TrackGuard không thua kém 2 project kia về quy mô (LOC, test count), và có bộ Q&A interview riêng riết sâu không kém AutoDiag — đủ để đứng ngang hàng khi so sánh trực tiếp trong portfolio.

### Ma trận chọn project theo JD (2/3)

| JD type | Project 1 | Project 2 |
|---|---|---|
| ADAS / Sensor Fusion / Perception | **TrackGuard** | AutoDiag *(HAL abstraction, ADAS sensor HAL angle)* |
| Autonomous Driving C++ / Robotics | **TrackGuard** | EventStreamCore *(real-time C++ depth)* |
| Senior Android Automotive/Framework | AutoDiag | EventStreamCore |
| Embedded/Real-time C++ (generic) | EventStreamCore | **TrackGuard** *(fixed-rate scheduler, lock-free)* |
| Functional-safety-adjacent (validation) | **TrackGuard** *(scenario validation, MOTA/MOTP)* | AutoDiag *(bring-up, QA)* |

> **Pitch tổng:** *"Ba project không đua nhau — mỗi cái chứng minh 1 mặt: EventStreamCore = real-time C++ throughput; AutoDiag = Android system/HAL engineering; TrackGuard = ADAS domain depth từ sensor đến decision. Với JD ADAS, tôi dẫn bằng TrackGuard, dùng AutoDiag hoặc EventStreamCore làm project phụ tuỳ JD nhấn Android hay nhấn real-time."*

---

## 8. Risk register (top 6)

| # | Risk | Mitigation |
|---|---|---|
| R1 | Lạc đề sang deep learning / train model | Cấm tuyệt đối: dùng detection có sẵn (dataset ground-truth + noise) — trọng tâm là fusion/tracking/decision C++ |
| R2 | Dataset KITTI license/dung lượng lớn | Chỉ dùng **subset nhỏ** (2-3 sequence, vài trăm MB) + tự sinh synthetic scenario cho phần còn lại |
| R3 | Hungarian algorithm / KF có bug số học tinh vi (covariance không PSD, NaN) | Bắt buộc unit test cho từng công thức TRƯỚC khi build pipeline tổng (P4-P6), assert symmetric+PSD trong debug build |
| R4 | 60 ngày trễ deadline | Visualization, UKF/IMM, ROS2, ARM64 là NICE/stretch — có thể cut, core P1-P10 + P12 (metrics/CI) vẫn ship được |
| R5 | Scope creep từ stretch goals ăn lấn thời gian core | Khoá quy tắc: stretch (§11) chỉ bắt đầu sau khi TOÀN BỘ checkpoint P1-P12 đã xanh, không xự vào giữa tuần |
| R6 | Overfitting threshold (FCW/AEB) vào đúng scenario đã viết, không generalize | Chia scenario thành 2 tập: "tuning set" (dùng để chọn threshold) và "holdout set" (chỉ dùng để báo cáo cuối, không dùng để tune) — giống train/test split, chứng minh hiểu overfitting ngay cả với rule-based system |

---

## 9. File index

| File | Mục đích | Khi cần đọc |
|---|---|---|
| **00_MASTER_PLAN.md** ← | Roadmap tổng, KPI, milestones, vị trí trong bộ 3 project | Mỗi đầu Phase |
| **01_ARCHITECTURE.md** | Full pipeline diagram (12 boundaries) + class design từng stage | Trước khi code mỗi layer |
| **02_PLAN_DETAILED_60DAY.md** | Day-by-day 60 ngày: học gì, code ở đâu, output/DoD, commit message | Mỗi tối trước khi code |
| **04_PROJECT_SCRIPT.md** | Elevator pitch, demo script, positioning script, honesty framing, FAQ | Trước phỏng vấn / trước quay demo |
| **06_REQUIREMENTS_TRACEABILITY.md** | SRS (FR/NFR) + traceability matrix requirement→design→test | Trước khi code mỗi feature quan trọng (viết ID trước, code sau) |
| **07_SAFETY_HAZARD_ANALYSIS.md** | HARA-style hazard analysis, safety goals, functional safety concept (portfolio-quality, không certified) | Trước khi thiết kế FCW/AEB (P8-P10), ôn trước phỏng vấn functional-safety-adjacent |
| **08_CODING_STANDARDS_STATIC_ANALYSIS.md** | Coding standard subset (24 rule), static analysis toolchain, review checklist, versioning | Đọc 1 lần trước P1, tra cứu liên tục khi code |
| **knowledge/03_KNOWLEDGE_INDEX.md** | **HUB** — mục lục kiến thức nền cần học trước/trong khi code | Bắt đầu ở đây trước P1 |
| **knowledge/03a_SENSORS_PERCEPTION.md** | Camera/radar/lidar model, detection format, noise model, FOV | Trước P2 |
| **knowledge/03b_KALMAN_FILTER_ESTIMATION.md** | KF/EKF/UKF math, CV/CTRV/IMM model, tuning Q/R, numerical stability | Trước P4-P5 |
| **knowledge/03c_DATA_ASSOCIATION_TRACKING.md** | Hungarian algorithm, gating, track lifecycle, MOTA/MOTP | Trước P6 |
| **knowledge/03d_SENSOR_FUSION_ARCHITECTURE.md** | Early vs late fusion, track-to-track fusion, calibration, time sync | Trước P3, P7 |
| **knowledge/03e_TTC_FCW_AEB_DECISION.md** | TTC formula, FCW hysteresis, AEB staged logic, NCAP/ISO 26262 tham khảo | Trước P8-P10 |
| **knowledge/03f_REALTIME_CPP_LINUX.md** | Fixed-rate scheduler, lock-free queue, profiling, sanitizers | Trước P11 |
| **knowledge/03g_TESTING_VALIDATION.md** | Unit test strategy, scenario-based testing, metrics, CI | Trước P12 |
| **05_INTERVIEW_PREP.md** | **HUB** — 150+ Q&A phỏng vấn, chia 5 file con theo chủ đề | Trước phỏng vấn (bắt đầu ở đây) |
| **05a_INTERVIEW_FOUNDATION.md** | Sensor/replay/sync/transform/architecture Q&A | Ôn trước vòng foundation |
| **05b_INTERVIEW_ESTIMATION_TRACKING.md** | KF/EKF/UKF/IMM + Hungarian/gating/tracking Q&A | Ôn trước vòng thuật toán |
| **05c_INTERVIEW_FUSION_DECISION.md** | Fusion + TTC/FCW/AEB Q&A | Ôn trước vòng system design ADAS |
| **05d_INTERVIEW_SYSTEM_VALIDATION.md** | Real-time engine, testing, metrics, CI Q&A | Ôn trước vòng engineering practice |
| **05e_INTERVIEW_BEHAVIORAL.md** | Level defense, STAR stories, honesty framing, curveballs, reverse questions | Ôn ngày trước phỏng vấn |

---

## 10. Quy tắc "khi nào dừng"

Nếu **bất cứ Phase nào trễ > 1 tuần** → cắt theo thứ tự ưu tiên:

**v1.0 (Days 1-30) — TUYỆT ĐỐI GIỮ:**
1. P1-P3: Replay + sensor adapter + sync/transform (nền tảng, không cắt được)
2. P4-P6: KF/EKF + association + track lifecycle (core "hiểu sâu" story)

**v2.0-v3.0 (Days 31-60) — cắt nếu trễ (thứ tự ưu tiên cắt):**
1. **Cut đầu tiên**: Visualization (P11 phần render) — giữ lại text/CSV log demo, note "video polish cut do thời gian"
2. **Cut thứ 2**: ARM64 cross-compile bonus — note "portable-by-design (pure C++17/CMake, không phụ thuộc OS-specific API)" trong docs
3. **Giữ bằng mọi giá**: P7 (fusion), P8-P10 (TTC/FCW/AEB — đây là phần "sánh vai" với 2 project kia), P12 (metrics + CI, vì đây là bằng chứng "không phải demo suông")

> **Mục tiêu cuối (tuần 12 / ngày 60)**: repo ~9.8K LOC core (~11.5K với stretch), 150+ test, 20+ scenario validated, MOTA/MOTP + FCW/AEB precision-recall report, CI matrix green (ASan/UBSan/TSan) + static analysis gate sạch (clang-tidy/cppcheck/Coverity), requirements traceability matrix đầy đủ, HARA-style safety concept cho FCW/AEB, video demo 90s, tag `v1.0` → sẵn sàng làm **main project cho mọi JD ADAS/Sensor Fusion/Perception/Autonomous Driving C++**, và là **project phụ mạnh** cho JD Embedded/Real-time C++ tổng quát, Functional Safety, và Requirements/Systems Engineering.

---

## 11. Stretch Goals (nếu còn dư thời gian sau Day 60)

> **Nguyên tắc:** chỉ bắt đầu stretch KHI toàn bộ checkpoint P1-P12 đã xanh và tag `v1.0` đã push. Không xự stretch vào giữa core — rủi ro lớn nhất của portfolio là core dở dang vì ham thêm tính năng.

| # | Stretch goal | Giá trị chứng minh | Effort ước tính |
|---|---|---|---|
| S1 | **Unscented Kalman Filter (UKF)** cho CTRV, so sánh RMSE với EKF | Hiểu sigma-point transform, khi nào UKF tốt hơn EKF (non-linearity mạnh) | 3-4 buổi tối |
| S2 | **IMM (Interacting Multiple Model)** blend CV + CTRV theo xác suất | Hiểu multi-model estimation — hướng production thật hay dùng | 4-5 buổi tối |
| S3 | **ARM64 cross-compile + QEMU CI** (giống AutoDiag P19) | Chứng minh core pipeline không khoá vào x86, portable sang embedded target | 2-3 buổi tối |
| S4 | **ROS2 publisher wrapper** tối giản (publish `FusedTrack`/`DecisionOutput` dạng ROS2 message) | Interop với hệ sinh thái robotics phổ biến, không thay đổi core logic | 2 buổi tối |
| S5 | Mở rộng scenario library lên 20-25 case (thêm weather/occlusion dài/multi-lane change phức tạp) | Validation rộng hơn, báo cáo MOTA/MOTP đáng tin hơn | 3 buổi tối |
| S6 | Web-based replay viewer xuất từ log CSV (Plotly/D3, không cần build native) | Dễ chia sẻ demo qua link thay vì video | 2 buổi tối |

> Nếu hoàn thành S1+S2, LOC thực tế tiến gần **~11,500** như ước tính ở §2, và câu chuyện "tôi hiểu cả IMM/UKF" là điểm cộng rất lớn với JD Sensor Fusion senior.

---

## 12. Interview Readiness

Sau khi ship `v1.0` (và tốt nhất là cả stretch S1-S2), chuyển sang ôn phỏng vấn theo [05_INTERVIEW_PREP.md](05_INTERVIEW_PREP.md) — hub dẫn tới 5 file con với **150+ Q&A** (short/deep/trap style, giống format AutoDiag) bao phủ toàn bộ 12 boundary + behavioral + system design + honesty framing. Lộ trình ôn 7 ngày trước phỏng vấn có sẵn trong file hub.
