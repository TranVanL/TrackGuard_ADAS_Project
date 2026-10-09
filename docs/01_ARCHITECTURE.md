# 🏗 TrackGuard — Architecture (Sensor → Fusion → Tracking → Decision Pipeline)

> **Core insight:** TrackGuard là một **staged real-time pipeline** giống kiến trúc perception/decision thật của xe ADAS cấp L2: mỗi stage là 1 module thay được (giống HAL abstraction bên AutoDiag), dữ liệu chảy qua bounded queue theo nhịp cố định (fixed-rate scheduler), và mọi quyết định cuối (FCW/AEB) đều truy vết ngược được về sensor gốc. Đây chính là điều interviewer ADAS muốn nghe: *"bạn có hiểu data flow từ raw sensor đến actuation command không, hay chỉ biết code 1 thuật toán rời rạc?"*

---

## 1. Vì sao kiến trúc này đúng?

Pipeline ADAS L2 thật (rút gọn, giống Mobileye/Bosch/Continental cấp khái niệm) luôn có 5 khối không thể thiếu:

```
Sensor → Perception (detect) → Fusion/Tracking → Threat Assessment → Decision/Actuation
```

**TrackGuard clone đúng 5 khối này**, thêm 2 khối hạ tầng bắt buộc mà nhiều project demo hay bỏ qua (và chính 2 khối này mới chứng minh "hiểu nghề", không phải chỉ "biết thuật toán"):

```
+ Time Sync & Ego-motion Compensation   (khối hay bị bỏ qua #1)
+ Real-time deterministic engine         (khối hay bị bỏ qua #2)
```

→ Interviewer hỏi "camera 30Hz, radar 20Hz, đồng bộ sao?" hoặc "pipeline chạy đúng nhịp real-time như nào?" — TrackGuard có câu trả lời bằng code thật, không phải "em nghĩ là...".

---

## 2. Full pipeline diagram (12 Boundaries)

```
┌──────────────────────────────────────────────────────────────────────────┐
│ STAGE -1  CONFIG & DETERMINISM MANAGER                                    │
│ ┌────────────────────────────────────────────────────────────────────┐   │
│ │ RunConfig (YAML) → seeded RNG (std::mt19937_64, seed ghi log)         │   │
│ │ ConfigHash (SHA-256 của config file) → in vào mọi report/metrics      │   │
│ │ → bằng chứng "kết quả này tứng ứng với config này", reproducible     │   │
│ └────────────────────────────────────────────────────────────────────┘   │
│                                                                          │
│ ═══ Boundary Z: CONFIG/DETERMINISM (seed → toàn bộ pipeline) ═════════   │
│                                                                          │
│ STAGE 0  SCENARIO / DATASET REPLAY HARNESS                                │
│ ┌────────────────────────────────────────────────────────────────────┐   │
│ │ KittiTrackingLoader (parses KITTI tracking label + oxts odometry)   │   │
│ │ ScenarioDsl (YAML: cut-in, hard-brake, pedestrian, FP-trap, x20+)     │   │
│ │ ReplayClock (virtual time, deterministic — same input = same output)│   │
│ └────────────────────────────────────────────────────────────────────┘   │
│                                                                          │
│ ═══ Boundary A: SENSOR INGEST (ISensorSource factory) ═══════════════    │
│                                                                          │
│ STAGE 1  SENSOR ADAPTERS                                                 │
│ ┌────────────────────────────────────────────────────────────────────┐   │
│ │ ISensorSource (pure virtual)                                       │   │
│ │   virtual std::vector<RawDetection> poll(Timestamp t) = 0          │   │
│ │   virtual SensorType type() const = 0                              │   │
│ │                                                                    │   │
│ │ CameraDetectionSource   → 2D bbox + class + confidence, 10Hz        │   │
│ │ RadarDetectionSource    → range, azimuth, range-rate, 20Hz          │   │
│ │ (bonus) LidarPointCloudSource → clustered point → centroid, 10Hz    │   │
│ └────────────────────────────────────────────────────────────────────┘   │
│                                                                          │
│ ═══ Boundary B: TIME SYNC + EGO-MOTION COMPENSATION ═════════════════    │
│                                                                          │
│ STAGE 2  SYNCHRONIZATION                                                 │
│ ┌────────────────────────────────────────────────────────────────────┐   │
│ │ TimeSyncBuffer                                                      │   │
│ │   buffer per-sensor detections, interpolate to common tick (50ms)  │   │
│ │ EgoMotionCompensator                                                 │   │
│ │   dead-reckoning từ odometry/IMU giữa 2 timestamp                   │   │
│ │   xoay + tịnh tiến detection cũ về frame ego hiện tại               │   │
│ └────────────────────────────────────────────────────────────────────┘   │
│                                                                          │
│ ═══ Boundary C: COORDINATE TRANSFORM + CALIBRATION ══════════════════    │
│                                                                          │
│ STAGE 3  PREPROCESSING                                                   │
│ ┌────────────────────────────────────────────────────────────────────┐   │
│ │ ExtrinsicCalibration (sensor→vehicle frame, 4x4 homogeneous matrix) │   │
│ │ FovGate (loại detection ngoài FOV/range hợp lệ của sensor)          │   │
│ │ ConfidenceGate (loại detection confidence < threshold)              │   │
│ └────────────────────────────────────────────────────────────────────┘   │
│                                                                          │
│ ═══ Boundary D: STATE ESTIMATION ═════════════════════════════════════   │
│                                                                          │
│ STAGE 4  KALMAN FILTER CORE                                              │
│ ┌────────────────────────────────────────────────────────────────────┐   │
│ │ KalmanFilterCV   (constant velocity — state [x,y,vx,vy])            │   │
│ │ ExtendedKalmanFilterCTRV (constant turn-rate & velocity —           │   │
│ │                            state [x,y,v,yaw,yaw_rate])              │   │
│ │ (stretch) UnscentedKalmanFilterCTRV — sigma-point, no Jacobian      │   │
│ │ (stretch) ImmEstimator — blend CV+CTRV theo model probability        │   │
│ │   predict(dt) → x' = f(x), P' = F P Fᵀ + Q                          │   │
│ │   update(z)   → y = z - h(x); K = P Hᵀ S⁻¹; x += Ky; Joseph form P  │   │
│ │   assert: P symmetric + positive semi-definite (debug build)        │   │
│ └────────────────────────────────────────────────────────────────────┘   │
│                                                                          │
│ ═══ Boundary E: DATA ASSOCIATION ═════════════════════════════════════   │
│                                                                          │
│ STAGE 5  ASSOCIATION                                                     │
│ ┌────────────────────────────────────────────────────────────────────┐   │
│ │ MahalanobisGate  (chi-square test, loại cặp track-detection quá xa)│   │
│ │ HungarianAssigner (O(n³) optimal assignment trên cost matrix)       │   │
│ │   cost(i,j) = Mahalanobis(track_i, det_j)  hoặc  1 - IoU(i,j)       │   │
│ └────────────────────────────────────────────────────────────────────┘   │
│                                                                          │
│ ═══ Boundary F: TRACK LIFECYCLE MANAGEMENT ═══════════════════════════   │
│                                                                          │
│ STAGE 6  TRACK MANAGER                                                  │
│ ┌────────────────────────────────────────────────────────────────────┐   │
│ │ TrackManager                                                        │   │
│ │   state machine: Tentative → Confirmed → Coasting → Deleted        │   │
│ │   matched N frame liên tiếp → Confirmed                            │   │
│ │   unmatched → Coasting (predict-only, tối đa K frame)               │   │
│ │   unmatched quá K frame hoặc Tentative unmatched → Deleted          │   │
│ │   track_id ổn định xuyên suốt occlusion ngắn                       │   │
│ └────────────────────────────────────────────────────────────────────┘   │
│                                                                          │
│ ═══ Boundary G: MULTI-SENSOR FUSION ══════════════════════════════════   │
│                                                                          │
│ STAGE 7  TRACK FUSER (track-to-track / late fusion)                      │
│ ┌────────────────────────────────────────────────────────────────────┐   │
│ │ TrackFuser                                                          │   │
│ │   match camera-track ↔ radar-track cùng vùng (gating tương tự E)   │   │
│ │   fuse state: covariance intersection (confidence-weighted)        │   │
│ │   fallback: track chỉ có 1 sensor vẫn output (không bắt buộc 2/2)  │   │
│ └────────────────────────────────────────────────────────────────────┘   │
│                                                                          │
│ ═══ Boundary H: THREAT ASSESSMENT ════════════════════════════════════   │
│                                                                          │
│ STAGE 8  TTC ESTIMATOR                                                   │
│ ┌────────────────────────────────────────────────────────────────────┐   │
│ │ TTCEstimator                                                        │   │
│ │   closing_speed = -(d(range)/dt) chiếu lên trục dọc ego            │   │
│ │   TTC = range / closing_speed  (closing_speed > 0, else ∞)          │   │
│ │   PathPredictor: dự đoán quỹ đạo ego (constant curvature) để lọc    │   │
│ │                  object không cùng làn (giảm false positive)        │   │
│ └────────────────────────────────────────────────────────────────────┘   │
│                                                                          │
│ ═══ Boundary I: DECISION LAYER ═══════════════════════════════════════   │
│                                                                          │
│ STAGE 9  FCW + AEB                                                       │
│ ┌────────────────────────────────────────────────────────────────────┐   │
│ │ FcwDecision (hysteresis state machine: Idle→Warn→Idle)              │   │
│ │   TTC < T_warn (vd 2.6s) N frame liên tiếp → Warn                   │   │
│ │   TTC > T_warn + margin → về Idle (chống chattering)                │   │
│ │ AebDecision (staged braking state machine)                          │   │
│ │   Stage1 (partial brake) TTC < 1.6s · Stage2 (full brake) TTC<0.9s  │   │
│ │   Suppression: driver đang phanh/đánh lái mạnh → suppress           │   │
│ └────────────────────────────────────────────────────────────────────┘   │
│                                                                          │
│ ═══ Boundary J: REAL-TIME ENGINE ═════════════════════════════════════   │
│                                                                          │
│ [Engine]  FixedRateScheduler (20Hz, budget 50ms/cycle) +                 │
│           bounded lock-free SPSC queue giữa mỗi stage (reuse pattern     │
│           EventStreamCore) + cycle-time profiling (p50/p95/p99)          │
│                                                                          │
│ ═══ Boundary K: VALIDATION & METRICS ═════════════════════════════════   │
│                                                                          │
│ [QA]      ScenarioRunner (20+ case) + MOTA/MOTP calculator +             │
│           FCW/AEB precision-recall + gtest (KF/EKF/UKF/association/TTC/  │
│           decision) + ASan/UBSan/TSan + CI matrix                        │
│                                                                          │
│ ═══ Boundary L: PORTABILITY & INTEROP (stretch) ══════════════════════   │
│                                                                          │
│ [Bonus]   ARM64 cross-compile + QEMU CI (giống AutoDiag P19) +          │
│           ROS2 publisher wrapper tối giản, config-gated (mặc định tắt)  │
└──────────────────────────────────────────────────────────────────────────┘
```

---

## 3. Vì sao chọn Track-level (Late) Fusion, không phải Early Fusion

| Tiêu chí | Early/Low-level Fusion (raw data) | **Track-level/Late Fusion (đã chọn)** |
|---|---|---|
| Độ phức tạp triển khai | Rất cao (cần calibrate pixel-level, đồng bộ raw signal) | Vừa phải — mỗi sensor track độc lập trước, fuse sau |
| Khả năng thêm sensor mới | Khó (phải sửa core fusion algorithm) | Dễ — sensor mới chỉ cần track riêng rồi cắm vào `TrackFuser` |
| Chịu lỗi 1 sensor | Toàn hệ thống ảnh hưởng | Sensor hỏng → fallback dùng track sensor còn lại (graceful degrade) |
| Độ chính xác lý thuyết tối đa | Cao hơn (không mất thông tin sớm) | Thấp hơn 1 chút — chấp nhận trade-off vì lý do maintainability |
| Phù hợp cho portfolio 60 ngày | Không khả thi (cần raw sensor model phức tạp, dễ lạc đề ML) | **Khả thi, đúng trọng tâm C++/thuật toán fusion-tracking-decision** |

> **Interview line:** *"Tôi chọn late fusion có chủ đích — nó đúng với constraint thời gian, dễ chứng minh đúng bằng test, và là pattern thực tế nhiều hệ thống production dùng (mỗi sensor có track riêng, fusion center chỉ hợp nhất track). Tôi hiểu rõ trade-off với early fusion và có thể thảo luận sâu nếu được hỏi."*

---

## 4. Safety & Quality Architecture (không chỉ thuật toán đúng — còn phải truy vết được và static-analysis sạch)

Ba lớp “quality” phủ xuyên suốt 12 boundary trên, không phải 1 boundary riêng mà là constraint áp dụng cho mọi boundary:

```
┌─────────────────────────────────────────────────────────────────────┐
│  LỚP 1 — REQUIREMENTS TRACEABILITY (xem 06_REQUIREMENTS_TRACEABILITY.md)   │
│  Mỗi FR/NFR có ID → trace tới class/hàm cụ thể (@satisfies) → trace   │
│  tới test case cụ thể (@verifies). Ví dụ: FR-013 (Joseph form) →      │
│  estimation::KalmanFilterCV::update() → TC-KF-003.                    │
├─────────────────────────────────────────────────────────────────────┤
│  LỚP 2 — SAFETY CONCEPT (xem 07_SAFETY_HAZARD_ANALYSIS.md)            │
│  Hazard (H1-H7) → Safety Goal (SG-01..05) → Technical Safety          │
│  Requirement (TSR-xx) → cơ chế kiến trúc cụ thể trong Boundary G/I.   │
│  Ví dụ: SG-02 (AEB stage2 không dựa 1 sensor) → TrackFuser phải expose│
│  contributing_sensors → AebDecision kiểm tra trước khi leo stage.     │
├─────────────────────────────────────────────────────────────────────┤
│  LỚP 3 — CODING STANDARD & STATIC ANALYSIS (xem 08_CODING_STANDARDS_ │
│  STATIC_ANALYSIS.md) — 24 rule TGCS-xx (MISRA C++/AUTOSAR-inspired)   │
│  enforce bằng clang-tidy + cppcheck + Coverity như required CI gate,  │
│  song song với ASan/UBSan/TSan (Boundary K).                          │
└─────────────────────────────────────────────────────────────────────┘
```

> **Vì sao vẽ riêng thành 1 section?** Vì đây chính là phần một project ADAS "đẳng cấp CV" cần có mà hầu hết portfolio chỉ dừng ở thuật toán bỏ qua: khả năng trả lời *"làm sao biết feature X đã test đủ?"* (→ 06), *"tại sao AEB thiết kế thế này chứ không phải khác?"* (→ 07), và *"code có tuân chuẩn automotive nào không hay chỉ 'chạy được'?"* (→ 08) — cả ba đều có tài liệu cụ thể, không phải lời hứa suông.

---

## 5. Data model xuyên suốt pipeline

```cpp
struct RunConfig {                // Stage -1 — nạp từ YAML, hash SHA-256 ghi vào mọi report
    uint64_t    rng_seed;
    MotionModel default_model;     // CV | CTRV | UKF_CTRV | IMM (stretch)
    float       fcw_ttc_threshold_s;
    float       aeb_stage1_ttc_s;
    float       aeb_stage2_ttc_s;
    bool        enable_ros2_publisher = false;   // stretch, off by default
};

struct RawDetection {
    Timestamp   t;
    SensorType  sensor;      // Camera | Radar | Lidar
    Vec2        pos;         // trong sensor frame
    Vec2        vel;         // chỉ Radar có range-rate trực tiếp
    float       confidence;
  };

struct TransformedDetection {     // sau Stage 3 — đã ở vehicle frame
    Timestamp t; SensorType sensor; Vec2 pos_vehicle; Vec2 vel_vehicle; float confidence;
};

enum class MotionModel { CV, CTRV, UKF_CTRV /*stretch*/, IMM /*stretch*/ };

struct Track {
    TrackId        id;
    TrackState     state;        // Tentative | Confirmed | Coasting | Deleted
    MotionModel    model;        // CV | CTRV | UKF_CTRV | IMM
    Eigen::VectorXd x;           // state vector
    Eigen::MatrixXd P;           // covariance
    int            hit_streak;   // matched liên tiếp
    int            miss_streak;  // unmatched liên tiếp
    std::vector<SensorType> contributing_sensors;
    // IMM-only (stretch): model_probs[CV]=0.7, model_probs[CTRV]=0.3, ...
    std::optional<std::unordered_map<MotionModel,float>> model_probs;
};

struct FusedTrack : Track { float fusion_confidence; };

struct ThreatAssessment { TrackId track_id; float ttc; float closing_speed; bool in_ego_path; };

struct DecisionOutput { FcwState fcw; AebStage aeb; TrackId primary_threat; std::string reason; };
```

→ Mỗi stage chỉ phụ thuộc struct của stage liền trước — giống nguyên tắc Open-Closed của `IDiagnosticHal` bên AutoDiag, ở đây là **pipeline stage thay được độc lập** (vd thay `HungarianAssigner` bằng `GreedyAssigner` không ảnh hưởng `TrackManager`, hoặc thay `MotionModel::CTRV` bằng `UKF_CTRV` không ảnh hưởng `TrackFuser`/`TTCEstimator` vì cả hai đều expose cùng interface `x`/`P`).

---

## 6. Thư mục source đề xuất

```
TrackGuard/
├── README.md
├── CMakeLists.txt
├── .clang-tidy          (TGCS coding standard checks — xem 08)
├── .github/
│   ├── workflows/       (ci.yml matrix gcc/clang, ci-arm64.yml QEMU — stretch)
│   └── PULL_REQUEST_TEMPLATE.md  (checklist review — xem 08 §4)
├── config/              (default.yaml, scenario-tuning.yaml — RunConfig defaults)
├── requirements/        (traceability.csv làm việc — xem 06_REQUIREMENTS_TRACEABILITY.md)
├── safety/              (working notes HARA — xem 07_SAFETY_HAZARD_ANALYSIS.md)
├── include/trackguard/
│   ├── config/          (RunConfig loader, ConfigHash, seeded RNG wrapper)
│   ├── sensors/        (ISensorSource, CameraDetectionSource, RadarDetectionSource)
│   ├── sync/            (TimeSyncBuffer, EgoMotionCompensator)
│   ├── transform/       (ExtrinsicCalibration, FovGate, ConfidenceGate)
│   ├── estimation/      (KalmanFilterCV, ExtendedKalmanFilterCTRV, MotionModel)
│   ├── estimation_stretch/ (UnscentedKalmanFilterCTRV, ImmEstimator — stretch only)
│   ├── association/     (MahalanobisGate, HungarianAssigner, IoUGate)
│   ├── tracking/        (Track, TrackManager)
│   ├── fusion/          (TrackFuser)
│   ├── threat/          (TTCEstimator, PathPredictor)
│   ├── decision/        (FcwDecision, AebDecision)
│   ├── engine/          (FixedRateScheduler, SpscQueue, Profiler)
│   ├── replay/          (KittiTrackingLoader, ScenarioDsl, ReplayClock)
│   └── interop/         (Ros2Publisher wrapper — stretch, config-gated)
├── src/                 (implementation .cpp tương ứng)
├── viz/                 (2D top-down renderer — SDL2/OpenCV, NICE)
├── metrics/             (MOTA/MOTP + precision-recall — C++ hoặc Python post-process)
├── scenarios/           (YAML scenario library: 20+ case, tuning set + holdout set)
├── data/kitti_subset/   (2-3 sequence nhỏ, gitignored nếu lớn — script download riêng)
├── tests/               (gtest: unit + scenario integration, 150+ test)
├── tools/               (trace_check.py — requirements traceability scanner, xem 06/08)
├── docs/                (CODING_STANDARD.md chi tiết, lessons.md)
├── toolchains/          (arm64-toolchain.cmake — stretch cross-compile)
└── CHANGELOG.md         (semver theo Conventional Commits — xem 08 §6)
```
