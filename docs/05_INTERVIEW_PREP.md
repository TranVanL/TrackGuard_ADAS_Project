# 🎤 TrackGuard — Interview Prep HUB (150+ Q&A, split by topic)

> **Đây là file trung tâm.** Nội dung Q&A chi tiết được tách thành 5 file con theo chủ đề để dễ ôn theo từng buổi. Mỗi câu có **short answer** (nói 30-60s), **deep dive** (khi bị đào "why/how"), và **trap/follow-up** (câu bẫy thường theo sau). Mục tiêu: defend được TrackGuard ở mức **Senior C++/ADAS/Perception** không thua kém AutoDiag hay EventStreamCore.
>
> Toàn bộ Q&A gắn với **pipeline 12-boundary thật của TrackGuard** — đọc to trước phỏng vấn, đừng học vẹt.

---

## 📚 Cấu trúc 5 file con

| File | Chủ đề | Nội dung | # Q |
|---|---|---|---|
| **[05a_INTERVIEW_FOUNDATION.md](05a_INTERVIEW_FOUNDATION.md)** | Foundation | Kiến trúc 12-boundary · Sensor model/synthetic radar · Replay/determinism · Time sync/ego-motion · Coordinate transform/calibration · Requirements traceability & safety case (HARA) | 26 |
| **[05b_INTERVIEW_ESTIMATION_TRACKING.md](05b_INTERVIEW_ESTIMATION_TRACKING.md)** | Estimation/Tracking | Kalman Filter (CV) · EKF (CTRV) · UKF/IMM (stretch) · Hungarian/gating · Track lifecycle · MOTA/MOTP | 38 |
| **[05c_INTERVIEW_FUSION_DECISION.md](05c_INTERVIEW_FUSION_DECISION.md)** | Fusion/Decision | Early vs late fusion · Covariance Intersection · TTC · FCW hysteresis · AEB staged logic + suppression · ISO 26262/NCAP honesty | 25 |
| **[05d_INTERVIEW_SYSTEM_VALIDATION.md](05d_INTERVIEW_SYSTEM_VALIDATION.md)** | System/Validation | Fixed-rate scheduler · Lock-free SPSC queue · Profiling p50/p95/p99 · Testing pyramid · Scenario/tuning-holdout · CI/sanitizers · Coding standard & static analysis · ARM64/ROS2 (stretch) | 34 |
| **[05e_INTERVIEW_BEHAVIORAL.md](05e_INTERVIEW_BEHAVIORAL.md)** | Behavioral | Level defense · System design whiteboard · Honesty framing · STAR stories · Demo script · Curveballs · Reverse questions | 28 |

> **Tổng: 151 Q&A** trên 25 section. Ôn theo thứ tự A → B → C → D → E, hoặc theo JD type (xem bảng dưới).

---

## 🗺 Lộ trình ôn theo buổi (7 ngày trước phỏng vấn)

| Ngày | File | Trọng tâm | Self-check |
|---|---|---|---|
| **D-7** | 05a §A1–A2 | 12-boundary pipeline + sensor/synthetic radar model | Vẽ pipeline từ trí nhớ không nhìn note |
| **D-6** | 05a §A3–A4 | Determinism/seeded RNG + time sync/ego-motion + calibration | Giải thích vì sao seed quan trọng cho scenario test |
| **D-5** | 05b §B1–B3 | KF (CV) predict/update + Joseph form + EKF (CTRV) + Jacobian | Viết tay công thức predict/update trên giấy |
| **D-4** | 05b §B4–B6 | UKF/IMM (stretch) + Hungarian + gating + track lifecycle | Giải thích sigma points không cần note |
| **D-3** | 05c §C1–C3 | Early vs late fusion + Covariance Intersection + TTC | Derive công thức TTC từ kinematics cơ bản |
| **D-2** | 05c §C4 + 05d §D1–D2 | FCW/AEB decision + real-time engine + lock-free queue | Vẽ state machine FCW/AEB từ trí nhớ |
| **D-1** | 05d §D3–D4 + 05e toàn bộ | Testing/CI + Level defense + STAR + demo + curveballs | Diễn thử demo 2 phút + 3 câu STAR |

---

## 🎯 Multi-JD framing (đọc trước khi vào phỏng vấn)

Identify JD type → chọn opening line đúng (10 giây đầu) → emphasis đúng section.

| JD type | Opening line | Section ưu tiên |
|---|---|---|
| **ADAS / Sensor Fusion Engineer** | *"I built TrackGuard — a C++17 multi-sensor fusion and tracking pipeline from raw detection to FCW/AEB decision, classical estimation end-to-end."* | 05b toàn bộ, 05c §C1–C2 |
| **Perception / Autonomous Driving C++** | *"TrackGuard clones the sensor→fusion→tracking→decision pipeline of a real L2 ADAS stack, replayed on KITTI + synthetic scenarios, fully deterministic."* | 05a §A1–A3, 05b §B5 |
| **Embedded/Real-time C++ (generic)** | *"My main real-time depth is EventStreamCore; TrackGuard adds a domain-specific fixed-rate scheduler + lock-free queue applied to ADAS."* | 05d §D1–D2 |
| **Test/Validation Engineer (ADAS)** | *"TrackGuard's ScenarioRunner + tuning/holdout split + MOTA/MOTP + precision-recall report is my proof of validation methodology, not just passing tests."* | 05d §D3 |
| **Senior generalist / robotics** | *"Three complementary projects: AutoDiag for Android system engineering, EventStreamCore for real-time throughput, TrackGuard for ADAS domain depth."* | 05e §E1–E2 |

> **Honesty rule (an toàn/certification):** *"TrackGuard's FCW/AEB thresholds are reference values inspired by NCAP/ISO 26262 concepts for a portfolio project — not certified production values. What I can defend deeply is the estimation math, the association algorithm, and the validation methodology behind them."* — chi tiết [05c §C4](05c_INTERVIEW_FUSION_DECISION.md) và [05e §E3](05e_INTERVIEW_BEHAVIORAL.md).

---

## ⚡ One-page cheat sheet (liếc 5 phút trước khi vào phòng)

**12 boundaries (thuộc lòng thứ tự):**
1. **Z** Config/Determinism → seeded RNG + ConfigHash (SHA-256)
2. **A** Sensor Ingest → `ISensorSource` factory (Camera/Radar/Lidar)
3. **B** Time Sync + Ego-motion → interpolation + dead-reckoning
4. **C** Coordinate Transform + Calibration → extrinsic matrix + FOV/confidence gate
5. **D** State Estimation → KF (CV) / EKF (CTRV) / UKF+IMM (stretch)
6. **E** Data Association → Mahalanobis gate + Hungarian O(n³)
7. **F** Track Lifecycle → Tentative→Confirmed→Coasting→Deleted
8. **G** Multi-Sensor Fusion → Covariance Intersection (late fusion)
9. **H** Threat Assessment → TTC + PathPredictor
10. **I** Decision Layer → FCW hysteresis + AEB staged/suppression
11. **J/K** Real-time Engine + Validation → fixed-rate scheduler + SPSC queue + ScenarioRunner
12. **L** Portability/Interop (stretch) → ARM64+QEMU + ROS2 wrapper

**Kill-shot numbers:** ~9.8K LOC core (~11.5K với stretch) · 150+ test · 20+ scenario (tuning+holdout split) · MOTA/MOTP + FCW/AEB precision-recall report · CI matrix gcc/clang × Debug(ASan/UBSan/TSan)/Release · 60 commit, 12 tag + `v1.0`.

**Reflex answers:**
- *Why classical (Kalman/Hungarian) instead of deep learning tracker?* → chứng minh hiểu toán nền tảng — DeepSORT vẫn dùng Kalman+Hungarian bên trong, chỉ đổi appearance embedding.
- *Why late fusion, not early?* → maintainability + graceful degrade khi 1 sensor hỏng, đúng constraint thời gian 60 ngày.
- *Why Joseph form for covariance update?* → numerically stable, giữ P symmetric+PSD ngay cả khi Kalman gain không tối ưu tuyệt đối.
- *Why hysteresis in FCW?* → chống chattering khi TTC dao động quanh threshold.
- *Why tuning/holdout split for scenarios?* → chống overfitting threshold vào đúng scenario đã viết, giống train/test split.
- *Why seeded RNG everywhere?* → determinism — scenario test phải reproducible để debug và để so sánh trước/sau khi đổi code.
- *steady_clock hay system_clock cho scheduler?* → `steady_clock` — không bị ảnh hưởng bởi NTP time-adjustment, monotonic.
- *Làm sao biết 1 feature đã test đủ?* → requirements traceability matrix (FR/NFR → design → test ID), không phải "chạy thử thấy được".
- *Vì sao AEB stage2 cần cross-check 2 sensor?* → trace từ HARA-style Safety Goal SG-02 (chống ghost-track false-positive braking), không phải quyết định ngẫu nhiên.

**Honesty boundaries (nói chủ động):** FCW/AEB threshold tham khảo NCAP/ISO 26262 concept, KHÔNG phải giá trị certified · radar là synthetic (sinh từ ground-truth + noise, không phải hardware thật) · HARA/requirements traceability là portfolio-quality artifact chứng minh hiểu quy trình, KHÔNG phải safety case được chứng nhận · UKF/IMM/ARM64/ROS2 là stretch, chỉ có nếu còn thời gian sau `v1.0` · không dùng deep learning cho detection/tracking, đây là lựa chọn có chủ đích.

---

## 🎬 Demo pitch 30 giây (memorize)

> *"TrackGuard is a 60-day C++17 ADAS portfolio: a full sensor-fusion-tracking-decision pipeline replayed on KITTI plus 20+ synthetic scenarios, fully deterministic via seeded RNG. Camera and synthetic-radar detections go through time sync and ego-motion compensation, then a Kalman/EKF estimator, Hungarian data association with Mahalanobis gating, a track lifecycle state machine, late fusion via covariance intersection, TTC estimation, and hysteresis-based FCW/staged AEB decision logic — all running on a 20Hz fixed-rate scheduler with a lock-free SPSC queue. Validated with 150+ tests, a tuning/holdout scenario split, MOTA/MOTP and precision-recall reporting, and a sanitizer-clean CI matrix."*

Demo 2-phút / script đầy đủ → [04_PROJECT_SCRIPT.md](04_PROJECT_SCRIPT.md).

---

## 🔗 Liên kết design docs (khi cần đào sâu 1 module)

| Chủ đề | Q&A | Design doc |
|---|---|---|
| Sensor model + synthetic radar | 05a §A2 | [knowledge/03a_SENSORS_PERCEPTION.md](knowledge/03a_SENSORS_PERCEPTION.md) |
| Kalman/EKF math | 05b §B1–B3 | [knowledge/03b_KALMAN_FILTER_ESTIMATION.md](knowledge/03b_KALMAN_FILTER_ESTIMATION.md) |
| UKF/IMM (stretch) | 05b §B4 | [knowledge/03h_ADVANCED_ESTIMATION_UKF_IMM.md](knowledge/03h_ADVANCED_ESTIMATION_UKF_IMM.md) |
| Hungarian/gating/lifecycle | 05b §B5–B6 | [knowledge/03c_DATA_ASSOCIATION_TRACKING.md](knowledge/03c_DATA_ASSOCIATION_TRACKING.md) |
| Fusion architecture | 05c §C1–C2 | [knowledge/03d_SENSOR_FUSION_ARCHITECTURE.md](knowledge/03d_SENSOR_FUSION_ARCHITECTURE.md) |
| TTC/FCW/AEB | 05c §C3–C4 | [knowledge/03e_TTC_FCW_AEB_DECISION.md](knowledge/03e_TTC_FCW_AEB_DECISION.md) |
| Real-time engine | 05d §D1–D2 | [knowledge/03f_REALTIME_CPP_LINUX.md](knowledge/03f_REALTIME_CPP_LINUX.md) |
| Testing/validation/CI | 05d §D3–D4 | [knowledge/03g_TESTING_VALIDATION.md](knowledge/03g_TESTING_VALIDATION.md) |
| Requirements traceability | 05a §A4 | [06_REQUIREMENTS_TRACEABILITY.md](06_REQUIREMENTS_TRACEABILITY.md) |
| HARA / safety concept (FCW/AEB) | 05a §A4, 05c §C4 | [07_SAFETY_HAZARD_ANALYSIS.md](07_SAFETY_HAZARD_ANALYSIS.md) |
| Coding standard & static analysis | 05d §D5 | [08_CODING_STANDARDS_STATIC_ANALYSIS.md](08_CODING_STANDARDS_STATIC_ANALYSIS.md) |
| Full architecture | 05a §A1 | [01_ARCHITECTURE.md](01_ARCHITECTURE.md) |
| Day-by-day plan | — | [02_PLAN_DETAILED_60DAY.md](02_PLAN_DETAILED_60DAY.md) |

---

## ✅ Cách dùng hiệu quả

1. **Đọc to** short answer đến khi thuộc — đừng đọc thầm.
2. **Chỉ bung deep dive khi bị hỏi** — tránh info-dump làm interviewer mất mạch.
3. **Chuẩn bị sẵn trap follow-up** — mỗi câu đều có phần "Trap" để không bị bắt bài.
4. **Luyện nói trade-off** ("why-not X"), không chỉ "what" — đó là tín hiệu senior.
5. **Honesty chủ động** — nói trước ranh giới (synthetic radar, NCAP tham khảo, stretch goals chưa chắc xong) để build trust, đừng chờ bị bắt.
6. **Demo phải narrate boundary** đang được cross (vd "đây là lúc Hungarian assign xong, track chuyển Tentative→Confirmed"), không phải chỉ tả UI.

> Chúc phỏng vấn tốt. Calibrated honesty > confident bluffing. 🚀
