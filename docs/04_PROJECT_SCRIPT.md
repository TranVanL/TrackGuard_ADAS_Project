# 🎬 TrackGuard — Project Script (Pitch / Demo / Positioning / FAQ)

> File này dùng để **nói về project** — trước phỏng vấn, khi quay demo video, hoặc khi viết CV/LinkedIn. Đọc to trước khi dùng thật, chỉnh giọng văn theo phong cách cá nhân.
>
> 📚 Muốn đi sâu Q&A theo từng chủ đề (150+ câu, có Short/Deep dive/Trap)? Xem [05_INTERVIEW_PREP.md](05_INTERVIEW_PREP.md).

---

## 1. Elevator Pitch (10-15 giây — mở đầu câu chuyện)

> *"TrackGuard là một ADAS perception-to-decision pipeline viết bằng C++17 — tôi tự xây từ config/determinism boundary, sensor replay, qua Kalman/EKF (và stretch UKF/IMM) state estimation, Hungarian data association, multi-object tracking, đến multi-sensor fusion và cuối cùng ra quyết định FCW/AEB dựa trên TTC. Mục tiêu là chứng minh tôi hiểu ADAS software end-to-end qua 12 boundary rõ ràng, không chỉ 1 thuật toán rời rạc."*

---

## 2. Elevator Pitch mở rộng (30-45 giây — khi được hỏi "kể về project của bạn")

> *"Tôi build TrackGuard trong 60 ngày, mỗi tối 2-3 tiếng, với tuần stretch thứ 13 optional. Toàn bộ config chạy qua 1 `RunConfig` + seeded RNG để đảm bảo determinism — mọi report đều kèm ConfigHash để trace lại đúng config nào tạo ra kết quả nào. Ý tưởng là mô phỏng đúng data flow thật của 1 hệ thống ADAS cấp L2: camera và radar (tôi dùng dữ liệu replay từ KITTI tracking dataset và scenario tự sinh) được đồng bộ thời gian, bù chuyển động ego, rồi đưa vào Kalman Filter — tôi dùng cả model constant-velocity và constant-turn-rate cho object đi thẳng và rẽ cua, cộng thêm UKF và IMM ở stretch week để chứng minh hiểu sâu hơn về non-linear estimation. Sau đó tôi làm data association bằng Hungarian algorithm với gating Mahalanobis, quản lý vòng đời track qua occlusion, rồi hợp nhất track từ 2 sensor bằng covariance intersection. Cuối cùng là tính TTC và ra quyết định FCW/AEB với hysteresis chống dao động và suppression khi tài xế đã phản ứng. Tôi validate toàn bộ bằng MOTA/MOTP và precision-recall trên 20+ scenario — chia rõ tuning set/holdout set để tránh overfit ngưỡng — chạy trên 1 real-time engine nhịp cố định 20Hz với lock-free queue, và CI matrix full sanitizer (ASan/UBSan/TSan)."*

---

## 3. Positioning trong bộ 3 project (khi được hỏi "bạn có mấy project?")

> *"Tôi có 3 project bổ trợ nhau, không đua nhau: **EventStreamCore** chứng minh C++ real-time throughput — hàng triệu event/s, lock-free, sub-microsecond latency. **AutoDiag** chứng minh Android system engineering — clone kiến trúc AAOS CarService, HAL abstraction, JNI, bring-up artifacts. **TrackGuard** chứng minh domain depth ADAS — từ sensor đến fusion, tracking, và quyết định an toàn. Với JD ADAS/Sensor Fusion, tôi dẫn bằng TrackGuard. Với JD Android Automotive, tôi dẫn bằng AutoDiag. Với JD Embedded/Real-time C++ thuần, tôi dẫn bằng EventStreamCore hoặc kết hợp TrackGuard's real-time engine."*

---

## 4. Demo Video Script (90-120 giây)

**[0:00-0:15] Mở đầu — vấn đề**
> "Đây là TrackGuard — một sensor fusion và collision decision pipeline. Tôi sẽ replay 1 scenario: xe phía trước phanh gấp."

**[0:15-0:45] Visualization chạy — giải thích từng lớp**
> "Các chấm nhạt là detection thô từ camera (xanh) và radar (đỏ). Hình chữ nhật có ID là track đã qua Kalman Filter và fusion. Chú ý track vẫn giữ ID ổn định dù có nhiễu đo lường."

**[0:45-1:15] Threat assessment & decision**
> "Khi TTC giảm dưới ngưỡng, đèn cảnh báo FCW bật (màu vàng). Khi TTC tiếp tục giảm, hệ thống vào AEB Stage 1 — phanh một phần (màu cam), rồi Stage 2 — phanh toàn lực (màu đỏ) nếu vẫn nguy hiểm."

**[1:15-1:30] Kết — số liệu**
> "Toàn bộ pipeline chạy nhịp cố định 20Hz, đã validate trên 20+ scenario (14 tuning + 6 holdout) với MOTA/MOTP và precision-recall report đầy đủ trong repo, CI xanh trên cả ASan/UBSan/TSan."

---

## 5. Technical Deep-Dive Script (3-5 phút — khi interviewer muốn nghe sâu)

Thứ tự kể chuyện theo đúng 12 boundary của data flow (Z→L), dễ nhớ nhất:

0. **Config/Determinism (Z)** — "Trước khi chạy gì cả, tôi load `RunConfig` từ YAML và seed 1 `std::mt19937_64` — mọi randomness trong pipeline (noise injection, v.v.) đều đi qua RNG này, không dùng `rand()` hay `std::random_device`. Mỗi report kèm SHA-256 `ConfigHash` để trace ngược lại đúng config nào tạo ra kết quả đó — đây là điều kiện tiên quyết để mọi test scenario reproducible."
1. **Sensor & Replay (A)** — "Tôi replay từ KITTI + tự sinh synthetic radar detection từ ground-truth cộng noise Gaussian, seed cố định để deterministic."
2. **Sync & Ego-motion (B)** — "Camera 10Hz, radar 20Hz lệch pha — tôi dùng interpolation buffer đưa về tick chung, và dead-reckoning bù chuyển động ego giữa các frame."
3. **Coordinate Transform/Calibration (C)** — "Extrinsic calibration đưa detection về vehicle frame chung, cộng FOV gate và confidence gate để loại detection vô lý trước khi vào estimation."
4. **State Estimation (D)** — "Tôi dùng Kalman Filter tuyến tính cho constant-velocity, và Extended Kalman Filter cho constant-turn-rate khi object rẽ cua — có Jacobian tuyến tính hoá, xử lý riêng case yaw-rate gần 0, và dùng Joseph form để giữ covariance luôn symmetric/positive-semidefinite. Ở stretch week tôi làm thêm UKF (sigma-point/unscented transform) và IMM (blend model probability CV+CTRV) để chứng minh hiểu rõ khi nào EKF không đủ."
5. **Data Association (E)** — "Tôi dùng Hungarian algorithm O(n³) với gating Mahalanobis theo chi-square threshold để loại cặp vô lý trước khi assignment."
6. **Track Lifecycle (F)** — "State machine Tentative/Confirmed/Coasting/Deleted xử lý occlusion mà không làm mất ID track."
7. **Multi-Sensor Fusion (G)** — "Track từ camera và radar được hợp nhất bằng Covariance Intersection — tôi chọn late fusion có chủ đích vì dễ mở rộng thêm sensor và chịu lỗi tốt hơn early fusion."
8. **Threat Assessment (H)** — "TTC tính từ closing speed tương đối, có `PathPredictor` lọc lane để chỉ tính TTC với object thực sự trên đường đi."
9. **Decision Layer (I)** — "FCW có hysteresis chống chattering (on/off threshold khác nhau + debounce), AEB có staged braking và suppression khi tài xế đã phản ứng — mọi quyết định đều có `reason` string để trace lại."
10. **Real-time Engine (J)** — "Toàn bộ chạy trên fixed-rate scheduler 20Hz dùng `steady_clock`, với lock-free SPSC queue (cùng DNA với EventStreamCore) đẩy metrics/log ra thread riêng, không block real-time thread."
11. **Validation/Metrics (K)** — "Tôi đo p50/p95/p99 cycle time, validate bằng MOTA/MOTP và precision-recall trên 20+ scenario chia tuning/holdout, CI matrix gcc/clang × Debug(ASan/UBSan/TSan)/Release."
12. **Portability/Interop (L, stretch)** — "Nếu còn thời gian, tôi cross-compile ARM64 qua QEMU và thêm 1 ROS2 publisher wrapper optional, để chứng minh core logic không phụ thuộc platform hay middleware cụ thể."

---

## 6. Honesty Framing (nói CHỦ ĐỘNG, không đợi bị hỏi)

> *"Vài điều tôi muốn nói rõ ngay: TrackGuard dùng classical pipeline (Kalman Filter, Hungarian, rule-based decision) — không dùng deep learning cho detection/tracking, vì mục tiêu là chứng minh hiểu toán nền tảng của estimation và combinatorial optimization, chính nền tảng này cần thiết để hiểu cả DL-based tracker sau này. Tôi dùng dataset KITTI và scenario tự sinh, không có sensor thật hay xe thật — radar là synthetic, sinh từ ground-truth cộng noise. Ngưỡng FCW/AEB tôi chọn qua tuning set/holdout set discipline, không phải số chính thức NCAP/ISO 26262 của 1 hãng cụ thể, và hệ thống này không phải certified — tôi hiểu khái niệm và thiết kế theo tinh thần đó ở mức kiến trúc, không claim hơn thế. Cụ thể, tôi có viết 1 HARA-style hazard analysis và requirements traceability matrix ([07](07_SAFETY_HAZARD_ANALYSIS.md), [06](06_REQUIREMENTS_TRACEABILITY.md)) để luyện đúng tư duy quy trình — và tôi nói rõ đó không phải safety case được chứng nhận. Và UKF/IMM/ARM64+QEMU/ROS2 wrapper là stretch goal tuần 13 — tôi sẽ nói rõ nếu phần nào chưa kịp hoàn thành khi phỏng vấn."*

> 📚 Xem thêm chi tiết Q&A theo từng chủ đề honesty framing tại [05a](05a_INTERVIEW_FOUNDATION.md), [05c §C3-C4](05c_INTERVIEW_FUSION_DECISION.md), và [05e §E3](05e_INTERVIEW_BEHAVIORAL.md).

---

## 7. FAQ / Câu hỏi khó dự đoán trước

| Câu hỏi | Trả lời gợi ý |
|---|---|
| "Sao không dùng deep learning tracker như DeepSORT/ByteTrack?" | "DeepSORT/ByteTrack vẫn dùng Kalman Filter + Hungarian bên trong, chỉ thêm CNN embedding cho appearance feature. Tôi chọn xây nền tảng classical trước, đó là phần khó và dễ bị bỏ qua nếu chỉ gọi thư viện." |
| "Radar của bạn là thật hay giả lập?" | "Giả lập từ ground-truth + noise model thực tế (Gaussian theo đặc tính sai số thật của radar) — tôi nói rõ điều này trong docs, đây là kỹ thuật sensor model synthesis chuẩn khi test ADAS không có hardware thật." |
| "Early fusion có tốt hơn không?" | "Về lý thuyết đúng, giữ nhiều thông tin hơn — nhưng đòi hỏi đồng bộ raw signal rất chính xác và khó mở rộng thêm sensor. Tôi chọn late fusion có chủ đích, hiểu rõ trade-off, và có thể thảo luận sâu hơn nếu cần." |
| "Hệ thống này có thể chạy trên ECU thật không?" | "Kiến trúc thuần C++17/CMake, không phụ thuộc OS-specific API, nên về nguyên tắc portable. Tôi chưa cross-compile/test trên ARM thật (khác với AutoDiag đã làm ARM64+QEMU), nhưng code không có gì chặn việc đó." |
| "MOTA/MOTP của bạn bao nhiêu?" | Trả lời bằng SỐ THẬT lấy từ report cuối P12/K — không làm tròn đẹp giả tạo, nói rõ số trên tuning set VÀ trên holdout set riêng biệt, và giải thích context (dataset nhỏ, scenario tự thiết kế, không so sánh trực tiếp được với benchmark public lớn). |
| "Tại sao không dùng ROS2?" | "Tôi có thêm 1 ROS2 publisher wrapper ở stretch week, config-gated tắt mặc định — nhưng cố tình giữ core logic độc lập framework để dễ test/port, tương tự cách `hal/` bên AutoDiag build được không cần Android runtime." |
| "Sao lại thêm UKF/IMM nếu EKF đã đủ dùng?" | "EKF (CTRV) đã đủ cho hầu hết scenario L2 ADAS — UKF/IMM là stretch để chứng minh tôi hiểu trade-off (sigma-point/unscented transform tránh linearization error, model-probability blending cho xe đổi giữa đi thẳng/rẽ cua), không phải vì EKF không đủ." |
| "Có test race condition trên lock-free queue không?" | "Có — CI chạy ThreadSanitizer (TSan) riêng trên build Debug đúng để bắt data race trên SPSC ring buffer, cùng pattern lock-free tôi dùng ở EventStreamCore." |
| "Làm sao bạn biết 1 feature đã được test đầy đủ, không phải chỉ 'chạy thử thấy được'?" | "Tôi có 1 requirements traceability matrix ([06](06_REQUIREMENTS_TRACEABILITY.md)) — mỗi FR/NFR có ID, trace tới class/hàm hiện thực (`@satisfies`) và tới test case cụ thể (`@verifies`). Nếu 1 requirement Must không có test nào verify, đó là gap phải đóng trước khi tag release." |
| "Dựa vào đâu bạn thiết kế AEB stage2 cần cross-check 2 sensor?" | "Từ 1 HARA-style hazard analysis tôi tự viết ([07](07_SAFETY_HAZARD_ANALYSIS.md)) — hazard 'AEB kích hoạt sai do ghost track 1 sensor' được phân tích ra Safety Goal SG-02, rồi trace xuống technical requirement cụ thể trong kiến trúc — không phải quyết định ngẫu nhiên. Tôi luôn nói rõ đây là tài liệu tham khảo/portfolio, không phải safety case certified." |
| "Code của bạn có tuân chuẩn coding standard nào không?" | "Có — tôi tự đặt 1 subset 24 rule lấy cảm hứng từ MISRA C++/AUTOSAR C++14 ([08](08_CODING_STANDARDS_STATIC_ANALYSIS.md)), enforce bằng clang-tidy/cppcheck/Coverity như required CI check, không phải chỉ tự giác review tay." |

---

## 8. Câu mở đầu theo từng loại JD (chọn nhanh khi biết JD trước phỏng vấn)

| JD type | Opening line |
|---|---|
| **ADAS Software Engineer** | *"TrackGuard là 1 pipeline ADAS đầy đủ tôi tự viết — sensor đến fusion, tracking, và quyết định FCW/AEB, bằng C++17 thuần."* |
| **Sensor Fusion Engineer** | *"Trọng tâm TrackGuard là track-to-track fusion bằng Covariance Intersection giữa camera và radar, tôi có thể đi sâu vào lý do chọn late fusion và cách tôi validate bằng RMSE so sánh."* |
| **Perception Engineer (classical)** | *"Tôi tập trung vào phần perception classical — Kalman/EKF estimation và Hungarian-based multi-object tracking, với MOTA/MOTP validation."* |
| **Autonomous Driving C++ Engineer** | *"TrackGuard chạy trên 1 real-time engine nhịp cố định 20Hz với lock-free queue — cùng tinh thần real-time với EventStreamCore của tôi, áp dụng vào domain ADAS."* |
| **Functional Safety adjacent** | *"Tôi thiết kế decision layer (FCW/AEB) theo tinh thần ISO 26262 ở mức kiến trúc — hysteresis, staged response, suppression rules, traceable `reason` string — và validate bằng scenario library (tuning/holdout split) với precision-recall report."* |
| **Test/Validation Engineer** | *"Tôi có testing pyramid đầy đủ — unit closed-form, integration stage-by-stage, 20+ scenario end-to-end chia tuning/holdout — cộng CI matrix gcc/clang × Debug(ASan/UBSan/TSan)/Release, và tôi có thể nói sâu về methodology chống overfit threshold."* |
| **Robotics/ROS generalist** | *"Core pipeline độc lập middleware, nhưng tôi có thêm 1 ROS2 publisher wrapper config-gated ở stretch week để chứng minh interop dễ dàng với ecosystem robotics chuẩn."* |

---

## 9. Kill-shot numbers (thuộc lòng, đọc vanh vách không cần note)

> ~9,850 LOC C++17 core (~11,500 với stretch) · 150+ test (unit + integration + scenario) · 20+ scenario (14 tuning + 6+ holdout: cut-in/hard-brake/pedestrian/FP-trap/occlusion/dropout/...) · MOTA/MOTP + FCW/AEB precision-recall report thật, báo cáo riêng tuning vs holdout · CI matrix gcc/clang × Debug(ASan/UBSan/TSan)/Release + stretch ARM64+QEMU allow-failure · static analysis gate sạch (clang-tidy/cppcheck/Coverity, 24-rule coding standard) · fixed-rate 20Hz, lock-free SPSC queue, p50/p95/p99 profiling · Config/Determinism boundary — seeded RNG + SHA-256 ConfigHash trên mọi report · requirements traceability matrix (FR/NFR → design → test) · HARA-style hazard analysis + safety goals cho FCW/AEB (portfolio-quality, không certified) · 12 pipeline boundary (Z→L) · 60 commit core + stretch week 13, tag `w01-done`→`w12-done` + `v1.0` (+ `v1.1` nếu stretch xong) · 150+ Q&A interview-prep suite ([05_INTERVIEW_PREP.md](05_INTERVIEW_PREP.md)).
