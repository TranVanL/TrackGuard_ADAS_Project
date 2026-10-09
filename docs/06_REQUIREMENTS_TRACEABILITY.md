# 📋 TrackGuard — Requirements Specification & Traceability Matrix

> **Mục đích:** Chứng minh kỷ luật kỹ thuật phần mềm production — mỗi requirement có ID, mỗi ID trace được tới thiết kế (boundary/class) và tới bằng chứng kiểm chứng (test). Đây là điều interviewer senior/automotive hay hỏi: *"Làm sao bạn biết feature X đã được test đầy đủ, không phải chỉ 'chạy thử thấy được'?"* — câu trả lời là ma trận này.
>
> 🔗 Cross-ref: [00_MASTER_PLAN.md](00_MASTER_PLAN.md) · [01_ARCHITECTURE.md](01_ARCHITECTURE.md) · [07_SAFETY_HAZARD_ANALYSIS.md](07_SAFETY_HAZARD_ANALYSIS.md) (Safety Goals SG-xxx trace vào đây) · [08_CODING_STANDARDS_STATIC_ANALYSIS.md](08_CODING_STANDARDS_STATIC_ANALYSIS.md)

---

## 1. Quy ước ID

| Prefix | Ý nghĩa | Ví dụ |
|---|---|---|
| `FR-xxx` | Functional Requirement | `FR-014` |
| `NFR-xxx` | Non-Functional Requirement (real-time, determinism, portability...) | `NFR-003` |
| `SG-xxx` | Safety Goal (định nghĩa & chi tiết ở [07](07_SAFETY_HAZARD_ANALYSIS.md)) | `SG-02` |
| `TSR-xxx` | Technical Safety Requirement (derive từ SG, cầu nối sang FR/NFR) | `TSR-05` |
| `TC-xxx` | Test Case ID (unit/integration/scenario) | `TC-KF-007` |

**Quy tắc gắn tag trong code:** mỗi hàm/class hiện thực 1 requirement quan trọng phải có Doxygen tag `@satisfies{FR-xxx}` (hoặc `NFR-xxx`/`TSR-xxx`) trong comment ngay phía trên khai báo. Mỗi test case phải có `@verifies{FR-xxx}` tương ứng. Một script nhỏ (`tools/trace_check.py`, kế hoạch — xem [08 §6](08_CODING_STANDARDS_STATIC_ANALYSIS.md)) quét toàn bộ `include/`, `src/`, `tests/` để phát hiện: (a) requirement không có test nào `@verifies` nó, (b) code có `@satisfies` một ID không tồn tại trong bảng dưới — chạy như 1 CI check optional, fail nếu (a)/(b) xảy ra với FR/SG (không bắt buộc với toàn bộ NFR để tránh quá cứng nhắc).

---

## 2. Functional Requirements (FR)

| ID | Requirement | Boundary | Priority (MoSCoW) |
|---|---|---|---|
| FR-001 | Hệ thống PHẢI load `RunConfig` từ file YAML tại khởi động, fail-fast nếu thiếu field bắt buộc | Z | Must |
| FR-002 | Mọi nguồn ngẫu nhiên (noise injection, v.v.) PHẢI đi qua 1 `std::mt19937_64` được seed từ `RunConfig.rng_seed` | Z | Must |
| FR-003 | Mỗi report/metrics output PHẢI kèm SHA-256 `ConfigHash` của config file đã dùng | Z | Must |
| FR-004 | Hệ thống PHẢI đọc được KITTI tracking label + oxts odometry cho ít nhất 2 sequence | A | Must |
| FR-005 | Hệ thống PHẢI sinh được synthetic radar detection từ ground-truth + Gaussian noise model tham số hoá được | A | Must |
| FR-006 | `CameraDetectionSource` PHẢI phát detection ở tick ~10Hz; `RadarDetectionSource` ~20Hz | A | Must |
| FR-007 | `TimeSyncBuffer` PHẢI nội suy detection của mỗi sensor về 1 tick chung (50ms) | B | Must |
| FR-008 | `EgoMotionCompensator` PHẢI bù chuyển động ego (dead-reckoning) cho detection cũ trước khi dùng | B | Must |
| FR-009 | `ExtrinsicCalibration` PHẢI biến đổi detection từ sensor frame sang vehicle frame chung | C | Must |
| FR-010 | Hệ thống PHẢI loại detection ngoài FOV hoặc dưới ngưỡng confidence trước khi vào estimation | C | Must |
| FR-011 | `KalmanFilterCV` PHẢI hiện thực đúng predict/update cho constant-velocity model | D | Must |
| FR-012 | `ExtendedKalmanFilterCTRV` PHẢI hiện thực đúng predict/update cho constant-turn-rate model, xử lý riêng case `\|yaw_rate\| < ε` | D | Must |
| FR-013 | Covariance update PHẢI dùng Joseph form, giữ `P` symmetric + PSD (assert trong debug build) | D | Must |
| FR-014 | Hệ thống PHẢI gán detection↔track bằng Hungarian algorithm trên cost matrix đã gate | E | Must |
| FR-015 | `MahalanobisGate` PHẢI loại cặp track-detection có `d² > χ²_threshold` (df=2, 95%) trước khi assignment | E | Must |
| FR-016 | `TrackManager` PHẢI hiện thực state machine Tentative→Confirmed→Coasting→Deleted đúng ngưỡng N (confirm)/K (coast) | F | Must |
| FR-017 | `TrackFuser` PHẢI hợp nhất track camera+radar bằng Covariance Intersection, chọn `ω` tối thiểu hoá `trace(P_fused)` | G | Must |
| FR-018 | Track chỉ có 1 sensor PHẢI vẫn được output (graceful degrade), không bắt buộc phải có cả 2 sensor | G | Must |
| FR-019 | `TTCEstimator` PHẢI tính TTC từ closing speed tương đối, trả về vô cực nếu không closing | H | Must |
| FR-020 | `PathPredictor` PHẢI lọc object ngoài hành lang quỹ đạo ego dự đoán trước khi tính TTC | H | Should |
| FR-021 | `FcwDecision` PHẢI dùng hysteresis (ngưỡng on/off khác nhau + debounce N frame) để tránh chattering | I | Must |
| FR-022 | `AebDecision` PHẢI có tối thiểu 2 stage braking, ngưỡng TTC nghiêm ngặt hơn FCW | I | Must |
| FR-023 | `AebDecision` PHẢI suppress/giảm stage nếu phát hiện driver override (phanh/đánh lái chủ động) | I | Must |
| FR-024 | Mọi `DecisionOutput` PHẢI kèm `reason` string mô tả căn cứ ra quyết định | I | Must |
| FR-025 | `FixedRateScheduler` PHẢI chạy pipeline ở nhịp cố định 20Hz dùng `steady_clock`, không dùng `system_clock` | J | Must |
| FR-026 | Metrics/log output PHẢI đi qua lock-free SPSC queue, không block real-time thread | J | Must |
| FR-027 | `ScenarioRunner` PHẢI chạy được tối thiểu 20 scenario (14 tuning + 6 holdout), sinh report MOTA/MOTP + precision-recall riêng biệt cho mỗi tập | K | Must |
| FR-028 *(stretch)* | `UnscentedKalmanFilterCTRV` PHẢI hiện thực sigma-point generation + unscented transform đúng công thức chuẩn | D (stretch) | Could |
| FR-029 *(stretch)* | `ImmEstimator` PHẢI hiện thực đủ 4 bước mixing/filtering/model-prob-update/combination | D (stretch) | Could |
| FR-030 *(stretch)* | `Ros2Publisher` PHẢI publish `FusedTrack`/`DecisionOutput`, config-gated tắt mặc định | L (stretch) | Could |

---

## 3. Non-Functional Requirements (NFR)

| ID | Requirement | Category | Verification |
|---|---|---|---|
| NFR-001 | Cycle time p99 phải nhỏ hơn 80% budget (40ms/50ms @20Hz) trên mọi scenario tuning set | Real-time | Profiling report (K) |
| NFR-002 | Cùng seed + cùng config PHẢI cho ra kết quả bit-identical giữa 2 lần chạy | Determinism | `TC-DET-001`: chạy 2 lần, diff output |
| NFR-003 | Toàn bộ Debug build test suite PHẢI pass sạch dưới ASan + UBSan + TSan | Memory/Concurrency safety | CI required job |
| NFR-004 | Codebase PHẢI build sạch (0 warning ở `-Wall -Wextra -Wpedantic`) trên gcc VÀ clang | Portability/Maintainability | CI required job |
| NFR-005 | `P` (covariance) PHẢI luôn symmetric + PSD sau mọi predict/update (debug assert) | Numerical stability | `TC-KF-003`, `TC-EKF-005` |
| NFR-006 | Không có dynamic memory allocation (`new`/heap-growing container) trong real-time hot path (Boundary D-I) ngoài khởi tạo | Real-time/embedded readiness | clang-tidy + code review checklist ([08](08_CODING_STANDARDS_STATIC_ANALYSIS.md)) |
| NFR-007 | Static analysis (clang-tidy + cppcheck + Coverity) PHẢI chạy như required CI check, 0 high-severity finding mới trên PR | Code quality | CI required job |
| NFR-008 | Mọi public API PHẢI có Doxygen comment (mô tả + `@param`/`@return` + `@satisfies` nếu áp dụng) | Documentation | Doxygen `WARN_IF_UNDOCUMENTED=YES` build check |
| NFR-009 | Test coverage (line) trên core estimation/association/decision logic PHẢI ≥ 85% | Testability | lcov report |
| NFR-010 *(stretch)* | Codebase PHẢI cross-compile sạch cho ARM64 và chạy được dưới QEMU | Portability | `ci-arm64.yml` (allow-failure) |

---

## 4. Traceability Matrix (mẫu rút gọn — đại diện mỗi boundary 1-2 dòng, đầy đủ nằm trong `requirements/traceability.csv` khi bắt đầu code)

| Requirement | Design Component | Verification (Test ID) | Status |
|---|---|---|---|
| FR-001, NFR-002 | `config::RunConfigLoader` | `TC-CFG-001` (load valid YAML), `TC-CFG-002` (fail-fast missing field), `TC-DET-001` (bit-identical rerun) | Planned |
| FR-011, FR-013 | `estimation::KalmanFilterCV` | `TC-KF-001` (closed-form constant-velocity predict), `TC-KF-002` (update convergence), `TC-KF-003` (PSD assert never fires over 1000 cycles) | Planned |
| FR-012 | `estimation::ExtendedKalmanFilterCTRV` | `TC-EKF-001` (closed-form circular motion), `TC-EKF-002` (yaw_rate≈0 boundary), `TC-EKF-003` (Jacobian numerical verification) | Planned |
| FR-014, FR-015 | `association::HungarianAssigner`, `association::MahalanobisGate` | `TC-ASSOC-001` (known-optimal cost matrix), `TC-ASSOC-002` (gating rejects implausible pair) | Planned |
| FR-016 | `tracking::TrackManager` | `TC-TRACK-001..004` (state transitions N/K thresholds), `TC-SCEN-occlusion` (multi-object dense scenario) | Planned |
| FR-017, FR-018 | `fusion::TrackFuser` | `TC-FUSION-001` (CI known-covariance property), `TC-SCEN-radar_only_dropout`, `TC-SCEN-camera_only_dropout` | Planned |
| FR-019, FR-020 | `threat::TTCEstimator`, `threat::PathPredictor` | `TC-TTC-001` (closed-form TTC), `TC-TTC-002` (diverging object → ∞) | Planned |
| FR-021, FR-022, FR-023, FR-024 | `decision::FcwDecision`, `decision::AebDecision` | `TC-FCW-001` (no-chatter on synthetic oscillating TTC), `TC-AEB-001` (staged ordering), `TC-AEB-002` (override suppression) | Planned |
| FR-025, FR-026, NFR-001, NFR-003 | `engine::FixedRateScheduler`, `engine::SpscQueue` | `TC-ENGINE-001` (monotonic tick under `steady_clock`), TSan-clean SPSC stress test, p50/p95/p99 profiling report | Planned |
| FR-027 | `metrics::ScenarioRunner`, `metrics::MotaMotpCalculator` | 20+ scenario suite, tuning vs. holdout report | Planned |
| SG-01, SG-02 *(xem 07)* | `decision::FcwDecision`+`decision::AebDecision`+`fusion::TrackFuser` (cross-check trước AEB stage2) | `TC-SAFETY-001..003` (xem 07 §5 Functional Safety Concept) | Planned |

> **Trạng thái `Planned`** vì codebase hiện tại (giai đoạn Master Plan) chưa bắt đầu implement — ma trận này là **hợp đồng thiết kế** viết TRƯỚC khi code, đúng tinh thần "requirements-first" thay vì viết code trước rồi mới nghĩ test là gì.

---

## 5. Change Management (nhẹ, phù hợp solo project)

1. Mọi thay đổi requirement (thêm/sửa FR/NFR) → cập nhật bảng ở file này TRƯỚC khi sửa code.
2. Nếu requirement mới ảnh hưởng Safety Goal đã có ([07](07_SAFETY_HAZARD_ANALYSIS.md)) → phải review lại Functional Safety Concept tương ứng.
3. Mỗi PR/commit liên quan 1 FR/NFR cụ thể nên ghi rõ ID trong commit message (Conventional Commits + ID, vd `feat(estimation): implement Joseph-form update (FR-013)`— xem [08 §7](08_CODING_STANDARDS_STATIC_ANALYSIS.md)).
4. Requirement bị huỷ/deprioritize (vd stretch không kịp làm) → đánh dấu `Deprecated`/`Deferred` trong bảng, không xoá — giữ lịch sử quyết định.

---

## 6. Vì sao làm việc này dù chỉ là project cá nhân?

> **Interview line:** *"Tôi biết đây là dự án cá nhân 60 ngày, không ai bắt tôi viết SRS — nhưng requirements traceability là kỹ năng, không phải thủ tục hành chính; tôi muốn chứng minh tôi có thể làm việc trong 1 team có process thật (automotive/safety-relevant) chứ không chỉ code một mình không ai kiểm tra. Ma trận này cũng giúp chính tôi tránh bỏ sót test case khi 60 ngày code với deadline tự đặt ra."*
