# 🧰 TrackGuard — Coding Standard, Static Analysis & Release Discipline

> **Mục đích:** Chứng minh code không chỉ "chạy đúng" mà còn viết theo kỷ luật kỹ thuật automotive-flavored — điều interviewer senior/embedded/automotive sẽ soi kỹ hơn cả thuật toán. File này định nghĩa: coding standard subset (MISRA C++/AUTOSAR C++14-inspired), static analysis toolchain, code review checklist, documentation standard, và versioning/release discipline.
>
> 🔗 Cross-ref: [01_ARCHITECTURE.md](01_ARCHITECTURE.md) · [06_REQUIREMENTS_TRACEABILITY.md](06_REQUIREMENTS_TRACEABILITY.md) · [knowledge/03g_TESTING_VALIDATION.md](knowledge/03g_TESTING_VALIDATION.md) · [knowledge/03f_REALTIME_CPP_LINUX.md](knowledge/03f_REALTIME_CPP_LINUX.md)

---

## 1. Vì sao cần 1 coding standard cho project cá nhân?

MISRA C++:2008 và AUTOSAR C++14 có **300+ rule** — không khả thi áp dụng đầy đủ cho 1 người trong 60 ngày, và nhiều rule (vd cấm hoàn toàn exception, cấm STL) không hợp lý cho 1 project học thuật dùng C++17/Eigen/gtest hiện đại. Thay vào đó, **TrackGuard Coding Standard v1.0** chọn lọc **~24 rule có giá trị cao nhất** — những rule mà vi phạm thực sự gây bug nghiêm trọng trong hệ thống real-time/estimation, và có thể **enforce tự động bằng static analysis** thay vì chỉ "tự giác".

> **Nguyên tắc chọn rule:** ưu tiên rule (a) ngăn được lớp lỗi đã từng xảy ra trong dự án này (vd covariance không PSD, chia cho 0 ở yaw_rate), (b) enforce được bằng tool (không phải "quy tắc danh dự"), (c) không mâu thuẫn với C++17 idiomatic modern code.

---

## 2. TrackGuard Coding Standard v1.0 (24 rule, đánh số TGCS-xxx)

### Nhóm A — An toàn bộ nhớ & khởi tạo

| ID | Rule | Lý do |
|---|---|---|
| TGCS-01 | Mọi biến PHẢI được khởi tạo tại nơi khai báo (không khai báo rồi gán sau) | Tránh UB đọc giá trị chưa xác định — UBSan bắt được nhưng rule này ngăn từ code review |
| TGCS-02 | Cấm `new`/`delete` trực tiếp trong real-time hot path (Boundary D-I) — dùng RAII/pre-allocated buffer/`std::array` | NFR-006 — tránh heap allocation không xác định latency trong chu kỳ 50ms |
| TGCS-03 | Mọi class quản lý resource PHẢI tuân Rule of Zero hoặc Rule of Five đầy đủ (không Rule of Three nửa vời) | Tránh double-free/leak khi copy/move không định nghĩa đúng |
| TGCS-04 | Cấm truyền con trỏ thô (`T*`) sở hữu ownership qua interface — dùng `std::unique_ptr`/`std::shared_ptr`/reference | Rõ ràng ai sở hữu, ai chỉ mượn — enforce bằng `cppcoreguidelines-owning-memory` |

### Nhóm B — Số học & UB

| ID | Rule | Lý do |
|---|---|---|
| TGCS-05 | Cấm phép chia không kiểm tra mẫu số có thể ≈0 (case CTRV `yaw_rate`) — PHẢI có guard `if (\|denom\| < eps)` | Bug đã từng gặp dạng này ở CTRV Jacobian — xem [05b Q10](05b_INTERVIEW_ESTIMATION_TRACKING.md) |
| TGCS-06 | Cấm narrowing conversion ngầm định (`double`→`float`, `int64`→`int32`) — PHẢI dùng `static_cast` tường minh | UBSan/`-Wconversion` bắt được, nhưng rule ngăn từ review |
| TGCS-07 | Không dùng `assert()` cho input validation ở boundary công khai — chỉ dùng cho invariant nội bộ (debug build); input từ config/sensor PHẢI validate bằng exception/error code | `assert` bị compile out ở Release — input xấu từ config file phải fail rõ ràng cả ở Release |
| TGCS-08 | Mọi phép so sánh floating-point bằng nhau (`==`) PHẢI thay bằng so sánh sai số (`std::abs(a-b) < eps`) trừ khi so sánh với hằng số đặc biệt (0.0, chưa khởi tạo) | Tránh lỗi hội tụ/gating sai do rounding error |

### Nhóm C — Covariance & Estimation-specific

| ID | Rule | Lý do |
|---|---|---|
| TGCS-09 | Mọi hàm cập nhật covariance `P` PHẢI dùng Joseph form, không dùng `(I-KH)P` đơn giản hoá | FR-013 — đã trace nguyên nhân 1 bug thật do form đơn giản hoá |
| TGCS-10 | Debug build PHẢI assert `P` symmetric (`(P - Pᵀ).norm() < eps`) và PSD (Cholesky không fail) sau mọi predict/update | NFR-005 |
| TGCS-11 | Mọi ma trận Jacobian PHẢI có unit test verify bằng finite-difference numerical Jacobian | Bắt lỗi đạo hàm sai sớm, trước khi lan ra RMSE cuối |

### Nhóm D — Real-time & Concurrency

| ID | Rule | Lý do |
|---|---|---|
| TGCS-12 | Cấm `std::mutex`/blocking lock trong real-time thread (Boundary J trở lên trong pipeline chính) | Tránh priority inversion/unbounded blocking — dùng lock-free SPSC thay thế |
| TGCS-13 | Mọi truy cập biến chia sẻ giữa thread PHẢI dùng `std::atomic` với memory order tường minh (không dùng `memory_order_seq_cst` mặc định nếu acquire/release đủ) | Hiệu năng + rõ ràng ý định đồng bộ hoá |
| TGCS-14 | Timing/scheduling PHẢI dùng `std::chrono::steady_clock`, cấm `system_clock` cho bất kỳ tính toán duration nào | FR-025 |
| TGCS-15 | Mọi lock-free data structure PHẢI có TSan stress test riêng trước khi merge | Đã note ở [05d Q4-Q5](05d_INTERVIEW_SYSTEM_VALIDATION.md) |

### Nhóm E — Cấu trúc & Readability

| ID | Rule | Lý do |
|---|---|---|
| TGCS-16 | Cấm magic number trong logic quyết định (TTC threshold, N/K lifecycle...) — PHẢI là `constexpr` có tên, đọc từ `RunConfig` nếu tunable | Traceability + dễ tuning theo scenario, không sửa rải rác trong code |
| TGCS-17 | Header PHẢI include-guard bằng `#pragma once` (không dùng macro guard thủ công) | Đơn giản, chuẩn hiện đại, tránh macro trùng tên |
| TGCS-18 | Cấm `using namespace std;` (hoặc bất kỳ namespace nào) ở phạm vi header/global | Tránh ODR/ambiguity khi project lớn dần |
| TGCS-19 | Mọi constructor 1 tham số PHẢI `explicit` trừ khi cố ý cho phép implicit conversion (phải có comment giải thích lý do) | Tránh implicit conversion không mong muốn (vd `Track(int id)` vô tình nhận `float`) |
| TGCS-20 | Hàm PHẢI ≤ 60 dòng logic (không tính comment/blank) — vượt quá PHẢI refactor tách hàm con | Đọc/test/review dễ hơn — enforce bằng clang-tidy `readability-function-size` |
| TGCS-21 | Cấm recursion không có bound rõ ràng trong bất kỳ code path nào chạy trong real-time thread | Tránh stack overflow không dự đoán được trong hệ thống real-time |
| TGCS-22 | Enum PHẢI dùng `enum class` (scoped), không dùng enum trần | Tránh xung đột tên, ép kiểu ngầm định nguy hiểm |
| TGCS-23 | Mọi public API (header) PHẢI có Doxygen comment đầy đủ (`@brief`, `@param`, `@return`, và `@satisfies{FR-xxx}` nếu implement 1 requirement — xem [06](06_REQUIREMENTS_TRACEABILITY.md)) | NFR-008 |
| TGCS-24 | Commit message PHẢI theo Conventional Commits (`feat:`/`fix:`/`test:`/`docs:`/`refactor:`/`chore:`), kèm requirement ID nếu áp dụng | Traceability lịch sử, changelog tự sinh được |

> **Full rule text + rationale + code example (đúng/sai)** sẽ được viết chi tiết trong `docs/CODING_STANDARD.md` khi bắt đầu code — file này là bảng tóm tắt tra cứu nhanh.

---

## 3. Static Analysis Toolchain

| Tool | Vai trò | Cấu hình |
|---|---|---|
| **clang-tidy** | Bắt bug pattern + enforce style tự động, chạy mỗi PR | Checks bật: `bugprone-*`, `cppcoreguidelines-*` (owning-memory, pro-type-*), `misc-*`, `modernize-*` (trừ vài check quá aggressive), `readability-function-size`, `performance-*` — file `.clang-tidy` ở repo root |
| **cppcheck** | Bổ sung phát hiện cppcheck-specific (unused, null deref, buffer overflow pattern) mà clang-tidy có thể miss | Chạy `--enable=all --inconclusive` trong CI, review manual các finding "inconclusive" |
| **Coverity Scan** | Phân tích tĩnh sâu (interprocedural), đặc biệt mạnh cho memory/resource leak và concurrency defect — công cụ dùng thật trong công nghiệp automotive | Free tier cho open-source project; chạy định kỳ (không mỗi PR vì giới hạn build/tháng), báo cáo defect density trong `docs/coverity_report.md` |
| **include-what-you-use (IWYU)** | Giữ include list tối giản, tránh transitive-include phụ thuộc ngầm | Chạy optional, không block CI (nice-to-have) |
| **ASan/UBSan/TSan** | Runtime sanitizers — bổ sung cho static analysis (bắt lỗi static analysis không thấy vì cần runtime context) | Đã tích hợp CI matrix — xem [knowledge/03g](knowledge/03g_TESTING_VALIDATION.md) |

**CI Gate policy:** clang-tidy + cppcheck chạy **required** trên mọi PR — build fail nếu có finding severity `error`/`warning` MỚI (so với baseline). Coverity chạy định kỳ (weekly hoặc trước mỗi tag), không block PR nhưng defect phải được triage trong 1 tuần. Mọi suppression (`// NOLINT`) PHẢI kèm comment lý do — suppression không lý do bị coi là code smell trong review.

---

## 4. Code Review Checklist (áp dụng self-review vì solo project, hoặc dùng khi có reviewer)

- [ ] Có test mới cho logic mới, và test đó thực sự fail nếu logic bị revert (không phải test giả)
- [ ] Không có finding static-analysis mới (clang-tidy/cppcheck sạch)
- [ ] Không có warning mới ở `-Wall -Wextra -Wpedantic`
- [ ] Nếu sửa covariance/estimation logic → chạy lại toàn bộ PSD assert test + closed-form test
- [ ] Nếu sửa real-time path → không có `new`/mutex mới trong hot path (TGCS-02, TGCS-12)
- [ ] Public API mới có Doxygen comment + `@satisfies{FR-xxx}` nếu áp dụng
- [ ] Commit message theo Conventional Commits, có ID requirement nếu áp dụng (TGCS-24)
- [ ] Nếu ảnh hưởng Safety Goal ([07](07_SAFETY_HAZARD_ANALYSIS.md)) → note rõ trong PR description

---

## 5. Documentation Standard (Doxygen)

- Mọi header public PHẢI generate được Doxygen sạch, build với `WARN_IF_UNDOCUMENTED = YES` — CI fail nếu có warning undocumented API mới.
- Format chuẩn:
```cpp
/// @brief Predict step of the Constant-Velocity Kalman Filter.
/// @param dt Time delta since last predict, in seconds. Must be > 0.
/// @satisfies{FR-011}
void predict(double dt);
```
- File-level comment mỗi header nêu rõ boundary nó thuộc về (vd `// Boundary D: State Estimation`).

---

## 6. Versioning & Release Discipline

- **Semantic Versioning**: `v1.0.0` = tất cả core checkpoint P1-P12 xanh; `v1.1.0` = stretch (UKF/IMM/ARM64/ROS2) hoàn thành; patch version cho bugfix sau tag.
- **CHANGELOG.md** — cập nhật mỗi tag, nhóm theo Conventional Commit type (Features/Fixes/Tests/Docs).
- **Tag theo tuần**: `w01-done` → `w12-done` trong quá trình dev (xem [00_MASTER_PLAN.md](00_MASTER_PLAN.md)), tag chính thức `v1.0`/`v1.1` khi ship.
- **Branch strategy** (dù solo, vẫn áp dụng để luyện thói quen production): `main` luôn xanh CI; feature branch `feat/xxx` merge qua PR (dù self-review) — không commit thẳng vào `main` cho thay đổi lớn hơn 1 file nhỏ.
- **PR template** (`.github/PULL_REQUEST_TEMPLATE.md`) nhắc checklist §4 tự động mỗi lần mở PR.

---

## 7. Lightweight Requirements Traceability Tooling (kế hoạch, xem [06 §1](06_REQUIREMENTS_TRACEABILITY.md))

`tools/trace_check.py` (kế hoạch, viết ở P12 cùng lúc CI hoàn thiện): quét `@satisfies{FR-xxx}`/`@verifies{FR-xxx}` trong toàn bộ `include/`, `src/`, `tests/`, đối chiếu với bảng requirement trong [06](06_REQUIREMENTS_TRACEABILITY.md), cảnh báo requirement Must chưa có test nào verify. Chạy optional trong CI (không block, chỉ report) — mục tiêu là công cụ hỗ trợ tự-kiểm tra, không phải gate cứng gây trễ tiến độ solo project.

---

## 8. Vì sao đáng làm việc này?

> **Interview line:** *"Tôi không kỳ vọng ai review code của tôi vì đây là project cá nhân — nhưng tôi tự áp đặt kỷ luật này để luyện thói quen làm việc trong 1 team automotive thật, nơi static analysis, coding standard, và traceability là bắt buộc chứ không phải tuỳ chọn. Nếu được hỏi 'MISRA rule nào bạn thấy quan trọng nhất', tôi sẽ nói về guard chia-cho-0 ở CTRV Jacobian (TGCS-05) — vì đó là bug thật tôi từng gặp, không phải lý thuyết suông."*
