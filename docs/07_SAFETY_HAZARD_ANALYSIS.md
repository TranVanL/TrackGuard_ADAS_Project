# 🛡️ TrackGuard — Hazard Analysis & Risk Assessment (HARA) / Safety Concept

> ⚠️ **DISCLAIMER — ĐỌC TRƯỚC KHI DÙNG FILE NÀY TRONG PHỎNG VẤN:**
> Đây **KHÔNG PHẢI** một safety case được chứng nhận (certified). Đây là 1 tài liệu **tham khảo/portfolio-quality**, viết theo đúng **cấu trúc và phương pháp luận** của ISO 26262 Part 3 (Concept Phase — Hazard Analysis and Risk Assessment) và Part 6 (Functional Safety Concept), áp dụng ở mức độ giáo dục/chứng minh hiểu quy trình — **không** có: independent assessment, confirmation review, tool qualification, hardware-software interaction analysis, hay bất kỳ audit nào từ 1 certification body. Mục tiêu duy nhất: chứng minh tôi hiểu **cách một kỹ sư ADAS thật sẽ suy nghĩ về an toàn**, để thiết kế FCW/AEB không phải chỉ "code cho chạy" mà có căn cứ. Luôn nói rõ điều này CHỦ ĐỘNG khi trình bày (xem [04_PROJECT_SCRIPT.md §6](04_PROJECT_SCRIPT.md)).
>
> 🔗 Cross-ref: [01_ARCHITECTURE.md](01_ARCHITECTURE.md) · [06_REQUIREMENTS_TRACEABILITY.md](06_REQUIREMENTS_TRACEABILITY.md) · [knowledge/03e_TTC_FCW_AEB_DECISION.md](knowledge/03e_TTC_FCW_AEB_DECISION.md)

---

## 1. Item Definition

**Item:** Forward Collision Warning (FCW) và Autonomous Emergency Braking (AEB) decision function trong pipeline TrackGuard, nhận input là `FusedTrack` (đã qua estimation + association + fusion) và ego kinematics, xuất `DecisionOutput` (FCW state, AEB stage, `reason`).

**Giả định vận hành (Assumptions of Use):**
- Xe cấp **SAE Level 2** — tài xế PHẢI giám sát liên tục và có thể can thiệp bất cứ lúc nào (AEB không thay thế trách nhiệm tài xế).
- Vận hành trên đường cao tốc/đô thị tốc độ trung bình-cao, điều kiện thời tiết tốt-trung bình (không model sương mù/tuyết dày trong scope hiện tại).
- Input đến từ camera + radar đã qua estimation/fusion — **không** phân tích lỗi ở mức cảm biến vật lý (physical sensor failure mode) hay ECU hardware.
- Phạm vi phân tích: **software decision logic only** — không bao gồm brake actuator, steering actuator, hay mechanical/electrical failure modes (out of scope cho 1 portfolio C++ project).

---

## 2. Hazard Identification (HAZOP-style, rút gọn)

| # | Malfunction/Deviation | Hazardous Event | Ví dụ tình huống |
|---|---|---|---|
| H1 | **FCW không kích hoạt** khi thực sự có nguy cơ va chạm phía trước (missed warning) | Tài xế không được cảnh báo kịp, phản ứng trễ → va chạm phía sau | Track bị mất (ID switch/occlusion) đúng lúc TTC giảm nhanh |
| H2 | **FCW kích hoạt sai** (false positive) lặp lại nhiều lần | Tài xế mất lòng tin, tắt hệ thống hoặc phớt lờ cảnh báo thật sau này ("cry wolf effect") | Threshold quá nhạy trên đường cua, TTC noise dao động quanh ngưỡng |
| H3 | **AEB không kích hoạt** khi cần thiết (missed intervention) | Không phanh tự động khi tài xế không phản ứng kịp → va chạm | Fusion trả về confidence thấp do 1 sensor dropout, suppress nhầm AEB |
| H4 | **AEB kích hoạt không cần thiết** (unintended activation / false positive braking) | Phanh gấp đột ngột không có lý do → xe phía sau tông vào, hoặc mất kiểm soát lái | Ghost track từ association lỗi (mis-assignment ở khúc cua đông xe) |
| H5 | **AEB không nhường quyền kiểm soát cho tài xế** khi tài xế đã chủ động phản ứng | Xung đột giữa hệ thống và tài xế → phanh/lái không nhất quán, giảm khả năng kiểm soát | `AebDecision` không phát hiện đúng driver override signal |
| H6 | **Track ID switch tại thời điểm quyết định quan trọng** | Decision layer "quên" lịch sử đe doạ, reset debounce counter sai lúc → trễ cảnh báo (liên quan H1) | Hungarian mis-assignment trong scenario multi-object dày đặc |
| H7 | **Kết quả không reproducible** khi debug sự cố sau thực tế | Không trace lại được nguyên nhân gốc của 1 quyết định sai → không sửa được triệt để | Thiếu seeded RNG/ConfigHash (đã giải quyết bằng Boundary Z — xem §5) |

---

## 3. Risk Assessment (ASIL-style — THAM KHẢO, không phải xác định ASIL chính thức)

Phương pháp ISO 26262 dùng 3 tham số: **Severity (S0-S3)**, **Exposure (E0-E4)**, **Controllability (C0-C3)** để tra bảng ra ASIL (QM, A, B, C, D). Áp dụng *ở mức khái niệm* cho H1-H5:

| Hazard | Severity | Exposure | Controllability | ASIL tham khảo* |
|---|---|---|---|---|
| H1 (FCW miss) | S2 (nguy hiểm, có thể chấn thương) | E4 (thường xuyên — mọi chuyến lái xe cao tốc) | C2 (tài xế thường vẫn kiểm soát được nếu phản ứng) | **B** (tham khảo) |
| H2 (FCW false positive lặp lại) | S1 (khó chịu/mất tập trung, hiếm khi trực tiếp gây tai nạn) | E4 | C3 (khó kiểm soát hệ quả dài hạn: mất lòng tin hệ thống) | **QM-A** (tham khảo) |
| H3 (AEB miss) | S3 (nguy hiểm tính mạng) | E3 (khá thường xuyên — tình huống khẩn cấp không hiếm) | C3 (tài xế không kịp phản ứng, đó là lý do cần AEB) | **C** (tham khảo) |
| H4 (AEB false positive) | S2-S3 (tuỳ tốc độ, rủi ro tông sau/mất lái) | E3 | C2-C3 | **B-C** (tham khảo) |
| H5 (AEB không nhường tài xế) | S2 | E2 (ít xảy ra hơn — cần cả 2 điều kiện: nguy cơ + tài xế đã phản ứng) | C2 | **B** (tham khảo) |

> \* **"ASIL tham khảo"**: đây là bài tập áp dụng phương pháp luận, KHÔNG phải kết quả HARA chính thức theo ISO 26262 (thiếu review độc lập, thiếu dữ liệu thống kê exposure thật, thiếu hardware/system context đầy đủ). Nói rõ điều này nếu được hỏi sâu.

---

## 4. Safety Goals (derived từ hazard, cấp cao nhất — trace vào [06](06_REQUIREMENTS_TRACEABILITY.md))

| ID | Safety Goal | Liên quan Hazard |
|---|---|---|
| **SG-01** | Hệ thống KHÔNG được bỏ sót cảnh báo FCW khi TTC giảm dưới ngưỡng nguy hiểm và track liên tục hợp lệ (không do lỗi track lifecycle) | H1, H6 |
| **SG-02** | Hệ thống KHÔNG được kích hoạt AEB stage 2 (phanh toàn lực) chỉ dựa trên 1 nguồn cảm biến không được xác nhận chéo | H4 |
| **SG-03** | Hệ thống PHẢI phát hiện và suppress/giảm AEB khi có tín hiệu driver override rõ ràng | H5 |
| **SG-04** | Mọi quyết định FCW/AEB PHẢI reproducible và truy vết được (không phụ thuộc may rủi runtime) | H7 |
| **SG-05** | Hệ thống PHẢI duy trì continuity của track (không mất ID) qua occlusion ngắn để tránh reset sai trạng thái quyết định | H6 |

---

## 5. Functional Safety Concept — Safety Goal → Technical Safety Requirement → Kiến trúc → Kiểm chứng

| Safety Goal | Technical Safety Requirement (TSR) | Kiến trúc thực hiện | Kiểm chứng (test) |
|---|---|---|---|
| SG-01 | TSR-01: FCW hysteresis phải dùng debounce dựa trên track đã qua lifecycle ổn định, không reset khi track ở trạng thái `Coasting` (không phải `Deleted`) | `FcwDecision` chỉ đọc TTC từ track `Confirmed`/`Coasting`, không tính lại từ track mới nếu track cũ còn tồn tại | `TC-FCW-002` (occlusion ngắn không làm mất debounce) |
| SG-02 | TSR-02: AEB stage 2 chỉ trigger khi `FusedTrack.contributing_sensors.size() >= 2` HOẶC single-sensor confidence vượt ngưỡng rất cao + TTC cực ngắn (fallback an toàn) | `AebDecision` kiểm tra `contributing_sensors` trước khi cho phép leo stage 2 | `TC-SAFETY-001` (single-sensor ghost track không kích hoạt stage2) |
| SG-03 | TSR-03: Driver-override signal (brake pressure/steering input) phải được đọc và áp dụng suppression TRƯỚC khi tính stage escalation, không phải sau | `AebDecision::step()` check override đầu hàm | `TC-AEB-002` |
| SG-04 | TSR-04: Toàn bộ RNG đi qua seeded `std::mt19937_64`; mọi report kèm `ConfigHash` | Boundary Z (`RunConfig`, `ConfigHash`) | `TC-DET-001` |
| SG-05 | TSR-05: Track chỉ chuyển `Coasting`→`Deleted` sau K frame liên tiếp unmatched (không xoá ngay lập tức khi unmatched 1 frame) | `TrackManager` state machine | `TC-TRACK-003`, `TC-SCEN-occlusion` |

---

## 6. Safe State Definition

Khi hệ thống phát hiện **input không hợp lệ/implausible** ở bất kỳ boundary nào (vd Cholesky decomposition fail trên `P` không PSD, config load fail, track association trả về ma trận cost toàn `NaN`) → hệ thống chuyển vào **Safe State**:

> **Safe State = "FCW/AEB decision layer output `Idle`/`NoStage`, log lỗi rõ ràng với `reason`, KHÔNG đưa ra bất kỳ quyết định braking/warning nào dựa trên dữ liệu không đáng tin."**

Đây là lựa chọn thiết kế **fail-silent** (không phải fail-operational) — hợp lý cho 1 hệ thống L2 (driver vẫn giám sát), khác với hệ thống L4+ cần fail-operational (out of scope).

---

## 7. Giới hạn phân tích (Limitations — nói rõ khi được hỏi)

- Không có Hardware-Software Interaction Analysis (HSIA) — giả định hardware/actuator hoàn hảo.
- Không có Independent Safety Assessment hay Confirmation Review theo ISO 26262 Part 2.
- Exposure/Controllability là ước lượng chủ quan của tác giả, không dựa trên dữ liệu thống kê tai nạn thật.
- Không phân tích Common Cause Failure hay Dependent Failure giữa camera/radar (giả định lỗi độc lập — đơn giản hoá).
- Không có ASIL decomposition (phân rã ASIL qua redundancy) — chỉ dừng ở mức xác định ASIL tham khảo cho toàn item.
- Phạm vi chỉ là decision *software logic*; không bao gồm actuator, brake hydraulics, hay E/E architecture thật.

---

## 8. Vì sao vẫn đáng làm dù không certified?

> **Interview line:** *"Tôi biết rõ đây không phải 1 safety case thật — nhưng tư duy 'từ hazard → safety goal → technical requirement → kiến trúc → test' là cách 1 kỹ sư ADAS senior thiết kế decision logic, không phải chỉ chọn ngưỡng TTC theo cảm tính. Khi tôi nói AEB stage 2 cần cross-check 2 sensor, đó không phải ngẫu nhiên — nó trace thẳng về SG-02 vì tôi đã phân tích hazard 'AEB false positive' trước khi viết code."*
