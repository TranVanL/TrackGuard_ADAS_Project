# 📚 TrackGuard — Knowledge Index (HUB)

> **Đây là file trung tâm.** Kiến thức nền cần học được tách thành 8 file con theo chủ đề, đọc TRƯỚC phase tương ứng trong [02_PLAN_DETAILED_60DAY.md](../02_PLAN_DETAILED_60DAY.md). Mục tiêu: hiểu đúng lý thuyết TRƯỚC khi code, vì sai công thức (Kalman gain, Hungarian cost, TTC) sẽ lan lỗi ra toàn pipeline mà rất khó debug ngược.
>
> 📐 Kiến thức **quy trình kỹ thuật** (không phải thuật toán) — requirements traceability, hazard analysis/safety concept, coding standard/static analysis — nằm ở 3 file root riêng, đọc song song: [../06_REQUIREMENTS_TRACEABILITY.md](../06_REQUIREMENTS_TRACEABILITY.md) · [../07_SAFETY_HAZARD_ANALYSIS.md](../07_SAFETY_HAZARD_ANALYSIS.md) · [../08_CODING_STANDARDS_STATIC_ANALYSIS.md](../08_CODING_STANDARDS_STATIC_ANALYSIS.md).

---

## 📖 Cấu trúc 8 file con

| File | Đọc trước Phase | Nội dung |
|---|---|---|
| **[03a_SENSORS_PERCEPTION.md](03a_SENSORS_PERCEPTION.md)** | P1-P2 | Camera/radar/lidar detection model, noise model, FOV, synthetic sensor generation |
| **[03b_KALMAN_FILTER_ESTIMATION.md](03b_KALMAN_FILTER_ESTIMATION.md)** | P4-P5 | KF/EKF math đầy đủ, CV/CTRV motion model, tuning Q/R, numerical stability |
| **[03c_DATA_ASSOCIATION_TRACKING.md](03c_DATA_ASSOCIATION_TRACKING.md)** | P6 | Hungarian algorithm, Mahalanobis/IoU gating, track lifecycle, MOTA/MOTP |
| **[03d_SENSOR_FUSION_ARCHITECTURE.md](03d_SENSOR_FUSION_ARCHITECTURE.md)** | P3, P7 | Time sync, ego-motion, calibration, early vs late fusion, covariance intersection |
| **[03e_TTC_FCW_AEB_DECISION.md](03e_TTC_FCW_AEB_DECISION.md)** | P8-P10 | TTC formula, FCW hysteresis, AEB staged logic, ISO 26262/NCAP tham khảo |
| **[03f_REALTIME_CPP_LINUX.md](03f_REALTIME_CPP_LINUX.md)** | P11 | Fixed-rate scheduler, lock-free queue, profiling, sanitizers |
| **[03g_TESTING_VALIDATION.md](03g_TESTING_VALIDATION.md)** | P12 | Unit test strategy, scenario-based testing, tuning/holdout split, metrics, CI |
| **[03h_ADVANCED_ESTIMATION_UKF_IMM.md](03h_ADVANCED_ESTIMATION_UKF_IMM.md)** | Stretch (P13) | Sigma-point/Unscented Transform, UKF, IMM model-probability blending |

---

## 🗺 Lộ trình học theo tuần (map với 02_PLAN_DETAILED_60DAY.md)

| Tuần | File cần đọc | Trọng tâm |
|---|---|---|
| W1-W2 | 03a §1-5 | Sensor model + noise + synthetic generation + seeded RNG/determinism |
| W3 | 03d §1-4 | Time sync + ego-motion + calibration + early/late fusion overview |
| W4-W5 | 03b toàn bộ | KF (CV) + EKF (CTRV) — công thức đầy đủ |
| W6 | 03c §1-4 | Hungarian + gating + track lifecycle |
| W7 | 03d §5-6 | Covariance intersection + track-to-track fusion |
| W8-W10 | 03e toàn bộ | TTC + FCW + AEB |
| W11 | 03f toàn bộ | Real-time engine patterns |
| W12 | 03g toàn bộ | Validation + tuning/holdout methodology + metrics + CI |
| Stretch (P13) | 03h toàn bộ | UKF sigma points + IMM model blending, ARM64/ROS2 (xem 00_MASTER_PLAN §11) |

---

## 🎯 Nguyên tắc học (áp dụng cho cả 7 file)

1. **Hiểu công thức trước khi code** — mỗi công thức toán phải viết ra tay được, không chỉ copy-paste
2. **Mỗi khái niệm gắn với 1 unit test cụ thể** — "tôi hiểu KF" nghĩa là có test chứng minh, không phải chỉ đọc xong
3. **Honesty boundary rõ ràng** — TrackGuard dùng model **classical** (Kalman, Hungarian, rule-based decision), KHÔNG dùng deep learning cho detection/tracking. Nếu bị hỏi "sao không dùng DL tracker (DeepSORT/ByteTrack)?" → trả lời thẳng: *"Portfolio này chọn classical pipeline có chủ đích để chứng minh hiểu toán nền tảng (estimation theory, combinatorial optimization) — nền tảng này là bắt buộc để hiểu cả DL-based tracker sau này, vì DeepSORT vẫn dùng Kalman Filter + Hungarian bên trong, chỉ thay appearance feature bằng CNN embedding."*
4. **Mọi số liệu tham khảo (NCAP threshold, chi-square table) đều note rõ nguồn/tính chất tham khảo**, không claim là số chính thức của 1 hãng xe cụ thể

---

## 🔑 Thuật ngữ cốt lõi cần thuộc lòng (glossary nhanh)

| Thuật ngữ | Định nghĩa 1 dòng |
|---|---|
| **CV model** | Constant Velocity — object di chuyển vận tốc không đổi |
| **CTRV model** | Constant Turn-Rate and Velocity — object di chuyển vận tốc + tốc độ quay không đổi (khi cua) |
| **Kalman Gain (K)** | Trọng số quyết định tin đo lường hay tin dự đoán hơn |
| **Mahalanobis distance** | Khoảng cách có tính đến hiệp phương sai (covariance) — khác Euclidean vì tôn trọng độ bất định |
| **Gating** | Loại bỏ sớm cặp track-detection không hợp lý trước khi assignment |
| **Hungarian algorithm** | Thuật toán tối ưu assignment 1-1 chi phí nhỏ nhất, O(n³) |
| **Track lifecycle** | Vòng đời 1 track: Tentative → Confirmed → Coasting → Deleted |
| **Early/Low-level fusion** | Hợp nhất dữ liệu thô trước khi ra track |
| **Late/Track-level fusion** | Mỗi sensor ra track riêng, hợp nhất track ở bước cuối |
| **Covariance Intersection** | Công thức hợp nhất 2 ước lượng không biết độ tương quan, vẫn ra kết quả "an toàn" (consistent) |
| **TTC** | Time-To-Collision — thời gian còn lại tới va chạm nếu giữ nguyên vận tốc tương đối |
| **FCW** | Forward Collision Warning — cảnh báo, không tự phanh |
| **AEB** | Autonomous Emergency Braking — tự động phanh |
| **MOTA/MOTP** | Multi-Object Tracking Accuracy/Precision — chỉ số chuẩn đánh giá chất lượng tracking |
| **Hysteresis** | Cơ chế chống dao động trạng thái (cần vượt ngưỡng theo 2 chiều khác nhau để đổi state) |
| **Sigma points** | Tập điểm rời rạc đại diện phân phối xác suất, dùng trong UKF (Unscented Transform) |
| **Unscented Transform** | Kỹ thuật truyền phân phối qua hàm non-linear bằng sigma points thay vì tuyến tính hoá |
| **IMM** | Interacting Multiple Model — chạy song song nhiều motion model, trộn theo xác suất động |
| **Model probability (μ)** | Xác suất "model này đúng tại thời điểm này" trong IMM, cập nhật theo likelihood đo lường |
| **Determinism/Reproducibility** | Cùng seed + cùng config → cùng kết quả tuyệt đối, điều kiện tiên quyết để scenario test đáng tin |
| **ConfigHash** | Hash (SHA-256) của file config, in vào report để chứng minh kết quả ứng với đúng config nào |
| **Tuning set / Holdout set** | Tách scenario thành tập dùng để chọn threshold và tập chỉ dùng để đánh giá cuối, chống overfitting |
