# 📚 03h — Advanced Estimation: UKF & IMM (Stretch)

> **Đọc trước:** Phase 13 / Stretch Week (sau khi `v1.0` đã ship). File này KHÔNG bắt buộc cho v1.0 — chỉ cần cho ai muốn đẩy project lên mức "senior sensor fusion" thật sự. Yêu cầu đã hiểu vững [03b_KALMAN_FILTER_ESTIMATION.md](03b_KALMAN_FILTER_ESTIMATION.md) trước khi đọc file này.

---

## 1. Vì sao cần UKF khi đã có EKF?

EKF tuyến tính hoá non-linear function `f()`/`h()` bằng Jacobian tại 1 điểm (first-order Taylor expansion). Vấn đề:

- Sai số tuyến tính hoá tăng khi **non-linearity mạnh** (yaw_rate lớn, cua gấp) hoặc **covariance lớn** (uncertainty cao)
- Phải tính Jacobian tay — dễ sai dấu, dễ quên hạng tử, khó maintain khi đổi motion model
- Với model càng phức tạp (bicycle model, non-holonomic constraint) Jacobian càng cồng kềnh

**UKF (Unscented Kalman Filter)** né hoàn toàn việc tuyến tính hoá: thay vì xấp xỉ hàm, nó xấp xỉ **phân phối xác suất** bằng một tập điểm rời rạc (sigma points) đi qua đúng hàm non-linear thật, rồi khôi phục mean/covariance từ các điểm đã biến đổi. Về lý thuyết, UKF chính xác tới bậc 2 (2nd-order) của khai triển Taylor cho mọi non-linear function, trong khi EKF chỉ chính xác bậc 1.

> **Interview line:** *"UKF không tuyến tính hoá hàm — nó lấy mẫu đại diện phân phối rồi truyền qua hàm thật, nên tránh được sai số tuyến tính hoá của EKF, đổi lại chi phí tính toán cao hơn một chút (2n+1 lần evaluate f() thay vì 1 lần + Jacobian)."*

---

## 2. Sigma Point Generation

Với state dimension `n` (CTRV: n=5), sinh `2n+1` sigma points:

$$
\chi_0 = \hat{x}, \qquad
\chi_i = \hat{x} + \left(\sqrt{(n+\lambda)P}\right)_i \ (i=1..n), \qquad
\chi_{i+n} = \hat{x} - \left(\sqrt{(n+\lambda)P}\right)_i \ (i=1..n)
$$

- $\lambda = \alpha^2(n+\kappa) - n$ — tham số scaling (thường $\alpha=10^{-3}$, $\kappa=0$, $\beta=2$ cho phân phối Gaussian)
- $\sqrt{(n+\lambda)P}$ = Cholesky decomposition của ma trận $(n+\lambda)P$ (cột thứ $i$ của ma trận căn)
- Mỗi sigma point có trọng số mean $W_i^{(m)}$ và trọng số covariance $W_i^{(c)}$:

$$
W_0^{(m)} = \frac{\lambda}{n+\lambda}, \qquad
W_0^{(c)} = \frac{\lambda}{n+\lambda} + (1-\alpha^2+\beta), \qquad
W_i^{(m)} = W_i^{(c)} = \frac{1}{2(n+\lambda)} \ (i \ge 1)
$$

```cpp
Eigen::MatrixXd generateSigmaPoints(const Eigen::VectorXd& x, const Eigen::MatrixXd& P, double lambda) {
    int n = x.size();
    Eigen::MatrixXd A = ((n + lambda) * P).llt().matrixL();  // Cholesky
    Eigen::MatrixXd sigma(n, 2 * n + 1);
    sigma.col(0) = x;
    for (int i = 0; i < n; ++i) {
        sigma.col(i + 1)     = x + A.col(i);
        sigma.col(i + 1 + n) = x - A.col(i);
    }
    return sigma;
}
```

> **Assert quan trọng:** `P` phải PSD trước khi Cholesky — nếu `llt().info() == Eigen::NumericalIssue` → bug ở bước update trước đó (giống lý do phải giữ Joseph form trong EKF).

---

## 3. Predict step (Unscented Transform)

1. Sinh sigma points từ $(\hat{x}_{k-1}, P_{k-1})$
2. Truyền TỪNG điểm qua hàm chuyển động CTRV thật `f()` (không cần Jacobian):
   $$\chi_i^* = f(\chi_i, \Delta t)$$
3. Khôi phục mean/covariance dự đoán:
   $$
   \hat{x}_k^- = \sum_i W_i^{(m)} \chi_i^*, \qquad
   P_k^- = \sum_i W_i^{(c)} (\chi_i^* - \hat{x}_k^-)(\chi_i^* - \hat{x}_k^-)^T + Q
   $$

## 4. Update step

1. Truyền sigma points dự đoán qua hàm đo `h()`: $Z_i = h(\chi_i^*)$
2. Tính mean đo dự đoán $\hat{z}_k^-= \sum_i W_i^{(m)} Z_i$, covariance đo $S_k$, cross-covariance $T_k$
3. Kalman gain $K = T_k S_k^{-1}$, update như KF chuẩn: $\hat{x}_k = \hat{x}_k^- + K(z_k - \hat{z}_k^-)$, $P_k = P_k^- - K S_k K^T$

## 5. So sánh EKF vs UKF (thực nghiệm dự kiến trong project)

| Tiêu chí | EKF | UKF |
|---|---|---|
| Độ chính xác non-linear mạnh | Giảm khi yaw_rate lớn | Ổn định hơn |
| Chi phí tính toán / cycle | Thấp (1× f() + Jacobian) | Cao hơn (2n+1× f()) |
| Độ phức tạp code | Cần derive + maintain Jacobian | Không cần Jacobian, dễ đổi motion model hơn |
| Rủi ro số học | Linearization error tích luỹ | Cholesky fail nếu P không PSD |
| Khi nào chọn | Real-time budget chặt, non-linearity vừa phải | Non-linearity mạnh, đủ compute budget |

> Báo cáo RMSE thật (đo trên scenario cua gấp) sẽ được ghi vào `docs/ukf_vs_ekf_report.md` ở Stretch Day 61-62.

---

## 6. IMM (Interacting Multiple Model)

**Vấn đề:** dùng cố định 1 model (CV hoặc CTRV) cho toàn bộ track là sai lệch thực tế — 1 xe có lúc đi thẳng (CV đúng hơn), có lúc rẽ (CTRV đúng hơn). Chọn sai model tại 1 thời điểm → track "giật" hoặc trễ phản ứng.

**IMM chạy song song N filter** (thường CV + CTRV), mỗi filter có xác suất "đây là model đúng" $\mu_j$, cập nhật theo:

1. **Mixing** — trộn state đầu vào mỗi filter theo xác suất chuyển đổi Markov $p_{ij}$ (xác suất chuyển từ model $i$ sang $j$) và xác suất hiện tại $\mu_i$:
   $$
   \bar{\mu}_{i|j} = \frac{p_{ij}\mu_i}{\sum_k p_{kj}\mu_k}, \qquad
   \hat{x}_{0j} = \sum_i \bar{\mu}_{i|j} \hat{x}_i
   $$
2. **Filtering** — mỗi filter chạy predict+update riêng như bình thường với state đã mix
3. **Model probability update** — dùng likelihood $\Lambda_j$ (đo mức "hợp lý" của đo lường với model $j$) để cập nhật $\mu_j$:
   $$
   \mu_j = \frac{\Lambda_j \sum_i p_{ij}\mu_i}{\sum_k \Lambda_k \sum_i p_{ik}\mu_i}
   $$
4. **Combination** — state cuối cùng là trung bình trọng số của tất cả filter theo $\mu_j$ mới

```cpp
struct ImmEstimator {
    std::vector<KalmanFilterBase*> models;  // [0]=CV, [1]=CTRV
    Eigen::MatrixXd transition_prob;        // p_ij, 2x2 — vd [[0.95,0.05],[0.10,0.90]]
    Eigen::VectorXd model_prob;             // mu_j, khởi tạo [0.5, 0.5]

    void step(const Measurement& z, double dt) {
        mixInputs();                 // bước 1
        for (auto* m : models) { m->predict(dt); m->update(z); }  // bước 2
        updateModelProbabilities(z); // bước 3
        combineEstimates();          // bước 4
    }
};
```

> **Interview line:** *"IMM không phải chọn model đúng rồi bỏ qua model sai — nó chạy CẢ HAI song song và trộn kết quả theo xác suất động, nên chuyển đổi giữa đi thẳng và rẽ mượt hơn nhiều so với hard-switch giữa CV và CTRV."*

---

## 7. Khi nào KHÔNG cần UKF/IMM (honesty note)

Với hầu hết scenario ADAS L2 đơn giản (bám làn, phanh khẩn cấp phía trước), EKF/CTRV đã đủ chính xác — UKF/IMM chỉ có lợi ích rõ rệt khi:
- Object đổi hướng đột ngột và liên tục (giao lộ, bãi đỗ xe)
- Ngân sách tính toán cho phép chạy N filter song song mà vẫn giữ real-time budget

Đây là lý do project đặt UKF/IMM ở **Stretch**, không phải core — chứng minh hiểu **trade-off kỹ thuật**, không phải "thêm cho nhiều".
