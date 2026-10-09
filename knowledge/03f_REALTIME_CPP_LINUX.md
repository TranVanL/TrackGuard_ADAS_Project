# ⚙️ 03f — Real-time C++/Linux Engine (Fixed-Rate Scheduler, Lock-Free Queue, Profiling)

> Đọc trước Phase 11 ([00_MASTER_PLAN.md](../00_MASTER_PLAN.md) P11). File này chia sẻ tinh thần với kiến trúc real-time của **EventStreamCore** — TrackGuard không copy code, mà tái sử dụng pattern đã học, áp dụng vào bối cảnh pipeline ADAS (nhịp cố định thay vì throughput tối đa).

---

## 1. Vì sao ADAS pipeline cần deterministic fixed-rate, không phải "chạy nhanh nhất có thể"

Khác với EventStreamCore (tối ưu **throughput** — càng nhiều event/s càng tốt), pipeline ADAS cần tối ưu **tính xác định (determinism)**: mỗi chu kỳ (cycle) phải chạy **đúng nhịp cố định** (vd 20Hz = 50ms/cycle), không sớm không muộn. Lý do:
- FCW/AEB decision phụ thuộc `Δt` chính xác cho Kalman predict — nhịp không đều → sai số tích luỹ
- Hysteresis (N-frame confirm) giả định nhịp đều — nhịp thay đổi làm ý nghĩa "N frame" không còn nhất quán về thời gian thực
- Hệ thống thật (ECU) luôn chạy theo task scheduler nhịp cố định (OS Task/RTOS), không phải vòng lặp tự do

---

## 2. Fixed-Rate Scheduler — thiết kế

```cpp
class FixedRateScheduler {
public:
    explicit FixedRateScheduler(std::chrono::milliseconds period) : period_(period) {}

    void run(std::function<void()> tick_fn) {
        auto next = std::chrono::steady_clock::now();
        while (running_) {
            next += period_;
            auto start = std::chrono::steady_clock::now();
            tick_fn();
            auto elapsed = std::chrono::steady_clock::now() - start;
            if (elapsed > period_) {
                overrun_count_++;      // log/metric — vượt budget
            }
            std::this_thread::sleep_until(next);
        }
    }
private:
    std::chrono::milliseconds period_;
    std::atomic<bool> running_{true};
    std::atomic<uint64_t> overrun_count_{0};
};
```

**Điểm mấu chốt cần hiểu (không chỉ copy code):**
- Dùng `steady_clock` (monotonic), KHÔNG dùng `system_clock` (có thể bị NTP chỉnh lùi/tiến → phá vỡ tính đơn điệu)
- `next += period_` (cộng dồn theo lịch cố định), KHÔNG phải `next = now() + period_` (sẽ tích luỹ trễ dần theo thời gian — "clock drift")
- Đếm `overrun_count_` — đây là metric quan trọng để biết pipeline có "theo kịp" nhịp thiết kế không, dùng trong report profiling

> **Ghi chú trung thực:** Linux thông thường (không phải RTOS/PREEMPT_RT patch) là **best-effort real-time**, không phải hard real-time — `sleep_until` có thể trễ vài ms do OS scheduler. TrackGuard đo và báo cáo jitter thật, không claim "hard real-time guarantee".

---

## 3. Lock-free SPSC Queue giữa các stage

Mỗi stage trong pipeline (Sensor → Sync → Transform → KF → Association → ... → Decision) chạy tuần tự trong 1 cycle, nhưng nếu muốn tách stage ra thread riêng (vd Stage 1 Sensor Ingest chạy độc lập, đẩy dữ liệu qua queue cho Stage 2+), cần **SPSC (Single-Producer Single-Consumer) lock-free ring buffer** — pattern đã chứng minh trong EventStreamCore:

```cpp
template <typename T, size_t Capacity>
class SpscQueue {
    std::array<T, Capacity> buffer_;
    std::atomic<size_t> head_{0}; // producer ghi
    std::atomic<size_t> tail_{0}; // consumer đọc
public:
    bool push(const T& item) {
        size_t h = head_.load(std::memory_order_relaxed);
        size_t next = (h + 1) % Capacity;
        if (next == tail_.load(std::memory_order_acquire)) return false; // full
        buffer_[h] = item;
        head_.store(next, std::memory_order_release);
        return true;
    }
    bool pop(T& out) {
        size_t t = tail_.load(std::memory_order_relaxed);
        if (t == head_.load(std::memory_order_acquire)) return false; // empty
        out = buffer_[t];
        tail_.store((t + 1) % Capacity, std::memory_order_release);
        return true;
    }
};
```

**Vì sao `memory_order_acquire`/`release` chứ không phải `seq_cst`:** đủ để đảm bảo happens-before giữa producer ghi dữ liệu và consumer đọc (release trên head đồng bộ với acquire khi consumer check head) — nhanh hơn `seq_cst` (không cần full memory fence) mà vẫn đúng correctness cho SPSC.

**Bounded queue là chủ đích:** nếu queue đầy (`push` trả `false`), phải có chính sách rõ ràng (drop-oldest hoặc log overrun) — KHÔNG được block vô hạn (sẽ phá vỡ tính real-time của toàn pipeline).

---

## 4. Profiling — p50/p95/p99, không chỉ trung bình

Trung bình (mean) cycle time **che giấu outlier nguy hiểm**. Một pipeline có mean=10ms nhưng p99=80ms vẫn có thể miss deadline (budget 50ms) 1% số cycle — với ADAS, 1% miss deadline tại đúng thời điểm nguy hiểm là không chấp nhận được.

```cpp
class Profiler {
    std::vector<double> samples_ms_;
public:
    void record(double ms) { samples_ms_.push_back(ms); }
    double percentile(double p) {
        auto sorted = samples_ms_;
        std::sort(sorted.begin(), sorted.end());
        size_t idx = static_cast<size_t>(p * (sorted.size() - 1));
        return sorted[idx];
    }
};
// report: p50 = percentile(0.50), p95 = percentile(0.95), p99 = percentile(0.99)
```

Test bắt buộc: chạy toàn bộ scenario library qua pipeline thật, đo p99 < budget (50ms @ 20Hz) — đây là bằng chứng "pipeline sẵn sàng real-time", không phải chỉ chạy đúng logic.

---

## 5. Sanitizers — bắt buộc chạy trong CI

| Sanitizer | Bắt lỗi gì |
|---|---|
| **ASan** (AddressSanitizer) | Buffer overflow, use-after-free, memory leak |
| **UBSan** (UndefinedBehaviorSanitizer) | Integer overflow, null pointer deref, sai kiểu ép buộc |
| **TSan** (ThreadSanitizer, nếu dùng đa luồng cho SPSC queue) | Data race |

Chạy trên build Debug riêng (`-fsanitize=address,undefined`), không chạy chung với Release (ảnh hưởng performance benchmark).

## 6. Checklist tự-kiểm tra trước khi qua Phase 12
- [ ] Tôi giải thích được vì sao dùng `steady_clock` thay vì `system_clock`
- [ ] Tôi giải thích được vì sao `next += period_` đúng hơn `next = now() + period_`
- [ ] Tôi giải thích được memory ordering `acquire`/`release` trong SPSC queue dùng để làm gì
- [ ] Tôi hiểu vì sao p99 quan trọng hơn mean cho hệ thống real-time
