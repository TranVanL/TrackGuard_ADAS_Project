# 🎤 TrackGuard Interview — Part D: System & Validation

> **Scope:** Fixed-rate real-time engine · Lock-free SPSC queue · p50/p95/p99 profiling · Testing pyramid · Tuning/holdout methodology · CI matrix/sanitizers · ARM64/ROS2 stretch.
> **Level target:** Senior ADAS/Perception/Real-time C++. Mỗi câu có **short answer** + **deep dive** + **trap/follow-up**.

Index: [Hub](05_INTERVIEW_PREP.md) · [A: Foundation](05a_INTERVIEW_FOUNDATION.md) · [B: Estimation/Tracking](05b_INTERVIEW_ESTIMATION_TRACKING.md) · [C: Fusion/Decision](05c_INTERVIEW_FUSION_DECISION.md) · **D: System/Validation** · [E: Behavioral](05e_INTERVIEW_BEHAVIORAL.md)

---

## ⚙️ SECTION D1 — Real-Time Engine (Q1–Q8)

### Q1. Describe `FixedRateScheduler` — how do you actually get a fixed 20Hz loop in C++?

**Short:** A loop that computes the next wake-up time as `next = last_wake + period` (period = 50ms for 20Hz), sleeps until `next` using `std::this_thread::sleep_until`, then runs the full pipeline cycle, and measures actual elapsed time against the budget — critically using `std::chrono::steady_clock`, never `system_clock`, since `steady_clock` is guaranteed monotonic and immune to NTP/wall-clock adjustments that would otherwise corrupt timing.

**Deep dive:** Computing `next` by *adding the period to the previous target* (not to "now") avoids drift accumulation — if one cycle overruns slightly, the next tick still targets the correct absolute schedule rather than compounding delay indefinitely.

**Trap — "What's wrong with just calling `sleep_for(50ms)` every loop?"** → `sleep_for` measures relative to *when it's called*, so any processing time within the loop pushes the next tick later — this drifts over time (each cycle's actual period slowly grows), which is exactly what `sleep_until` with an absolute, accumulating target time-point avoids.

---

### Q2. What happens when a cycle overruns its 50ms budget?

**Short:** The scheduler detects `actual_duration > budget`, logs/counts it as an overrun (visible in the p99 profiling report), and — critically — does **not** try to "catch up" by running two cycles back-to-back; it simply computes the next absolute wake time as `last_target + period` regardless, so occasional overruns don't cause runaway cycle pile-up.

**Trap — "What if overruns happen constantly, not just occasionally?"** → That's a real performance bug requiring profiling (which stage is slow — check p50/p95/p99 per-stage breakdown) rather than a scheduling fix; the scheduler's job is to *report* overruns accurately, not mask them.

---

### Q3. Why `steady_clock` and not `system_clock`, concretely — what bug would `system_clock` cause?

**Short:** `system_clock` can jump backward or forward (NTP sync, manual clock change, daylight saving in naive implementations) — if that happened mid-run, a scheduler built on `system_clock` could compute a negative or wildly large "time until next tick," causing either a busy-loop or a multi-second stall. `steady_clock` has no such guarantee of wall-clock correspondence, only guaranteed monotonic forward progress, which is the only property a scheduler actually needs.

**Trap — "Have you actually tested this, or is it just 'best practice'?"** → It's standard `<chrono>` guidance (cppreference explicitly recommends `steady_clock` for durations/timeouts), and I unit-test that the scheduler still produces monotonically increasing tick timestamps under normal operation — testing an actual system-clock-jump scenario deterministically is impractical, so this is a design-correctness argument backed by the standard's documented guarantees rather than an empirical test.

---

### Q4. Describe your lock-free SPSC (single-producer single-consumer) queue.

**Short:** A ring buffer with atomic head/tail indices — producer only ever writes `head`, consumer only ever writes `tail`, each reads the other's index with `memory_order_acquire` and writes its own with `memory_order_release`. This ordering guarantees that when the consumer observes an updated `head`, it also observes all writes the producer made to the buffer slot *before* updating `head` (release-acquire establishes a happens-before edge).

**Deep dive:** This is the same lock-free SPSC "DNA" I used in EventStreamCore — not copy-pasted code, but the same well-understood pattern applied to TrackGuard's specific need (passing `FusedTrack`/`DecisionOutput` between the real-time pipeline thread and, e.g., a logging/telemetry consumer thread without blocking the real-time thread).

**Trap — "Why acquire/release instead of just `memory_order_seq_cst` everywhere for simplicity?"** → `seq_cst` is correct but strictly more expensive (a global total order across all seq_cst operations, extra fences) — acquire/release gives the exact synchronization guarantee needed (producer-to-consumer happens-before) at lower cost; using seq_cst everywhere would work but wastes performance the real-time budget can't always afford to waste.

---

### Q5. Why lock-free at all — why not just use a `std::mutex`-protected `std::queue`?

**Short:** A mutex can be contended, and worse, can suffer priority inversion or unbounded blocking if the consumer thread is preempted while holding the lock — for a real-time producer thread that must never miss its 50ms budget, even a rare lock-contention stall is unacceptable. A well-implemented lock-free SPSC ring buffer guarantees the producer never blocks on the consumer.

**Trap — "Isn't lock-free code notoriously hard to get right — how do you know yours is correct?"** → I validate it under ThreadSanitizer (part of the CI sanitizer matrix) specifically because TSan is designed to catch exactly this class of subtle memory-ordering bug that can pass thousands of manual test runs and still be wrong — plus stress tests with many iterations under high contention to shake out races.

---

### Q6. What goes through the SPSC queue versus what's computed synchronously in the real-time thread?

**Short:** The entire estimation → association → fusion → decision pipeline runs synchronously within the real-time thread's 50ms budget (must complete before the next tick); only *side outputs* — metrics/telemetry, logging, (stretch) ROS2 publishing — go through the SPSC queue to a separate non-real-time consumer thread, so slow I/O (disk logging, network) never risks blocking the real-time cycle.

---

### Q7. How do you profile p50/p95/p99 cycle time, and why report percentiles instead of just average?

**Short:** Record each cycle's wall-clock duration (via `steady_clock` timestamps around the pipeline call) into a buffer over a full scenario run, then sort and index into the array for p50/p95/p99. Percentiles matter because a real-time system's *worst-case* behavior (p99, or better, max) is what actually risks missing the budget — an average can look perfectly fine while a long tail of occasional slow cycles quietly causes real overruns.

**Trap — "What's an acceptable p99 for a 50ms budget system?"** → I target p99 comfortably under budget (e.g., under 35-40ms) to leave margin for the sleep_until scheduling overhead itself and any minor jitter — the exact number is scenario-dependent, and I explicitly report it (not just claim "it's fast") in the validation docs.

---

### Q8. What would you profile first if p99 suddenly spiked in a new scenario?

**Short:** Add per-stage timing breakdown (not just whole-cycle) — the usual suspects are Hungarian's O(n³) cost scaling with track count in a dense scenario, or an unexpected heap allocation (e.g., `Eigen::MatrixXd` resizing) inside a hot loop — profile with per-stage timestamps first before reaching for a full profiler (`perf`), since that's usually enough to localize the stage.

---

## 🧪 SECTION D2 — Testing Pyramid & Methodology (Q9–Q16)

### Q9. Describe your testing pyramid.

**Short:** Base layer: unit tests for individual math (KF predict/update closed-form, Jacobian numerical verification, Hungarian on known cost matrices, TTC closed-form) — fast, numerous, isolate single-function correctness. Middle layer: component/integration tests (sensor→sync→transform pipeline correctness, FCW/AEB state-machine sequencing). Top layer: full end-to-end scenario tests (20+ scenarios, asserting RMSE/MOTA/MOTP/decision-sequence correctness against ground truth) — fewer, slower, validate the whole system together.

**Trap — "Isn't 150+ tests just a vanity number?"** → The number matters less than the *distribution* across the pyramid — I can point to specific tests at each layer and explain what failure mode each one is uniquely positioned to catch (e.g., the closed-form KF test catches sign/index bugs that an end-to-end RMSE test might miss entirely if compensating errors cancel out).

---

### Q10. What's a bug class that only an end-to-end scenario test would catch, not a unit test?

**Short:** Compounding small errors across stages — e.g., a slightly-off extrinsic calibration that individually passes its own unit test (small absolute error) but combines with estimation noise across many cycles to produce a track that drifts enough to fail association gating in a long scenario — no single unit test would see this interaction, only a full-pipeline run over a long enough sequence.

---

### Q11. Explain the tuning set / holdout set split and why it matters.

**Short:** 14 scenarios form the "tuning set" used while iterating on FCW/AEB thresholds and lifecycle N/K parameters; 6+ separate "holdout set" scenarios are used *only* for final reporting, never seen during tuning — this mirrors train/test-split discipline in ML, applied to classical rule-based threshold tuning, specifically to catch overfitting (thresholds that look great on the exact scenarios you tuned against but don't generalize).

**Trap — "Rule-based thresholds don't really 'overfit' the way an ML model does — why apply this methodology at all?"** → They absolutely can — if I keep nudging `ttc_warn_on` until every tuning scenario passes perfectly, I have no evidence those exact values will still work on a scenario I haven't seen; the holdout set is what actually tests generalization, and reporting *both* numbers honestly (not just cherry-picking the tuning-set score) is the actual point.

---

### Q12. What's in your holdout set, specifically, and why those scenarios?

**Short:** `holdout_hard_brake_wet_road`, `holdout_cut_in_fast`, `holdout_pedestrian_occluded`, `holdout_dense_intersection`, `holdout_long_range_approach`, `holdout_false_positive_curve` — each deliberately chosen to stress a *different* dimension (sudden deceleration, aggressive cut-in, occlusion/track-loss, dense multi-object, long-range low-confidence detection, curve-induced false-positive risk) not to be redundant with the tuning set's scenarios.

---

### Q13. What would you do if a holdout scenario failed after tuning looked great on the tuning set?

**Short:** That's exactly the signal the split is designed to surface — I'd treat it as evidence of overfit thresholds, go back and either choose more conservative/generalizable threshold values or add the failure mode explicitly to the tuning set for the *next* iteration (while still keeping a fresh holdout set going forward) rather than just hand-tuning directly against the failing holdout case (which would defeat its purpose).

---

### Q14. How do you decide what counts as "pass" for a scenario test — hardcoded pass/fail thresholds?

**Short:** Yes, explicit numeric thresholds per metric (e.g., RMSE < X meters, MOTA > Y, correct FCW/AEB trigger sequence with correct ordering) baked into gtest assertions — chosen based on what's achievable with correctly-implemented (not just "runs without crashing") estimation/association/decision logic, verified empirically during development, not arbitrary round numbers.

---

### Q15. Why property-style closed-form tests (Q9 in Part B) instead of only scenario-based tests?

**Short:** Closed-form tests give a mathematically exact expected answer (constant-velocity kinematics, circular motion, Hungarian's known-optimal assignment) — a scenario test can tell you *something* is wrong but not always *what*; closed-form tests localize the failure to a specific function/matrix, dramatically speeding up debugging.

---

### Q16. Coverage — do you track code coverage, and what target did you set?

**Short:** Yes, via `lcov`/`gcov` integrated into CI — targeting high coverage on the estimation/association/decision core logic specifically (the parts with closed-form correctness properties), while being pragmatic about coverage on glue/adapter code where correctness is better verified by integration tests than by line coverage percentage alone.

**Trap — "Isn't 100% coverage the real goal?"** → No — coverage is a diagnostic signal for *untested* code, not a proxy for correctness; a line can be covered and still wrong if the assertion checking it is weak. I use coverage to find gaps, not as a vanity metric.

---

## 🔧 SECTION D3 — CI/CD & Sanitizers (Q17–Q23)

### Q17. Describe your CI matrix.

**Short:** GitHub Actions matrix across {gcc, clang} × {Debug (ASan+UBSan+TSan enabled), Release}, plus a stretch allow-failure job cross-compiling/testing under ARM64+QEMU emulation. Debug builds run the full sanitizer-instrumented test suite; Release builds verify optimized-build correctness and performance characteristics (p50/p95/p99 profiling) without sanitizer overhead skewing timing numbers.

**Trap — "Why not run sanitizers in Release too?"** → Sanitizers add significant runtime overhead (2-20x depending on which), which would make Release-build performance profiling (the whole point of a separate Release job) meaningless — Debug+sanitizers is for *correctness*, Release is for *performance*, and conflating them defeats both purposes.

---

### Q18. What does AddressSanitizer (ASan) actually catch that a normal test run wouldn't?

**Short:** Out-of-bounds reads/writes (e.g., an off-by-one in a ring buffer or matrix index), use-after-free, double-free — memory-safety bugs that can silently corrupt state or "work by luck" in a normal run (undefined behavior doesn't always crash immediately) but are deterministically flagged the instant they occur under ASan instrumentation.

---

### Q19. What does UndefinedBehaviorSanitizer (UBSan) catch, and give a TrackGuard-relevant example.

**Short:** Signed integer overflow, invalid enum values, misaligned pointer access, division by zero — e.g., it would catch if the CTRV yaw_rate-near-zero division-by-zero guard (Part B Q10) had a bug and actually let a division by exactly zero slip through in some edge case, even if the resulting NaN didn't immediately crash the program.

---

### Q20. What does ThreadSanitizer (TSan) catch, and why is it specifically important for TrackGuard?

**Short:** Data races — concurrent unsynchronized access to shared memory. It's specifically critical for the lock-free SPSC queue (Q4/Q5) since hand-rolled atomic memory-ordering code is exactly the class of bug that can pass every functional test yet still have a genuine race that only manifests under specific timing/interleaving — TSan instruments and detects the race directly rather than relying on getting unlucky enough to observe corrupted data.

**Trap — "TSan has runtime overhead — doesn't that mean it might miss timing-dependent races differently than production would exhibit them?"** → True — TSan changes timing characteristics, so it can occasionally cause different interleavings than production (both hiding and revealing different races) — this is a known limitation; it's a strong-but-not-perfect tool, which is why the SPSC design is also reasoned about analytically (memory-ordering correctness argument in Q4), not verified by TSan alone.

---

### Q21. Why is the ARM64+QEMU CI job "allow-failure" / stretch rather than required?

**Short:** Cross-compilation + QEMU emulation introduces its own toolchain complexity (a whole separate `arm64-toolchain.cmake`) that's valuable to demonstrate (portability awareness — real ADAS/embedded systems often deploy to ARM) but isn't core to proving the estimation/tracking/decision engineering — gating the required CI matrix on it would risk blocking legitimate PRs on cross-compilation toolchain flakiness unrelated to actual logic correctness.

---

### Q22. What would you check first if the ARM64 job failed but x86 Debug/Release both passed?

**Short:** Endianness/alignment assumptions (unlikely on ARM64 which is little-endian by default, but worth ruling out), floating-point behavior differences (ARM64 NEON vs x86 SSE/AVX can have subtly different FP rounding in edge cases), or simply a QEMU emulation environment/toolchain issue unrelated to the actual code — I'd reproduce locally with the same cross-compiler+QEMU setup before assuming a real portability bug.

---

### Q23. How do you keep CI fast enough to be useful, given how many sanitizer/matrix combinations you run?

**Short:** Unit + component tests run in every matrix cell (fast, so no cost concern); the full 20+ scenario end-to-end suite (slower) still comfortably fits within CI time budgets at this project's scale — if it ever became a bottleneck, the standard mitigation is parallelizing scenario runs across CI job shards, which I'd add if profiling showed CI wall-time becoming a real friction point.

---

## 🔌 SECTION D4 — Portability & ROS2 Interop (Stretch) (Q24–Q28)

### Q24. What does the ROS2 publisher wrapper actually do?

**Short:** Subscribes to the real-time pipeline's output (`FusedTrack`/`DecisionOutput`) via the SPSC queue's consumer side, converts them into ROS2 message types, and publishes on standard topics — purely additive, config-gated (`enable_ros2_publisher`), with zero impact on the core pipeline when disabled.

---

### Q25. Why keep ROS2 entirely optional instead of building the whole project on top of ROS2 from the start?

**Short:** Coupling the core pipeline to ROS2 would mean every unit/integration test needs a ROS2 environment to even compile/run — a significant CI and portability cost for a project whose actual engineering value (estimation, tracking, fusion, decision) is middleware-agnostic; keeping ROS2 as an optional adapter at the edge demonstrates the same "swap the edges, keep the core stable" design philosophy used throughout (Foundation Q4/Q8).

---

### Q26. What would a real robotics team actually want to see from this ROS2 integration?

**Short:** Correct message type mapping, proper QoS settings for a real-time-ish topic (e.g., best-effort vs reliable trade-off for track messages), and ideally a launch file — I'd frame the current stretch implementation honestly as "demonstrates the integration pattern," not a production-grade ROS2 node with full QoS tuning, unless that work was actually done.

---

### Q27. Cross-compiling for ARM64 — what class of bug is this actually trying to surface?

**Short:** Build-system portability issues (hardcoded x86-specific flags, assumptions about SIMD availability, toolchain-specific behavior) rather than algorithmic bugs — it's a signal of "this codebase isn't accidentally tied to one architecture," relevant for embedded/robotics roles where ARM targets (Jetson, Raspberry Pi-class boards) are common deployment targets.

---

### Q28. If asked to actually deploy TrackGuard on an embedded ARM board tomorrow, what's missing?

**Short:** Real sensor drivers (currently synthetic/KITTI-replay only), power/thermal profiling on the actual target hardware, a real-time OS or PREEMPT_RT kernel consideration for the scheduler's timing guarantees, and hardware-in-the-loop testing — I'd frame the current ARM64+QEMU CI job as validating *build portability*, explicitly not validating *runtime real-time guarantees on real hardware*, which is a materially different (and harder) claim.

---

## 🧰 SECTION D5 — Coding Standard & Static Analysis (Q29–Q34)

### Q29. Why write your own 24-rule coding standard instead of just following MISRA C++ or AUTOSAR C++14 directly?

**Short:** Full MISRA C++:2008/AUTOSAR C++14 have 300+ rules, many of which (banning exceptions entirely, restricting STL) don't fit a modern C++17 project using Eigen/gtest and aren't realistically enforceable solo in 60 days. [08_CODING_STANDARDS_STATIC_ANALYSIS.md](08_CODING_STANDARDS_STATIC_ANALYSIS.md) selects ~24 rules chosen for three criteria: they prevent bug classes I've actually encountered in this design, they're tool-enforceable (not "honor system"), and they don't fight idiomatic modern C++.

**Trap — "So you're not MISRA compliant?"** → Correct, and I say so explicitly — it's a *MISRA/AUTOSAR-inspired subset*, not a compliance claim; claiming full compliance without a compliance audit would be dishonest.

---

### Q30. Give a concrete example of a rule that came from a real bug, not theory.

**Short:** TGCS-05 — mandatory epsilon-guard before any division that could go near-zero — came directly from the CTRV EKF's `yaw_rate` term in the Jacobian; dividing by `yaw_rate` when the vehicle drives near-straight produces a near-singular/undefined term unless explicitly guarded with a small-angle branch. I hit this designing the Jacobian, not from reading a standard.

---

### Q31. What's your static analysis toolchain and what does each tool actually catch that the others don't?

**Short:** clang-tidy (style + bug-pattern + modernize checks, required per-PR), cppcheck (`--enable=all --inconclusive`, catches some null-deref/buffer patterns clang-tidy misses, required per-PR), Coverity Scan (deep interprocedural analysis, especially concurrency/resource-leak — runs periodically, not per-PR, due to scan-count limits, with a 1-week defect-triage SLA), and IWYU (include hygiene, optional/non-blocking). Runtime sanitizers (ASan/UBSan/TSan) complement these by catching what's only visible at runtime.

**Trap — "Why not run Coverity on every PR too?"** → Coverity's free/OSS tier has a limited scan quota per period, so it's used as a periodic deep-dive gate rather than burning quota on every commit — a real resource-management trade-off, not laziness.

---

### Q32. What's your policy on `// NOLINT` suppressions?

**Short:** Every suppression requires a comment explaining *why* the finding is a false positive or an accepted, reviewed trade-off — an unexplained suppression is treated as a code smell in review, because unexplained suppressions are exactly how static analysis value quietly erodes over time on real teams.

---

### Q33. How does the coding standard connect back to the requirements traceability matrix?

**Short:** TGCS-23 requires every public API to carry a Doxygen comment including `@satisfies{FR-xxx}` where applicable — so the coding standard is literally what *produces* the traceability links consumed by [06_REQUIREMENTS_TRACEABILITY.md](06_REQUIREMENTS_TRACEABILITY.md); they're not two separate efforts, one enforces the other.

---

### Q34. What would you do differently if this were a real team of 5 engineers instead of solo?

**Short:** Add mandatory peer review (not self-review) with a second required approver for anything touching estimation/decision logic, rotate static-analysis-finding ownership instead of one person triaging everything, and add a lightweight design-review step before implementing any new Safety-Goal-linked feature — the current process is deliberately scaled down for solo execution while still preserving the parts (traceability, static analysis gate, coding standard) that matter most for handing this code to someone else later.

---

## 🔗 Related design docs

[knowledge/03f_REALTIME_CPP_LINUX.md](knowledge/03f_REALTIME_CPP_LINUX.md) · [knowledge/03g_TESTING_VALIDATION.md](knowledge/03g_TESTING_VALIDATION.md) · [01_ARCHITECTURE.md](01_ARCHITECTURE.md) · [08_CODING_STANDARDS_STATIC_ANALYSIS.md](08_CODING_STANDARDS_STATIC_ANALYSIS.md)

Next: [Part E — Behavioral](05e_INTERVIEW_BEHAVIORAL.md)
