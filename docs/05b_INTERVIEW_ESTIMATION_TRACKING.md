# 🎤 TrackGuard Interview — Part B: Estimation & Tracking

> **Scope:** Kalman Filter (CV) · Extended Kalman Filter (CTRV) · UKF/IMM (stretch) · Hungarian data association + gating · Track lifecycle · MOTA/MOTP.
> **Level target:** Senior ADAS/Perception C++. Mỗi câu có **short answer** + **deep dive** + **trap/follow-up**.

Index: [Hub](05_INTERVIEW_PREP.md) · [A: Foundation](05a_INTERVIEW_FOUNDATION.md) · **B: Estimation/Tracking** · [C: Fusion/Decision](05c_INTERVIEW_FUSION_DECISION.md) · [D: System/Validation](05d_INTERVIEW_SYSTEM_VALIDATION.md) · [E: Behavioral](05e_INTERVIEW_BEHAVIORAL.md)

---

## 📐 SECTION B1 — Kalman Filter Core (CV Model) (Q1–Q8)

### Q1. Explain the Kalman Filter predict step in your CV model.

**Short:** State `x = [x, y, vx, vy]`. Predict: `x' = F·x`, `P' = F·P·Fᵀ + Q`, where `F` encodes constant-velocity motion (`x += vx·dt`, `y += vy·dt`) and `Q` is process noise covariance representing our uncertainty about how much the object might accelerate unexpectedly during `dt`.

**Deep dive:** `Q` isn't just "some noise" — it's typically parameterized by an assumed acceleration variance `σ_a²`, injected into the position/velocity blocks via the discretized white-noise-acceleration model (`Q ∝ [[dt⁴/4, dt³/2],[dt³/2, dt²]] · σ_a²` per axis). Tuning `σ_a²` too small makes the filter overconfident and slow to react to real acceleration; too large makes it noisy/jittery.

**Trap — "What happens if `dt` varies between calls (variable frame rate)?"** → `F` and `Q` are recomputed per-call using the actual `dt` measured from the replay clock, not assumed constant — this is required because KITTI/synthetic scenarios don't guarantee perfectly uniform timestamps.

---

### Q2. Walk through the update step and why you use Joseph form for the covariance update.

**Short:** `y = z - H·x` (innovation), `S = H·P·Hᵀ + R`, `K = P·Hᵀ·S⁻¹` (Kalman gain), `x += K·y`. For `P`, instead of the textbook `P = (I-K·H)·P`, I use the **Joseph form**: `P = (I-KH)·P·(I-KH)ᵀ + K·R·Kᵀ`. Joseph form guarantees `P` stays symmetric and positive semi-definite even under floating-point rounding error or a suboptimal `K`, whereas the simplified form can drift to non-PSD over many iterations.

**Deep dive:** I assert `(P - Pᵀ).norm() < eps` and check eigenvalues ≥ 0 in debug builds after every update — this caught a real bug during development where a typo in `H` silently produced a non-PSD `P` after ~200 update cycles, which the simplified form would have masked until much later (a NaN or Cholesky failure downstream, hard to trace back).

**Trap — "Doesn't Joseph form cost more computation?"** → Yes, roughly 2x the matrix multiplies of the naive form — but for a 4x4 (CV) or 5x5 (CTRV) state, this is negligible compared to the 20Hz/50ms cycle budget; numerical stability is worth far more than the microseconds saved.

---

### Q3. How do you initialize a new track's state and covariance?

**Short:** Position initialized directly from the first detection; velocity initialized to zero (or estimated from the first two detections if available); `P` initialized with small position uncertainty (matches sensor's known noise) but **large velocity uncertainty**, since we have no information about velocity yet.

**Trap — "Why not just set P to a huge uniform value everywhere?"** → Because position uncertainty *is* actually well-known from the sensor's noise model — overstating it would make the filter distrust the first few good detections unnecessarily, slowing convergence.

---

### Q4. What's the closed-form test you use to validate `predict()`?

**Short:** For an object moving at constant velocity with zero process noise, running `predict()` N times must produce a position that exactly matches the closed-form kinematic equation `x0 + v*t` (within floating-point epsilon). This isolates the `F` matrix construction from any noise/tuning concerns.

**Trap — "What if someone accidentally swapped the x/y velocity indices in `F`?"** → This exact test would fail immediately, since predicted x/y would diverge from the closed-form values in a very visible way — that's precisely the class of bug the test is designed to catch.

---

### Q5. How do you know your KF is "working" beyond just compiling and running?

**Short:** Three test tiers: (1) closed-form kinematic checks for `predict()`, (2) convergence checks for `update()` — repeated noisy measurements of a static/simple-motion object should make `P` shrink over time, (3) end-to-end RMSE against ground truth on a real scenario, which must stay below a defined threshold.

**Trap — "What if RMSE is low but P is overconfident (too small)?"** → I also check *consistency*: whether the actual estimation error stays within the filter's reported ±1σ/±2σ bounds most of the time (a basic NEES-style sanity check) — RMSE alone doesn't catch an overconfident filter that happens to get lucky on one scenario.

---

### Q6. What is `R` (measurement noise covariance) and how did you set it?

**Short:** `R` reflects the sensor's own measurement noise — for radar, derived directly from the injected synthetic noise parameters (range ±0.1m, azimuth ±0.5°, converted to Cartesian variance); for camera, a slightly looser estimate since bbox-center-to-position conversion adds its own imprecision.

**Trap — "What if `R` is mis-tuned — too small?"** → The filter over-trusts noisy measurements, producing jittery state estimates that overreact to sensor noise — I demonstrate this as a deliberate "what if I break tuning" experiment in my validation notes, showing RMSE degrading when `R` is artificially shrunk.

---

### Q7. Your `KalmanFilterCV` and `ExtendedKalmanFilterCTRV` — do they share code?

**Short:** They share the same interface contract (expose `x`, `P`, `predict(dt)`, `update(z)`) so `TrackManager`/`TrackFuser` can treat any estimator polymorphically, but the internal `F`/Jacobian computation differs because CV is linear (no Jacobian needed) and CTRV is non-linear (needs `computeJacobian`).

**Trap — "Could you template this instead of virtual dispatch?"** → Yes — templating would avoid vtable overhead, a valid optimization if profiling showed estimation dispatch as a bottleneck; I chose virtual dispatch for clarity/swappability first, with static dispatch as a documented future optimization if the profiler ever flags it.

---

### Q8. What's the single most likely bug source in a hand-rolled KF, and how do you guard against it?

**Short:** Sign/index errors in `F`, `H`, or the Jacobian — extremely easy to get subtly wrong and hard to notice because the filter still "runs," just converges to wrong or unstable estimates. Guard: closed-form unit tests for every matrix (predict without noise = exact kinematics; Jacobian numerically verified against finite-difference approximation).

**Trap — "How do you numerically verify a Jacobian?"** → Compute `f(x+ε) - f(x-ε)) / 2ε` per state dimension and compare to the analytical Jacobian — if they disagree beyond floating-point tolerance, the analytical derivation has a bug.

---

## 🔄 SECTION B2 — Extended Kalman Filter (CTRV Model) (Q9–Q14)

### Q9. Why CTRV instead of CV for turning objects?

**Short:** CV assumes constant velocity in a straight line — for a vehicle turning at a roundabout or curve, CV consistently under/overshoots the true position because it can't represent curvature. CTRV's state `[x, y, v, yaw, yaw_rate]` explicitly models constant speed *and* constant turn rate, correctly representing circular-arc motion.

**Trap — "So why not always use CTRV, even for straight-line objects?"** → CTRV is non-linear, requires Jacobian linearization (introduces approximation error) and is more expensive; for objects that are actually going straight, CV is simpler, cheaper, and just as accurate — hence the CV vs CTRV comparison report and (stretch) IMM to blend both.

---

### Q10. What's the special-case handling when `yaw_rate ≈ 0` in CTRV?

**Short:** The CTRV motion equations divide by `yaw_rate` (turning radius term), which blows up as `yaw_rate → 0`. When `|yaw_rate| < ε`, I switch to the CV-equivalent limiting form (straight-line motion) to avoid division by near-zero, both in the prediction equations and the Jacobian.

**Deep dive:** This is a textbook numerical-stability trap in CTRV implementations — I explicitly unit-test this boundary (yaw_rate = 0, yaw_rate = 1e-8, yaw_rate flipping sign) since it's exactly the kind of edge case that "works in the happy path demo" but crashes or diverges on real data.

**Trap — "What threshold did you pick for ε and why?"** → A small value like `1e-4 rad/s`, chosen because it's well below any physically meaningful turn rate but well above floating-point noise — documented as a tunable constant, not hardcoded magic.

---

### Q11. Derive (verbally) the CTRV Jacobian's role in the EKF update.

**Short:** The Jacobian `Fj` is the local linear approximation of the non-linear motion function `f(x)` around the current state estimate — it replaces the constant `F` matrix used in linear KF, letting the same predict/update math (`P' = Fj·P·Fjᵀ + Q`) apply to a non-linear system.

**Trap — "What's the cost of linearizing at the wrong point?"** → If the true state is far from the linearization point (large yaw_rate, big `dt`, or high uncertainty), the linear approximation error grows — this is exactly the motivation for UKF, which avoids linearization entirely (see §B4).

---

### Q12. How do you test CTRV correctness beyond "it compiles and runs on a curve"?

**Short:** A closed-form circular-motion test: an object moving at constant speed and constant yaw_rate traces an exact circle; running `predict()` N steps must match the closed-form circle equation within floating-point tolerance, isolating the CTRV motion model from noise/tuning effects — same philosophy as the CV closed-form test.

**Trap — "What if yaw wraps around 2π during the test?"** → I explicitly normalize yaw to `[-π, π]` after every predict/update and test that wraparound doesn't cause discontinuities in state or covariance.

---

### Q13. What numerical stability tests do you run beyond the happy path?

**Short:** Near-singular Jacobian cases (yaw_rate flipping sign rapidly), long scenarios (1000+ steps) asserting `P` stays PSD throughout (not just at the start), and large-noise-injection scenarios to check the filter doesn't diverge.

**Trap — "What would 'divergence' look like in your tests, concretely?"** → `P`'s trace growing unboundedly instead of converging, or eigenvalues becoming negative (failing the PSD assert) — both are asserted as explicit failure conditions, not just "does it crash."

---

### Q14. You report CV vs CTRV RMSE comparison — what did you learn from it?

**Short:** On a curved scenario, CTRV consistently achieves lower RMSE than CV (as expected, since CV can't represent curvature) — the comparison report quantifies *how much* better, which is the kind of evidence-based claim I'd want to defend in an interview rather than a hand-wavy "CTRV is better."

**Trap — "What if CV actually won on some scenario?"** → That would be expected on a straight-line scenario, where CTRV's extra flexibility adds noise without benefit — a case I'd document rather than hide, since it demonstrates I understand *when* each model applies rather than blindly favoring the more complex one.

---

## 🧮 SECTION B3 — Numerical Stability & Tuning (Q15–Q18)

### Q15. How do you tune `Q` and `R` in practice, beyond guessing?

**Short:** Start from physically-motivated priors (sensor spec noise for `R`, assumed max acceleration for `Q`), then use the **tuning scenario set** to adjust and observe RMSE/consistency — never tuned against the holdout set, to avoid overfitting thresholds to specific scenarios (same train/test-split discipline used for FCW/AEB thresholds).

**Trap — "Isn't manual tuning unscientific?"** → It's the standard approach for classical estimation absent a full system-ID pipeline; the discipline is in *separating tuning data from evaluation data*, not in avoiding manual tuning entirely — which is realistic industry practice for Kalman-based systems.

---

### Q16. What happens if `P` becomes ill-conditioned (huge condition number)?

**Short:** `S = H·P·Hᵀ + R` inversion becomes numerically unstable, producing an unreliable Kalman gain. Guard: debug-build PSD/symmetry asserts catch this early; in practice, Joseph form plus reasonable `Q`/`R` priors prevent it from occurring in the test scenario library.

**Trap — "What would you do in production if this happened despite Joseph form?"** → Add covariance inflation/regularization (adding a small diagonal term) as a last-resort safeguard, and log/alert on the condition rather than silently continuing with garbage output — documented as a known production concern this portfolio doesn't fully solve.

---

### Q17. Eigen is doing your linear algebra — any gotchas using it for a Kalman filter?

**Short:** Fixed-size vs dynamic-size `Eigen::VectorXd`/`MatrixXd` — I use dynamic sizing for flexibility across CV/CTRV/UKF state dimensions, aware this costs a small heap-allocation overhead per operation versus fixed-size `Matrix4d` (acceptable at 20Hz, would reconsider at higher rates).

**Trap — "Why not just use fixed-size Eigen types for performance?"** → Considered it — dynamic sizing was chosen for code reuse across CV(4)/CTRV(5)/UKF(5) state dimensions without duplicating the estimator interface; if profiling showed this as a bottleneck I'd template on state size instead.

---

### Q18. How do you make sure PSD/symmetry asserts don't add overhead in Release builds?

**Short:** Asserts are wrapped in a debug-only macro (compiled out via `NDEBUG` or a custom `TRACKGUARD_DEBUG_ASSERT`), so Release builds pay zero cost for the safety checks that catch bugs during development and CI (which always runs the Debug+sanitizer matrix).

---

## 🧩 SECTION B4 — UKF & IMM (Stretch) (Q19–Q24)

### Q19. What is the Unscented Kalman Filter and why would you add it as a stretch goal?

**Short:** UKF avoids linearizing `f()`/`h()` at all — instead it deterministically samples 2n+1 "sigma points" that capture the mean/covariance of the current state distribution, passes each through the *true* non-linear function, then recovers the transformed mean/covariance from the transformed points. It's accurate to 2nd-order Taylor expansion for any non-linearity, versus EKF's 1st-order accuracy.

**Deep dive:** The value of adding it as stretch (not core) is demonstrating I understand *when* the extra complexity is justified — strong non-linearity (sharp turns) — versus when EKF is "good enough."

**Trap — "Isn't UKF strictly better than EKF then? Why not always use it?"** → It costs `2n+1` function evaluations per predict/update instead of one plus a Jacobian — for a tight real-time budget with many tracks, that's a real cost; also, UKF doesn't need a differentiable `f()`, which matters more for genuinely nasty non-linearities than for CTRV, which already has a clean analytical Jacobian.

---

### Q20. Explain sigma point generation.

**Short:** For state dimension `n`, generate `χ_0 = x̂`, and `χ_i = x̂ ± (√((n+λ)P))_i` for `i=1..n`, where the square root is a Cholesky decomposition of `(n+λ)P`. Each point gets mean-weight `W_i^(m)` and covariance-weight `W_i^(c)`, with `λ` a scaling parameter controlling how spread out the points are.

**Trap — "What if `P` isn't PSD when you try to Cholesky-decompose it?"** → Cholesky fails (`llt().info() == NumericalIssue`) — this is actually a *useful* early-warning signal that something upstream (an update step) produced an invalid covariance, similar in spirit to the EKF's PSD assert.

---

### Q21. How does the predict step differ from EKF once you have sigma points?

**Short:** Instead of `x' = F·x` with a linear/linearized `F`, each of the `2n+1` sigma points is passed individually through the *actual* non-linear CTRV motion function, then the predicted mean and covariance are recovered as weighted sums over the transformed points — no Jacobian is computed at all.

**Trap — "Where does process noise `Q` enter in UKF?"** → Added directly to the recovered predicted covariance (`P_k⁻ = Σ Wᵢᶜ(...)ᵀ + Q`), same conceptual role as in EKF, just applied after the unscented transform instead of after a linear propagation.

---

### Q22. What's IMM and what problem does it solve that a single CTRV filter doesn't?

**Short:** A single fixed motion model (CV or CTRV) is wrong at least part of the time for any real vehicle — sometimes going straight, sometimes turning. IMM runs multiple filters (e.g., CV and CTRV) in parallel, mixes their state estimates according to a Markov model-transition probability, and blends the final output by each model's *current* probability of being correct — giving smoother, more adaptive tracking than hard-switching between models.

**Trap — "How is IMM different from just running CV and CTRV and picking whichever fits better after the fact?"** → IMM mixes states *before* each filter updates (not just at output time), so each filter's estimate benefits from the other's information every cycle, and model probabilities update continuously via measurement likelihood rather than a post-hoc decision rule.

---

### Q23. Walk through IMM's four steps.

**Short:** (1) **Mixing** — combine each filter's prior state into a mixed input for every other filter, weighted by transition probabilities and current model probabilities; (2) **Filtering** — each filter runs its own predict+update independently on its mixed input; (3) **Model probability update** — update each model's probability using how "likely" the new measurement was under that model; (4) **Combination** — the final output state is the probability-weighted average across all filters.

**Trap — "What's in the transition probability matrix and how would you set it?"** → A small Markov matrix (e.g., `[[0.95, 0.05],[0.10, 0.90]]`) representing "probability of staying in CV" vs "switching to CTRV" — set based on how frequently we expect real vehicles to change driving mode, tuned on the tuning scenario set, never the holdout set.

---

### Q24. If UKF/IMM never get built (stretch not reached), does that hurt the project?

**Short:** No — they're explicitly scoped as stretch, gated behind all `v1.0` core checkpoints being green first. The core estimation story (KF + EKF, closed-form tested, numerically stable) is a complete, defensible narrative on its own; UKF/IMM are additive proof of deeper estimation-theory range, not a gap if time runs out.

---

## 🔗 SECTION B5 — Data Association (Hungarian + Gating) (Q25–Q31)

### Q25. Explain the Hungarian algorithm's role in your pipeline.

**Short:** After predicting all existing tracks and receiving new detections, I build a cost matrix (rows=tracks, cols=detections) where `cost(i,j)` = Mahalanobis distance (or `1-IoU`), then run the Hungarian algorithm to find the globally optimal one-to-one assignment minimizing total cost — O(n³) but exact, unlike greedy nearest-neighbor which can produce suboptimal or inconsistent assignments.

**Trap — "Why not greedy nearest-neighbor — it's much simpler?"** → Greedy can make a locally-good but globally-bad assignment (grab the closest match first, forcing a worse match on a later pair) — with crossing/nearby objects (my multi-object dense scenario), this causes ID switches that hurt MOTA; Hungarian avoids this by construction.

---

### Q26. What is Mahalanobis distance and why not just use Euclidean distance for gating?

**Short:** Mahalanobis distance is `d² = (z - Hx)ᵀS⁻¹(z - Hx)` — it normalizes the raw positional difference by the *combined uncertainty* (`S`) of the prediction and measurement, so a match is judged relative to how confident we are, not just raw pixel/meter distance. Euclidean distance would unfairly penalize a track with high uncertainty (early in its life) even for a legitimately correct match.

**Trap — "What chi-square threshold did you use and why?"** → For 2D position (df=2) at 95% confidence, χ² ≈ 5.99 — a detection whose Mahalanobis distance squared exceeds this is gated out as implausible; this threshold is a textbook value pulled from the chi-square distribution table, not an arbitrary tuned constant.

---

### Q27. Why gate *before* running Hungarian, instead of just letting Hungarian pick the best overall assignment?

**Short:** Gating removes track-detection pairs that are statistically implausible *before* building the cost matrix — this both shrinks the matrix (performance) and prevents Hungarian from ever being forced to assign a detection to a track it has no business being matched to, just because it happened to be the "least bad" option in a degenerate case (e.g., every real match already taken).

---

### Q28. What's IoU-based cost and when would you prefer it over Mahalanobis?

**Short:** `1 - IoU(track_bbox, detection_bbox)` — useful when both track and detection have bounding-box representations (e.g., camera-only association) and overlap is a more direct/robust similarity signal than centroid distance, especially for close-range large objects where centroid distance can be misleading relative to box overlap.

**Trap — "Which do you actually use in TrackGuard?"** → Mahalanobis primarily (since the pipeline's canonical representation is point/state-based across all sensor types), with IoU noted as an available alternative for camera-only association paths — documented, not necessarily the default.

---

### Q29. What's the complexity of your association stage and does it matter at 20Hz?

**Short:** Hungarian is O(n³) in the number of tracks/detections; for realistic ADAS scenes (tens of tracks, not thousands), this is negligible relative to the 50ms cycle budget — profiled explicitly as part of the p50/p95/p99 cycle-time report to confirm it never becomes the bottleneck.

**Trap — "What if you had hundreds of tracks (e.g., a dense urban scene)?"** → O(n³) would start to matter; a production system would likely bucket by spatial region first (reducing effective n per Hungarian call) — noted as a scaling consideration outside this project's scope (highway/L2 ADAS scenarios, not dense urban).

---

### Q30. How do you validate Hungarian is actually implemented correctly?

**Short:** A textbook cost-matrix test with a known optimal assignment (from a reference source) — the assigner must reproduce that exact optimal assignment, isolating algorithm correctness from any pipeline-specific cost function issues.

---

### Q31. What's the trade-off of choosing Hungarian (optimal) vs a simpler heuristic given your 60-day timeline?

**Short:** Hungarian is a well-understood, well-tested classical algorithm — implementing it correctly (or using a compact reference implementation) costs less engineering time than debugging subtle correctness issues in a custom greedy heuristic, while also being the theoretically correct choice, so there was no real trade-off to make here.

---

## 🔁 SECTION B6 — Track Lifecycle & MOTA/MOTP (Q32–Q38)

### Q32. Describe the track lifecycle state machine.

**Short:** `Tentative` (just created, unconfirmed) → `Confirmed` after N consecutive matched frames; `Confirmed` → `Coasting` (predict-only, no update) when unmatched, for up to K frames; `Coasting` → `Confirmed` again if re-matched; `Coasting` → `Deleted` if unmatched beyond K frames; `Tentative` → `Deleted` immediately if unmatched (never confirmed).

**Trap — "Why not delete a track immediately on the first missed frame?"** → That would break tracking through brief real-world occlusion (a car passing behind a sign for 2-3 frames) — `Coasting` lets the Kalman filter's predict-only step bridge short gaps while preserving the track ID.

---

### Q33. How did you choose N (confirm threshold) and K (max coast frames)?

**Short:** Tuned on the tuning scenario set — N small enough to confirm real objects quickly (avoid latency), large enough to reject spurious one-frame false detections; K large enough to survive the occlusion scenarios I designed, small enough to avoid holding "ghost" tracks too long. Never tuned against the holdout set.

**Trap — "What would happen with N=1?"** → Every noisy false-positive detection would instantly become a "confirmed" track — that's exactly the failure mode N is designed to prevent, verified by a dedicated false-positive-trap scenario.

---

### Q34. What is MOTA and what does it actually measure?

**Short:** Multi-Object Tracking Accuracy combines three error types over the whole sequence — missed detections (false negatives), false positives, and ID switches — into one score: `MOTA = 1 - (FN + FP + IDSW) / GT`, where GT is total ground-truth object-frames. It's the standard metric from the MOT research/benchmark community, not something I invented.

**Trap — "Can MOTA be negative?"** → Yes, if error counts exceed the number of ground-truth objects — this is a known property of the metric, not a bug in my calculator.

---

### Q35. What is MOTP and how does it differ from MOTA?

**Short:** MOTP (Precision) measures the average localization error (e.g., positional distance) of *correctly matched* detections — it answers "how precise is my tracking when it does match," whereas MOTA answers "how often did I match/miss/misidentify at all." A tracker can have high MOTA (few misses/switches) but mediocre MOTP (matches are found but imprecise), or vice versa.

---

### Q36. Why do ID switches matter so much for MOTA, beyond just "it looks bad in the video"?

**Short:** An ID switch means a downstream consumer (e.g., FCW/AEB) would perceive a *new* threat appearing out of nowhere, potentially resetting hysteresis/confirm counters — an ID switch on a real threat could delay a warning that should have already been building confidence over several frames.

**Trap — "How do you detect an ID switch in your metrics calculator, precisely?"** → Compare the mapping between predicted-track-ID and ground-truth-object-ID frame-to-frame; if a ground-truth object's matched track ID changes without the object actually disappearing/reappearing, that's counted as a switch.

---

### Q37. Your multi-object dense scenario is your key stress test — what specifically does it validate?

**Short:** 5+ objects, some crossing paths, all present simultaneously — this is the scenario most likely to expose Hungarian/gating bugs (crossing objects create ambiguous, close cost-matrix entries) and track-lifecycle bugs (near-simultaneous occlusion of multiple objects). Passing it with low ID-switch count is the strongest evidence the association+lifecycle layers work under realistic stress, not just simple 1-2 object cases.

---

### Q38. If your MOTA/MOTP numbers aren't as high as a state-of-the-art paper's, how do you defend that in an interview?

**Short:** I'd frame it honestly: this is a from-scratch classical pipeline on a scoped 60-day timeline, evaluated with the correct methodology (proper metric, tuning/holdout split) — the value being demonstrated is that I understand *how* to measure tracking quality correctly and can reason about *why* a number is what it is, not that I'm claiming to beat published research benchmarks.

---

## 🔗 Related design docs

[knowledge/03b_KALMAN_FILTER_ESTIMATION.md](knowledge/03b_KALMAN_FILTER_ESTIMATION.md) · [knowledge/03h_ADVANCED_ESTIMATION_UKF_IMM.md](knowledge/03h_ADVANCED_ESTIMATION_UKF_IMM.md) · [knowledge/03c_DATA_ASSOCIATION_TRACKING.md](knowledge/03c_DATA_ASSOCIATION_TRACKING.md)

Next: [Part C — Fusion/Decision](05c_INTERVIEW_FUSION_DECISION.md)
