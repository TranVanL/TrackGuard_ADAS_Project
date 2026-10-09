# 🎤 TrackGuard Interview — Part E: Behavioral

> **Scope:** Level defense · System design whiteboard · Proactive honesty framing · STAR-format stories · Demo script walkthrough · Curveball questions · Reverse questions to ask interviewers.
> **Level target:** Senior ADAS/Perception/Real-time C++. Mỗi câu có **short answer** + **deep dive** + **trap/follow-up** (behavioral trap = how to avoid sounding rehearsed/dishonest).

Index: [Hub](05_INTERVIEW_PREP.md) · [A: Foundation](05a_INTERVIEW_FOUNDATION.md) · [B: Estimation/Tracking](05b_INTERVIEW_ESTIMATION_TRACKING.md) · [C: Fusion/Decision](05c_INTERVIEW_FUSION_DECISION.md) · [D: System/Validation](05d_INTERVIEW_SYSTEM_VALIDATION.md) · **E: Behavioral**

---

## 🎯 SECTION E1 — Level Defense (Q1–Q6)

### Q1. This is a solo portfolio project — how do you justify claiming senior-level engineering from it?

**Short:** Seniority isn't claimed from the *label* "TrackGuard" — it's demonstrated through specific decisions: choosing Joseph-form covariance updates over the textbook simplified form after understanding *why* it matters numerically, designing a tuning/holdout split before being told to, separating real-time-critical code from side-effect I/O via a lock-free queue, and being explicit about every simplification's limits rather than overclaiming. I'd point to concrete artifacts (the CV-vs-CTRV comparison report, the PSD-assert-catching-a-real-bug story) as evidence, not adjectives.

**Trap — "But you didn't work with a team or real deadlines/stakeholders on this."** → True, and I wouldn't claim otherwise — I'd redirect to what *is* transferable: the technical judgment behind each design decision, and separately discuss team/stakeholder experience from my actual professional history where relevant.

---

### Q2. What's the single hardest technical decision you made on this project, and how did you decide?

**Short:** Choosing late (track-level) fusion over early (detection-level) fusion — I considered both, prototyped reasoning about failure modes (Part C, Q1-Q2), and chose late fusion specifically because it's independently testable and degrades gracefully, even though early fusion has a theoretical accuracy ceiling advantage. The decision came down to: which failure mode is more acceptable for a 60-day solo project where I can't extensively validate exotic edge cases — an undetected calibration bug corrupting both sensors, or a slightly lower theoretical fusion ceiling with independently-verifiable sensor pipelines.

**Trap — "That sounds like you avoided the harder option."** → I'd frame it as risk-appropriate engineering judgment, not avoidance — and note I documented early fusion as a legitimate alternative with its own trade-offs, rather than pretending late fusion is unambiguously superior.

---

### Q3. How do you know your covariance-intersection weighting (ω selection) is actually correct, not just "runs without crashing"?

**Short:** I validate it the same way I validate everything else — closed-form/known-case tests (two sources with known covariances should produce a fused covariance that's provably no larger than either input alone, a mathematical property of CI) plus the CV/CTRV-comparison-style empirical RMSE check against ground truth (Part C, Q7). I don't claim correctness from "it looks reasonable," I claim it from specific checkable properties.

---

### Q4. If a staff/principal engineer reviewed this code, what's the first thing they'd push back on?

**Short:** Likely the grid-search approach to CI's `ω` parameter (Part C Q3) — a staff engineer would probably ask why I didn't derive or use a closed-form/golden-section search, and I'd have a ready answer: at this scale (infrequent per-track fusion calls, not a hot inner loop), the grid search's simplicity and correctness-transparency outweighed the marginal performance win of a more elegant search, but I'd happily implement golden-section search if profiling ever showed it mattered.

**Trap — "Isn't that just an excuse for not doing the harder thing?"** → No — it's an explicit engineering trade-off with a stated condition for revisiting it (profiling evidence), which is different from either avoiding the harder work or doing unnecessary optimization without justification.

---

### Q5. What would you do differently if you rebuilt this project from scratch?

**Short:** I'd design the tuning/holdout scenario split and the `RunConfig`/determinism boundary *first*, before writing any estimation code — I added both mid-project as I realized their importance, which meant some earlier tests needed retrofitting for seeded reproducibility. Building determinism-first is a lesson I'd carry into any future project on day one.

---

### Q6. How do you handle being asked a question about a part of the system you didn't build deeply (e.g., real hardware deployment)?

**Short:** State plainly what's actually implemented and tested versus what's a documented stretch/future concern (Part D, Q28 is a good example) — I never extrapolate confidence beyond what I've verified, and I'd rather say "I haven't validated that, here's what I'd expect and how I'd test it" than bluff.

---

## 🧱 SECTION E2 — System Design Whiteboard (Q7–Q12)

### Q7. "Design a multi-sensor fusion tracking system from scratch" — how do you structure your whiteboard answer?

**Short:** Start by clarifying requirements (sensors involved, real-time constraints, accuracy targets, failure-mode tolerance) *before* drawing boxes — then lay out the same boundary-driven pipeline shape as TrackGuard's (ingest → sync → transform → estimate → associate → track → fuse → decide), explaining each cut point's purpose, then discuss trade-offs at each stage (which model, which fusion strategy) grounded in the actual reasoning I used in TrackGuard.

**Trap — "The interviewer changes a requirement mid-whiteboard (e.g., '"now assume 10x more objects'") — what do you do?"** → Identify which stage breaks first under the new constraint (Hungarian's O(n³) becomes the bottleneck — Part D style analysis) and propose the standard mitigation (spatial bucketing/pre-clustering to shrink effective n) rather than re-deriving the whole design from scratch.

---

### Q8. "How would you extend this system to handle 10 cameras and 4 radars?"

**Short:** Sensor ingest layer already scales horizontally (each `ISensorSource` is independent), so the main pressure points are: (1) cross-sensor association at fusion time (more pairwise combinations), and (2) real-time budget per cycle growing with total detection count — I'd discuss batching/parallelizing the per-sensor estimation stage (each sensor's pipeline is already independent, so this parallelizes naturally across threads/cores) before touching the fusion/decision layers.

---

### Q9. "What if this needed to run on a distributed system, not a single process?"

**Short:** The late-fusion architecture is actually distribution-friendly — since each sensor's estimation+association+lifecycle pipeline is already independent, those could run as separate processes/nodes (naturally maps to a ROS2-style multi-node architecture, which the stretch `ROS2Publisher` hints at), with fusion/decision as a downstream aggregating node; I'd flag that this introduces new problems (network latency/jitter affecting the time-sync boundary, needing a distributed clock strategy) that a single-process design doesn't have to solve.

---

### Q10. "How would you add support for predicting *other* vehicles' future trajectories, not just tracking their current state?"

**Short:** The CTRV/UKF motion models already produce a predicted future state as part of their normal predict step — extending `predict()` to run multiple steps ahead (rather than just one `dt` per cycle) gives a trajectory forecast directly from the existing estimator, with growing uncertainty (covariance) at each step — I'd discuss the accuracy limits of this approach (constant-motion-model assumptions break down over longer horizons, real trajectory prediction would need maneuver/intent modeling) rather than overclaim it solves trajectory prediction generally.

---

### Q11. "Where are the single points of failure in this architecture?"

**Short:** The Config/Determinism boundary itself (a bad config value cascades everywhere — mitigated by validation at load time and `ConfigHash` traceability), and the real-time engine's scheduler thread (if it stalls, the whole pipeline stalls — mitigated by keeping all non-essential I/O off that thread via the SPSC queue). I'd name these honestly rather than claim the architecture has none.

---

### Q12. "Sketch how you'd add unit economics — i.e., how would you reduce CPU/memory cost for an embedded target?"

**Short:** Switch `Eigen::MatrixXd` (dynamic, heap-allocating) to fixed-size `Eigen::Matrix4d`/`Matrix5d` per motion model (removes heap allocation from the hot path — Part B Q17 trade-off, revisited here as the concrete embedded-target answer), reduce Hungarian's effective n via spatial bucketing, and profile before assuming which change matters most — I wouldn't guess-optimize without data.

---

## 🗣️ SECTION E3 — Honesty Framing & Proactive Disclosure (Q13–Q17)

### Q13. What are you most proactive about disclosing, unprompted, in this project?

**Short:** Three things, stated early in any deep discussion: (1) radar is synthetic (derived from KITTI ground truth + injected noise, not real hardware), (2) FCW/AEB thresholds are reference-quality/tuned-for-demonstration, not NCAP/ISO-26262-certified values, (3) UKF/IMM/ARM64/ROS2 are explicitly labeled stretch goals that may or may not be fully complete depending on when I stopped — I'd rather an interviewer hear these from me first than discover them and wonder what else wasn't disclosed.

---

### Q14. Why lead with limitations instead of leading with strengths?

**Short:** I don't lead *only* with limitations — I lead with the strong technical narrative (Joseph form, tuning/holdout discipline, lock-free queue, sanitizer matrix) and weave in the honest caveats at the natural point each topic comes up, rather than either hiding them or opening with a wall of disclaimers. The goal is calibrated confidence, not false modesty or overclaiming.

**Trap — "Doesn't over-disclosing limitations make you look less confident?"** → Confidence in an engineering interview should come from *precision* about what's proven versus assumed — an interviewer trusts "I tested X, I have not tested Y, here's why Y would be hard" far more than blanket confidence that later turns out to be unfounded.

---

### Q15. An interviewer says "so this whole thing is basically fake since the sensor data is synthetic" — how do you respond?

**Short:** Push back respectfully but factually: synthetic sensor generation from ground truth + injected noise is a standard, well-understood technique specifically for isolating and testing estimation/fusion/tracking/decision logic without needing real hardware or a labeled real-sensor dataset — the engineering under test (Kalman math, association, fusion, decision layering, real-time infra) is completely real and independently verifiable; only the sensor *input* is a controlled proxy, and that's stated as a deliberate scope boundary, not a concealment.

---

### Q16. How do you talk about the parts of the project that are *not* built yet (stretch goals) if asked directly?

**Short:** Directly: "UKF and IMM are stretch goals — here's the math and design I'd use (Part B, Q19-Q23), and here's honestly whether I got to implementing them by the time of this interview." I never present a design document as if it were working, tested code.

---

### Q17. What's a mistake or wrong turn you made during this project, and what did you learn?

**Short:** Early on I didn't build the `RunConfig`/seeded-RNG determinism boundary until partway through — a couple of early scenario tests were flaky because randomness wasn't seeded consistently, which cost debugging time chasing what looked like a real Kalman-filter bug but was actually just non-reproducible noise. Lesson: build the determinism/reproducibility foundation before writing anything that depends on randomness, not after (see also E1 Q5).

---

## 📖 SECTION E4 — STAR Stories (Q18–Q21)

### Q18. Tell me about a time you found a subtle bug that could have easily gone unnoticed.

**Situation:** During KF/EKF development, tests were passing and RMSE looked reasonable.
**Task:** I'd added debug-build PSD/symmetry asserts on the covariance matrix as a general safety practice, not because I suspected a specific bug.
**Action:** After ~200 update cycles in a longer scenario test, the PSD assert fired — I traced it to a subtle typo in the measurement matrix `H` that was *technically* producing plausible-looking output most of the time but silently violating covariance validity under accumulated floating-point rounding.
**Result:** Fixed the `H` matrix bug, and more importantly, kept the Joseph-form update + PSD asserts as a permanent guard rather than a one-off debugging step, since this class of bug is exactly the kind that "passes the demo" but would fail in a longer real deployment.

---

### Q19. Tell me about a time you had to make a trade-off between doing something "the right way" and shipping on time.

**Situation:** 60-day self-imposed timeline, with UKF/IMM/ARM64/ROS2 all technically desirable additions.
**Task:** Decide what's core versus stretch without letting scope creep threaten the core deliverable.
**Action:** Explicitly split the plan into a required v1.0 checklist and a gated Phase 13/stretch week — core estimation, association, fusion, decision, real-time engine, and the full test/CI matrix had to be complete and green *before* any stretch work started.
**Result:** This meant some stretch items (UKF/IMM) were designed in full but implementation timing depended on how much runway remained — a conscious trade-off I can discuss honestly rather than one I'd need to hide.

---

### Q20. Tell me about a time you had to defend a design decision against a reasonable-sounding counterargument.

**Situation:** Late vs. early sensor fusion (Part C Q1) — early fusion has a real theoretical accuracy advantage.
**Task:** Decide and be able to defend late fusion despite that.
**Action:** Weighed testability/fault-isolation/graceful-degradation benefits against the theoretical ceiling early fusion offers, and made the call based on what's actually achievable and verifiable within project constraints, documenting the reasoning rather than presenting it as an obviously-correct choice.
**Result:** I can articulate both sides and specifically why I chose one, which is a stronger position than either not knowing the alternative existed or claiming my choice has no downsides.

---

### Q21. Tell me about a time you had to learn something (a concept/technique) you didn't already know well to complete a task.

**Situation:** Covariance Intersection was not something I'd implemented before starting this project.
**Task:** Understand and correctly implement a fusion technique that's specifically designed for *unknown correlation* between estimates — subtler than naive inverse-covariance weighting.
**Action:** Studied the mathematical formulation, understood *why* naive weighting fails (produces overconfident/inconsistent covariance under real correlation), and validated my implementation against known mathematical properties (fused covariance never larger than either input) before trusting it in the pipeline.
**Result:** Correctly implemented and validated CI, and can now explain not just how to use it but why it exists and what problem it specifically solves versus simpler alternatives.

---

## 🎬 SECTION E5 — Demo Script & Curveballs (Q22–Q25)

### Q22. Walk me through how you'd actually demo this project live in 3 minutes.

**Short:** (1) 30-second pitch (memorized block from the hub file) — what it is, why classical not DL, the 12-boundary shape. (2) Run a scenario replay showing FCW triggering with the `reason` string visible, then AEB staging as TTC drops further. (3) Show the CV-vs-CTRV or fused-vs-single-sensor RMSE comparison report as concrete evidence, not just a visual. (4) Mention the tuning/holdout split and CI sanitizer matrix as the "how do I know this isn't fragile" answer, saving 30 seconds for questions.

---

### Q23. Curveball: "What if I told you TTC-based FCW is considered outdated and modern systems use learned risk models — defend your approach."

**Short:** TTC-based FCW is still a widely used, well-understood baseline in real deployed systems and standards discussions precisely because it's interpretable and traceable (Part C Q25) — a learned risk model can outperform it in aggregate but is harder to explain/certify/debug for any single decision; I'd frame TTC+hysteresis as the appropriate baseline for a project demonstrating classical estimation/decision engineering, while acknowledging learned risk models are a legitimate, different research direction I'd be glad to discuss the trade-offs of.

---

### Q24. Curveball: "Your MOTA number isn't as good as published SOTA trackers — why should I be impressed?"

**Short:** Direct comparison to SOTA research trackers (often DL-based, trained on massive datasets, evaluated on standardized benchmarks with leaderboards) isn't the right comparison — the value here is demonstrating correct implementation of the *classical* estimation/association/tracking theory end-to-end, with proper measurement methodology (Part D, tuning/holdout), which is a different and complementary skill set to training a SOTA DL tracker.

---

### Q25. Curveball: "If you had one more week, what's the highest-leverage thing you'd add?"

**Short:** Extending the closed-form/property-based test coverage to the fusion and decision layers as rigorously as estimation already has (Part B/C), since that's currently validated more by scenario-level tests than by the same tight closed-form guarantees the KF/EKF math enjoys — that's the area where I'd get the most correctness-confidence per hour invested.

---

## ❓ SECTION E6 — Reverse Questions to Ask Interviewers (Q26–Q28)

### Q26. Good reverse questions about the *engineering culture* around safety-relevant systems.

- "How does your team validate threshold/tuning decisions in production ADAS features — is there an equivalent to a tuning/holdout split, or a different methodology?"
- "What's your CI/testing philosophy for real-time or safety-adjacent code — do you run sanitizer matrices, and how do you handle flaky hardware-in-the-loop tests?"

---

### Q27. Good reverse questions about scope and team structure.

- "Is the perception/fusion team responsible for the full pipeline end-to-end, or split between detection, tracking, and decision-layer teams?"
- "How much of the sensor fusion work is classical/rule-based versus learned models in your current stack, and is that shifting?"

---

### Q28. Good reverse questions that show genuine technical curiosity (not just "process" questions).

- "For your real-time pipeline, what does your actual p99 cycle-time budget look like, and what's the biggest source of tail latency you've had to fix?"
- "When two sensors disagree strongly on an object's position, what's your team's actual fusion/arbitration strategy in production?"

---

## 🔗 Related design docs

[00_MASTER_PLAN.md](00_MASTER_PLAN.md) · [04_PROJECT_SCRIPT.md](04_PROJECT_SCRIPT.md) · [05_INTERVIEW_PREP.md](05_INTERVIEW_PREP.md)

Back to: [Hub](05_INTERVIEW_PREP.md)
