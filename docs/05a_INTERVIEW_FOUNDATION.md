# 🎤 TrackGuard Interview — Part A: Foundation

> **Scope:** 12-boundary architecture overview · Sensor model + synthetic radar generation · Replay/determinism · Time sync/ego-motion · Coordinate transform/calibration.
> **Level target:** Senior ADAS/Perception C++. Mỗi câu có **short answer** (30-60s) + **deep dive** + **trap/follow-up**.

Index: [Hub](05_INTERVIEW_PREP.md) · **A: Foundation** · [B: Estimation/Tracking](05b_INTERVIEW_ESTIMATION_TRACKING.md) · [C: Fusion/Decision](05c_INTERVIEW_FUSION_DECISION.md) · [D: System/Validation](05d_INTERVIEW_SYSTEM_VALIDATION.md) · [E: Behavioral](05e_INTERVIEW_BEHAVIORAL.md)

---

## 🏗️ SECTION A1 — Architecture Overview (Q1–Q8)

### Q1. Describe TrackGuard's architecture end-to-end.

**Short:** TrackGuard is a 12-boundary staged real-time pipeline: config/determinism → replay harness → sensor adapters → time sync/ego-motion → coordinate transform → Kalman/EKF estimation → Hungarian data association → track lifecycle → multi-sensor (late) fusion → TTC threat assessment → FCW/AEB decision → real-time engine + validation, plus an optional portability/interop boundary. Each stage depends only on the struct produced by the stage before it, so any stage is swappable without touching the rest (Open-Closed).

**Deep dive:** The two "infrastructure" boundaries most demo projects skip are Time Sync/Ego-motion and the Real-time Engine — I built both explicitly because they're what interviewers actually probe ("camera at 10Hz, radar at 20Hz, how do you synchronize?" / "does your pipeline actually run at a fixed rate?"). Config/Determinism is a third one I added specifically to make every scenario test reproducible.

**Trap — "Which boundary was hardest to get right?"** → Track lifecycle (Boundary F): getting the Tentative→Confirmed→Coasting→Deleted thresholds right so IDs survive short occlusion but don't leak stale tracks took several iterations against the multi-object dense scenario.

---

### Q2. Why did you choose a classical (Kalman + Hungarian + rule-based) pipeline instead of a deep-learning tracker like DeepSORT or ByteTrack?

**Short:** Deliberate choice to prove foundational estimation theory and combinatorial optimization understanding — not because I don't know DL trackers. DeepSORT itself still uses a Kalman Filter + Hungarian assignment internally; it just replaces the appearance/re-ID feature with a CNN embedding. Understanding the classical core is a prerequisite to understanding DL-based trackers, not a substitute.

**Deep dive:** A DL tracker also introduces training data, GPU dependency, and non-determinism concerns that would derail a 60-day portfolio scoped around C++ engineering depth. The classical pipeline is fully testable with closed-form expectations (unit test asserts an exact RMSE bound), which a trained model can't offer as cleanly.

**Trap — "So you've never touched DL trackers?"** → I understand the architecture (detection → embedding → Kalman-based motion + appearance-cost Hungarian assignment) and can discuss it, I just didn't build one here — this project's honesty framing is explicit about that boundary.

---

### Q3. Walk me through what happens when a new video frame and radar sweep arrive at the same time (roughly).

**Short:** Each sensor's `ISensorSource::poll(t)` returns raw detections; `TimeSyncBuffer` buffers per-sensor and interpolates both to a common tick (50ms); `EgoMotionCompensator` dead-reckons older detections into the current ego frame using odometry; `ExtrinsicCalibration`+gates transform to vehicle frame and filter out-of-FOV/low-confidence detections; then each sensor's detections go through their own KF/EKF + Hungarian association + track lifecycle independently, and finally `TrackFuser` fuses camera-track and radar-track pairs via covariance intersection.

**Deep dive:** Running association *per-sensor first* (rather than fusing raw detections before association) is the "late fusion" choice — it means a camera-only or radar-only track can still exist and be output even if the other sensor briefly drops out.

**Trap — "What if two sensors report the same object with very different confidence?"** → Covariance Intersection weights the fused state by each source's covariance, so the more confident (lower-covariance) source dominates the fused estimate automatically — no manual confidence heuristic needed.

---

### Q4. Why is `RawDetection` a separate struct from `TransformedDetection`?

**Short:** `RawDetection` is in the sensor's own frame (pixels/range-azimuth); `TransformedDetection` is post-calibration, in the shared vehicle frame. Keeping them distinct enforces that every stage after calibration only ever consumes vehicle-frame data — you can't accidentally feed raw sensor-frame coordinates into the Kalman filter.

**Deep dive:** This is the same "Open-Closed via typed stage boundaries" idea used throughout — each struct is effectively a contract. Adding a new sensor (e.g., Lidar) only requires it to produce a `RawDetection` and go through the same `ExtrinsicCalibration`; nothing downstream changes.

**Trap — "Isn't this over-engineering for a portfolio project?"** → It's the minimum structure needed to make the pipeline testable stage-by-stage; without it, bugs from a bad calibration would be indistinguishable from bugs in the Kalman filter when debugging test failures.

---

### Q5. What's `DecisionOutput` and why does it include a `reason` string?

**Short:** `DecisionOutput` carries the final FCW/AEB state plus the `primary_threat` track ID and a `reason` string (e.g., "TTC 0.8s < AEB stage2 threshold, no driver override detected"). The reason field exists so every actuation-adjacent decision is traceable back to the sensor data that caused it — a debugging and (conceptually) an auditability requirement.

**Deep dive:** This mirrors a real safety-relevant system property: any actuation command should be explainable, not just "black box says brake." For TrackGuard it's mainly for debugging and demo narration, but the design intent is explicitly stated as inspired by real functional-safety practice.

**Trap — "Is this actually safety-certified traceability?"** → No — explicitly not. It's a portfolio-quality analog of the idea, not an ISO 26262 traceability matrix.

---

### Q6. If interviewer asks you to whiteboard the pipeline from memory — what's your strategy?

**Short:** Draw left-to-right: Config/Determinism → Replay/Sensor Ingest → Time Sync/Ego-motion → Transform/Calibration → Estimation (KF/EKF) → Association (Hungarian+gating) → Track Lifecycle → Fusion (Covariance Intersection) → TTC → FCW/AEB → wrap the whole thing in a box labeled "Fixed-rate Engine (20Hz) + Validation".

**Deep dive:** Emphasize the boundary labels, not just boxes — interviewers are testing whether you understand *why* each cut point exists (testability, swappability, fault isolation), not just that you can draw boxes.

**Trap — "Where would you insert a Lidar sensor?"** → Right after Boundary A (Sensor Ingest) as a new `ISensorSource` implementation; it flows through the exact same Time Sync → Transform → Estimation pipeline as Camera/Radar with zero changes downstream.

---

### Q7. Why separate `estimation/` and `estimation_stretch/` directories?

**Short:** `estimation/` holds the core CV/EKF-CTRV filters required for `v1.0`; `estimation_stretch/` holds UKF and IMM, which are explicitly stretch goals gated behind finishing all core checkpoints first. Physically separating them makes the scope boundary visible in the codebase, not just in docs.

**Deep dive:** This also means the core pipeline never depends on the stretch module — if UKF/IMM never gets built, nothing else breaks or needs stubbing.

**Trap — "Doesn't that risk code duplication between KF and UKF?"** → Both implement the same estimator interface (expose `x`/`P`), so a common motion-model abstraction is shared; only the predict/update *mechanism* (linearization vs sigma points) differs.

---

### Q8. What would you change if you had to support a 6th sensor type tomorrow?

**Short:** Implement `ISensorSource` for it, add a `SensorType` enum value, and (if it has different noise characteristics) tune its `R` matrix in the KF/EKF construction and its FOV/confidence gate thresholds. No changes needed to association, tracking, fusion, or decision layers.

**Trap — "What if the new sensor has a completely different measurement space, e.g., a point cloud instead of a 2D detection?"** → I'd add a pre-processing stage before Boundary A (e.g., a clustering step to reduce point cloud → centroid detections) so it still emerges as a standard `RawDetection`, preserving the pipeline contract instead of special-casing it downstream.

---

## 📡 SECTION A2 — Sensor Model & Synthetic Radar (Q9–Q14)

### Q9. KITTI doesn't have radar data — how did you get radar detections?

**Short:** I generate **synthetic radar detections** from the KITTI ground-truth 3D object positions: compute range/azimuth/range-rate from ground-truth, then inject Gaussian noise (range ±0.1m, azimuth ±0.5°, range-rate ±0.2m/s) using a seeded RNG. This is a standard technique for building sensor-model test data when raw hardware data isn't available.

**Deep dive:** Range-rate comes from finite-differencing ground-truth position between frames (which is itself a simplification — I note this explicitly as "computed from ground-truth motion, not measured Doppler," to avoid overclaiming realism).

**Trap — "Doesn't that make radar 'too accurate' since it's derived from ground truth?"** → Yes, partially — the noise injection is what keeps it non-trivial (association/estimation still has real uncertainty to handle), but I'm explicit that this isn't a substitute for a real radar's Doppler ambiguities, ghost targets, or multipath. It's a controlled test fixture, not a hardware simulator.

---

### Q10. Why is the noise seeded rather than random each run?

**Short:** Determinism — the same scenario + same seed must always produce the exact same detections, so a test failure is reproducible and a before/after code comparison is meaningful (fixing a bug shouldn't accidentally "fix" a test because randomness happened to favor it that run).

**Deep dive:** The seed and a config hash are logged into every report, so a MOTA/MOTP number can always be traced back to "this exact seed + this exact config produced this exact result."

**Trap — "What if you want to test robustness across many noise realizations?"** → Run the same scenario with multiple explicit seeds (seed sweep) as a separate, opt-in test mode — determinism-by-default doesn't preclude deliberately testing across seeds when that's the actual goal.

---

### Q11. What does `CameraDetectionSource` actually output, and how is confidence assigned?

**Short:** It wraps `KittiTrackingLoader`, converts each ground-truth bbox center to an approximate 2D position, and assigns a synthetic confidence in [0.7, 0.95] (since KITTI ground truth has no native confidence — real detectors would).

**Deep dive:** The confidence range is intentionally realistic-but-imperfect (never 1.0) so `ConfidenceGate` in Boundary C has real filtering work to do, and so track initialization logic isn't trivially "always trust the detection."

**Trap — "Why not just use confidence 1.0 since it's ground truth?"** → Because that would hide bugs in the confidence-gating and fusion-weighting logic that a real detector's imperfect confidence would otherwise exercise.

---

### Q12. Explain the FOV gate and why it matters.

**Short:** `FovGate` discards detections outside a sensor's valid angular range or max range (e.g., radar past 150m or outside ±60°). Without it, a detection that's geometrically nonsensical for that sensor type could still enter the pipeline and pollute association.

**Deep dive:** FOV limits are sensor-type-specific constants set per adapter — this is metadata that belongs to the sensor model, not to any downstream stage, reinforcing the "each stage trusts its input contract" design.

**Trap — "What if FOV gating removes a real object?"** → That's expected and correct — a sensor genuinely cannot see outside its FOV; the fused track for that object should then rely on whichever other sensor *can* see it (graceful degrade via late fusion).

---

### Q13. How would you model a Lidar sensor differently from Camera/Radar?

**Short:** Lidar's `RawDetection` would come from a point-cloud clustering step (not implemented in core, listed as bonus) — cluster points into object-level centroids first, then feed those centroids through the same pipeline as any other sensor.

**Trap — "What kind of noise model would Lidar have?"** → Typically much lower positional noise than camera/radar but is affected by occlusion and point density falloff with range — I'd model that as range-dependent covariance growth rather than a flat Gaussian.

---

### Q14. What's the risk of using KITTI's ground truth directly as your "sensor output" for camera?

**Short:** It means the camera adapter is *too accurate* relative to a real detector (no false positives, no missed detections, no class confusion) — I'm explicit about this in the honesty framing: it's a controlled proxy for camera detections, useful for testing fusion/tracking/decision logic, not a claim about real camera-detector accuracy.

**Trap — "Would a reviewer catch this as 'cheating'?"** → Not if it's disclosed — the entire point of the project is fusion/tracking/decision C++ engineering, not building a camera detector; using ground truth + noise is the standard way to isolate that scope honestly.

---

## ⏱️ SECTION A3 — Determinism, Time Sync & Ego-Motion (Q15–Q20)

### Q15. What is `ConfigHash` and why put it in every report?

**Short:** `ConfigHash` is a SHA-256 hash of the `RunConfig` YAML file, embedded into every metrics/MOTA report. It's proof that "this exact result came from this exact configuration" — if thresholds change later, old reports remain distinguishable from new ones even if someone forgets to rename the report file.

**Trap — "What if two different configs hash-collide?"** → SHA-256 collision is cryptographically infeasible at this scale; the purpose here is accidental-mismatch prevention, not adversarial security.

---

### Q16. Why interpolate detections to a common tick instead of just processing each sensor's detections as they arrive?

**Short:** Camera (10Hz) and Radar (20Hz) arrive at different rates and offsets; without a common tick, association would be comparing detections from different points in time, silently degrading accuracy (especially for fast-moving objects). `TimeSyncBuffer` linearly interpolates the two nearest buffered detections per sensor to a shared 50ms tick.

**Deep dive:** Linear interpolation is a simplification — it assumes constant velocity within the interpolation window (~50ms), which is reasonable at that timescale but would break down for very fast dynamics or long gaps (handled by capping interpolation window and treating stale data as a dropout instead).

**Trap — "What if a sensor gap exceeds the interpolation window?"** → Treated as sensor dropout for that tick — the fusion layer's graceful degrade path (single-sensor track) handles it rather than extrapolating unreliable data.

---

### Q17. Explain `EgoMotionCompensator`'s dead-reckoning step.

**Short:** Given ego velocity and yaw-rate from odometry/IMU between two timestamps, compute Δx, Δy, Δyaw via simple integration, then rotate+translate older buffered detections into the current ego frame before using them — otherwise a detection from 50ms ago is stated relative to where the ego *was*, not where it *is now*, which corrupts both interpolation and association if the ego is turning.

**Deep dive:** Unit test: ego stationary → compensation is identity; ego turning 90° → detection rotates by exactly that amount — this is the closed-form check I used to validate correctness.

**Trap — "Why not use a full 6-DOF motion model?"** → 2D dead-reckoning (x, y, yaw) is sufficient for a ground-vehicle L2 ADAS scope; full 6-DOF (roll/pitch/z) would matter for off-road or aerial platforms, out of scope here.

---

### Q18. Why is `ExtrinsicCalibration` a 4x4 homogeneous matrix (or simplified 2D affine) rather than hand-tuned offsets?

**Short:** A homogeneous transform composably handles rotation+translation between sensor frame and vehicle frame in one matrix multiply, and generalizes cleanly if a sensor's mounting position/orientation changes — hand-tuned offsets would require re-deriving the math for every mounting change.

**Trap — "Where did your calibration numbers come from?"** → Simplified/approximated from KITTI's calibration file structure conceptually (not claiming to reproduce KITTI's exact extrinsics precisely) — noted explicitly in docs as a simplification for a 2D pipeline.

---

### Q19. How do you validate that Stage 1-3 (sensors→sync→transform) work correctly before touching estimation?

**Short:** An integration test compares transformed detection positions against ground truth after sync+transform, asserting the error stays below a small threshold — this isolates sync/transform bugs from estimation bugs, since a downstream RMSE failure could otherwise come from either layer.

**Trap — "Isn't that redundant with the final end-to-end RMSE test?"** → No — it's specifically there to localize failures faster; if Stage 1-3 integration test passes but end-to-end RMSE fails, the bug must be in estimation/association, not preprocessing.

---

### Q20. What's the "stretch" `ROS2Publisher` and why is it config-gated off by default?

**Short:** It's a thin wrapper that republishes `FusedTrack`/`DecisionOutput` as ROS2 messages for interop with robotics tooling — entirely optional, disabled by default (`enable_ros2_publisher: false`) so the core pipeline has zero ROS2 dependency unless explicitly opted in.

**Trap — "Why would you add ROS2 support to an ADAS project at all?"** → To demonstrate the core pipeline is interoperable with a common robotics ecosystem without coupling the core logic to any particular middleware — a design signal for JDs that value ROS/robotics generalism.

---

## 🧾 SECTION A4 — Requirements Traceability & Safety Case Honesty (Q21–Q26)

### Q21. Why does a solo 60-day portfolio project need a requirements traceability matrix?

**Short:** Because "it works when I ran it" is not the same claim as "every Must-have requirement has a test that would fail if the logic regressed." [06_REQUIREMENTS_TRACEABILITY.md](06_REQUIREMENTS_TRACEABILITY.md) gives every FR/NFR an ID, traced via `@satisfies`/`@verifies` Doxygen tags to the implementing class and the verifying test — so I (or a reviewer) can answer "is FR-013 actually tested?" by looking at a table instead of trusting memory.

**Trap — "Isn't this overkill for one person?"** → It's a habit-forming discipline for working on real automotive/safety-adjacent teams where this is mandatory, not administrative overhead — and it directly prevents the common solo-project failure mode of silently dropping test coverage under deadline pressure.

---

### Q22. What is HARA and why did you write one for a non-certified hobby project?

**Short:** Hazard Analysis and Risk Assessment — an ISO 26262 Part 3 concept-phase technique that goes hazard → severity/exposure/controllability → safety goal → technical safety requirement → architecture mechanism → test. I applied the *methodology*, not the *certification*, to FCW/AEB in [07_SAFETY_HAZARD_ANALYSIS.md](07_SAFETY_HAZARD_ANALYSIS.md) — it's explicitly disclaimed as portfolio-quality/reference-only, not a certified safety case.

**Deep dive:** Example: hazard "AEB unintended activation from a single ghost-tracked sensor" → Safety Goal SG-02 → Technical Safety Requirement TSR-02 ("AEB stage2 requires ≥2 contributing sensors OR very-high single-sensor confidence + very-short TTC fallback") → implemented as a check in `AebDecision` before stage escalation → verified by `TC-SAFETY-001`.

**Trap — "So is your AEB ASIL-B/C rated?"** → No — I explicitly call any ASIL letter in that document "ASIL tham khảo/reference-only": it's a self-assessed, non-reviewed application of the S/E/C parameters for demonstration, not an official ASIL determination which requires independent assessment I don't have access to as a solo developer.

---

### Q23. What's the difference between fail-silent and fail-operational, and which did you choose?

**Short:** Fail-silent means the system stops acting (goes to a safe neutral state) when it detects untrusted/implausible input; fail-operational means it must keep functioning correctly even after a fault, which requires redundancy this project doesn't have. I chose fail-silent — on implausible input (non-PSD covariance, NaN cost matrix, config load failure), the decision layer outputs `Idle`/`NoStage` with a clear `reason` rather than acting on bad data.

**Trap — "Isn't fail-silent dangerous for AEB — what if it silently stops braking mid-emergency?"** → Fail-silent here means *falling back to not making a new automated intervention decision*, not disabling the vehicle's actual brakes — and it's justified by the SAE L2 assumption that the human driver remains the fallback; fail-operational would be required for L4+, which is explicitly out of scope.

---

### Q24. Walk me through one full FR → design → test trace, end to end.

**Short:** FR-013 ("covariance update must use Joseph form, keep P symmetric+PSD") → satisfied by `estimation::KalmanFilterCV::update()` (tagged `@satisfies{FR-013}`) → verified by `TC-KF-003` (debug-build assertion that P stays symmetric/PSD over 1000 predict/update cycles, tagged `@verifies{FR-013}`) → row present in the Traceability Matrix in [06 §4](06_REQUIREMENTS_TRACEABILITY.md).

---

### Q25. What are the honest limitations of your HARA?

**Short:** No hardware-software interaction analysis, no independent safety assessment/confirmation review, S/E/C parameters are my own subjective estimates (not based on real accident statistics), no common-cause-failure analysis between camera and radar, no ASIL decomposition, and the whole scope is software decision logic only — excluding actuators and E/E hardware entirely. All six are listed explicitly in [07 §7](07_SAFETY_HAZARD_ANALYSIS.md) so I never have to be caught out by a follow-up question.

---

### Q26. Why go to this trouble instead of just writing more tests?

**Short:** More tests alone don't tell you *which* requirements they cover or *why* a threshold exists — traceability and HARA are the connective tissue between "why did I choose this architecture" and "how do I prove it does what it claims," which is exactly the gap between a hobbyist demo and something that reads as engineered for a real constrained domain.

---

## 🔗 Related design docs

[knowledge/03a_SENSORS_PERCEPTION.md](knowledge/03a_SENSORS_PERCEPTION.md) · [knowledge/03d_SENSOR_FUSION_ARCHITECTURE.md](knowledge/03d_SENSOR_FUSION_ARCHITECTURE.md) · [01_ARCHITECTURE.md](01_ARCHITECTURE.md) · [06_REQUIREMENTS_TRACEABILITY.md](06_REQUIREMENTS_TRACEABILITY.md) · [07_SAFETY_HAZARD_ANALYSIS.md](07_SAFETY_HAZARD_ANALYSIS.md)

Next: [Part B — Estimation/Tracking](05b_INTERVIEW_ESTIMATION_TRACKING.md)
