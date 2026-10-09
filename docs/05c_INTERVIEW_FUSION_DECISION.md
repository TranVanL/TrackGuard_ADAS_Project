# 🎤 TrackGuard Interview — Part C: Fusion & Decision

> **Scope:** Early vs late fusion trade-off · Covariance Intersection · TTC kinematics · FCW hysteresis · AEB staged braking + override suppression · ISO 26262/NCAP honesty framing.
> **Level target:** Senior ADAS/Perception C++. Mỗi câu có **short answer** + **deep dive** + **trap/follow-up**.

Index: [Hub](05_INTERVIEW_PREP.md) · [A: Foundation](05a_INTERVIEW_FOUNDATION.md) · [B: Estimation/Tracking](05b_INTERVIEW_ESTIMATION_TRACKING.md) · **C: Fusion/Decision** · [D: System/Validation](05d_INTERVIEW_SYSTEM_VALIDATION.md) · [E: Behavioral](05e_INTERVIEW_BEHAVIORAL.md)

---

## 🔀 SECTION C1 — Fusion Architecture (Q1–Q8)

### Q1. Early fusion vs late fusion — which did you choose and why?

**Short:** Late (track-level) fusion — each sensor runs its own full estimation+association+lifecycle pipeline independently, producing sensor-specific tracks, which are then combined at the track level via Covariance Intersection. Early fusion would combine raw detections *before* estimation, which requires solving cross-sensor association at the raw-detection level (much harder, more failure-prone) and couples the sensors so tightly that one sensor's dropout can corrupt the whole pipeline.

**Deep dive:** Late fusion's key benefit is graceful degradation — a camera-only or radar-only track is still fully valid output if the other sensor drops out, since each sensor's pipeline never depended on the other to begin with. The cost is that late fusion can't correct one sensor's association mistake using the other sensor's better data at the point of association (early fusion could, in theory).

**Trap — "Isn't early fusion strictly more accurate, since you use all the information earlier?"** → In theory, yes, if perfectly implemented — in practice, early fusion requires precise cross-sensor calibration and a much harder association problem (different measurement spaces, different noise characteristics) that's a common source of subtle bugs; I chose late fusion as the more robust, testable, and honestly-scoped-for-60-days option, and I say so explicitly.

---

### Q2. What's the actual failure mode you're protecting against with late fusion?

**Short:** A single point of failure in the pipeline: if I fused raw detections and the calibration or cross-sensor association had a bug, *both* sensors' information would be corrupted simultaneously in a way that's hard to detect. With late fusion, each sensor's pipeline is independently testable and independently correct before fusion ever sees the data.

**Trap — "Have you actually tested this claim — did you inject a sensor dropout and confirm graceful degrade?"** → Yes — the `radar_only_dropout` and `camera_only_dropout` tuning scenarios specifically simulate one sensor going silent mid-scenario, and the test asserts the surviving sensor's track continues without an ID switch or gap.

---

### Q3. Walk through Covariance Intersection (CI) mathematically.

**Short:** Given two estimates `(x1, P1)` and `(x2, P2)` of the same quantity with *unknown correlation* between them, CI computes the fused estimate as `P_fused⁻¹ = ω·P1⁻¹ + (1-ω)·P2⁻¹` and `x_fused = P_fused·(ω·P1⁻¹·x1 + (1-ω)·P2⁻¹·x2)`, where `ω ∈ [0,1]` is chosen (e.g., by minimizing `det(P_fused)` or `trace(P_fused)`) to produce a *consistent* (not overconfident) fused covariance.

**Deep dive:** The reason CI matters here specifically is that camera-track and radar-track estimates *are* correlated in an unknown way — both ultimately derive from the same physical object's motion and share some common process-noise assumptions — so naively combining them as if independent (simple inverse-covariance weighting) would produce an overconfident (too-small) fused covariance, a textbook consistency violation.

**Trap — "How do you actually pick ω in code?"** → A 1D line search over `ω ∈ [0,1]` minimizing `trace(P_fused)`, since a closed-form solution isn't generally available — documented as an accepted approximation (grid search is fine at this scale/frequency; a closed-form or golden-section search would be a valid optimization if profiling required it).

---

### Q4. Why not just average the two tracks' positions?

**Short:** Naive averaging ignores each source's confidence — if radar is much more confident (smaller covariance) than camera at long range, a straight average unfairly drags the fused estimate toward the less reliable camera detection. CI (or even simple inverse-covariance weighting, if independence were assumable) weights by confidence automatically.

**Trap — "When *would* simple averaging be acceptable?"** → If sensors were verified to have independent, uncorrelated errors — a stronger assumption than is generally safe for two tracks derived from overlapping detections of the same object, hence CI's conservative "unknown correlation" framing is the safer default.

---

### Q5. What happens if only one sensor has a track for an object (no fusion candidate)?

**Short:** That sensor's track is passed through to the output directly, unfused, with a flag/attribute indicating single-sensor origin — the "graceful degrade" path. No CI is attempted because there's nothing to fuse with.

**Trap — "Does the decision layer (FCW/AEB) treat single-sensor and fused tracks differently?"** → It could (e.g., require slightly higher confidence for single-sensor-only threats), but in the core implementation I treat them the same to keep the decision layer simple, noting the asymmetric-trust idea as a documented possible refinement rather than implemented behavior.

---

### Q6. How do you associate a camera-track with a radar-track for fusion (cross-sensor association)?

**Short:** Similar to intra-sensor association — Mahalanobis-gated nearest-match between the two sensors' track lists, but at the *track* level (post-estimation) rather than the *detection* level, meaning each side already has a smoothed, converged position estimate rather than raw noisy detections, making the matching problem meaningfully easier and more stable.

---

### Q7. What's a concrete example of late fusion producing a *better* result than either sensor alone?

**Short:** Camera has good angular/lateral precision but weaker range precision (harder to judge distance from a monocular-style bbox); radar has excellent range/range-rate precision but weaker lateral precision. CI naturally weights the fused position to inherit camera's lateral accuracy and radar's range accuracy, producing a fused estimate more precise than either alone in its respective weak axis.

**Trap — "Do your tests actually demonstrate this quantitatively?"** → Yes — the CV/CTRV-style comparison methodology extends to fusion: I compare fused-track RMSE against camera-only and radar-only RMSE on the same scenario, expecting (and validating) that fused RMSE is lower or comparable in both axes, not just "trust me."

---

### Q8. If you had to add a third sensor (Lidar), how would fusion generalize?

**Short:** CI generalizes to N sources by sequentially fusing pairs (fuse camera+radar, then fuse that result with lidar) since CI is associative for combining consistent estimates — no redesign needed, just an additional pairwise fusion call in the fusion stage.

---

## ⏱️ SECTION C2 — TTC Kinematics (Q9–Q13)

### Q9. Derive Time-To-Collision (TTC) from first principles.

**Short:** For a lead object at relative distance `d` closing at relative velocity `v_rel` (positive = closing), assuming constant relative velocity over the short prediction horizon: `TTC = d / v_rel`. If `v_rel ≤ 0` (not closing, or moving away), TTC is undefined/infinite (no collision risk from current kinematics).

**Deep dive:** I use *relative* longitudinal distance/velocity computed from the fused track's position/velocity minus ego's own velocity (from odometry) — this must be done in a consistent frame (vehicle frame, forward-positive), which is exactly why the coordinate transform/calibration boundary upstream matters so much: a frame error here silently corrupts every TTC computation downstream.

**Trap — "What if relative acceleration is significant, not just constant velocity?"** → A more refined TTC formula incorporates relative acceleration (solving the quadratic `d = v_rel·t + ½a_rel·t²`) — I use the constant-velocity approximation as the primary formula (industry-common simplification for short horizons) and note the quadratic refinement as a documented possible extension, not implemented by default.

---

### Q10. Why does TTC alone risk being noisy/jittery as a warning trigger?

**Short:** TTC is a ratio of two noisy estimated quantities (distance, relative velocity) — small estimation noise in either can cause TTC to swing significantly, especially as `v_rel` approaches zero (denominator near zero → TTC magnitude explodes). This is exactly why the raw TTC value is never used directly to trigger FCW — it feeds into a hysteresis state machine instead.

---

### Q11. How does `PathPredictor` decide which detected object is "the" lead vehicle to compute TTC against?

**Short:** Filters candidate tracks by lateral offset relative to ego's predicted path (using ego's current curvature from yaw rate) and forward distance, keeping only objects within a lane-width-ish lateral corridor projected forward — this avoids computing TTC against an adjacent-lane vehicle that poses no actual collision risk.

**Trap — "What about curved roads — doesn't a straight lateral corridor mispredict?"** → The corridor is curved based on ego's current yaw rate (a simple constant-curvature path projection), not a flat straight-line assumption — still a simplification (doesn't account for the *lead vehicle's* future path), documented as such.

---

### Q12. What's the difference between TTC and headway/time-gap?

**Short:** TTC is about *closing* speed (only meaningful when actually approaching); time-gap/headway is `distance / ego_speed` regardless of relative motion — a following-distance metric used more for ACC-style following than collision-imminent warnings. TrackGuard focuses on TTC since FCW/AEB are collision-imminent systems, not headway-maintenance systems.

---

### Q13. How do you unit-test TTC computation?

**Short:** Closed-form scenarios: object at exactly 20m closing at exactly 10m/s → TTC must equal exactly 2.0s (within floating-point tolerance); object moving away (negative relative velocity) → TTC must report "undefined"/infinite, not a nonsensical negative number.

---

## 🚨 SECTION C3 — FCW Hysteresis (Q14–Q19)

### Q14. Why not just trigger FCW the instant TTC drops below a threshold?

**Short:** Because TTC is noisy near the threshold boundary (see Q10), a naive threshold-crossing trigger would rapidly flicker on/off as noisy TTC oscillates around the threshold — annoying and, more importantly, undermines driver trust in the warning system. `FcwStateMachine` uses hysteresis: a *different*, more lenient threshold to turn the warning *off* than the stricter one used to turn it *on*, plus a minimum-duration/debounce requirement.

**Trap — "Isn't hysteresis itself an arbitrary tuning knob then?"** → Yes, and I say so explicitly — the exact thresholds are tuned on the tuning scenario set (never holdout) and explicitly labeled in docs as "reference-quality, not NCAP-certified values" — the engineering pattern (hysteresis to prevent chattering) is the transferable, defensible knowledge; the exact numbers are not claimed to be industry-certified.

---

### Q15. Describe the FCW state machine's states and transitions.

**Short:** `Idle` → `Warning` when TTC < `ttc_warn_on` threshold sustained for `min_on_frames`; `Warning` → `Idle` only when TTC > `ttc_warn_off` threshold (a *larger*, more lenient value than `ttc_warn_on`) sustained for `min_off_frames`. The asymmetric on/off thresholds plus frame-count debouncing are what prevent rapid flicker.

---

### Q16. What if the tracked object's ID switches while a warning is active — does the warning reset?

**Short:** This is exactly the scenario the track-lifecycle `Coasting` state and low-ID-switch validation exist to prevent — as long as the underlying track survives (doesn't get deleted/reassigned a new ID) through brief association hiccups, the FCW state machine's debounce counters continue uninterrupted; an actual ID switch would incorrectly reset the debounce counters, which is why minimizing ID switches (Part B, MOTA) directly matters for decision-layer quality, not just as an abstract tracking metric.

---

### Q17. How would you validate the hysteresis logic itself, independent of the TTC computation?

**Short:** Feed the state machine a synthetic, controlled sequence of TTC values directly (bypassing the full pipeline) — including a value that oscillates right at the "on" threshold — and assert the state machine does *not* flicker, while a value that clearly crosses both on/off thresholds *does* transition as expected. This isolates hysteresis-logic correctness from any upstream estimation noise.

---

### Q18. What's the honest disclosure you make about FCW thresholds in this project?

**Short:** Documented explicitly in the knowledge base: these thresholds are reference-quality values chosen via the tuning/holdout scenario methodology for demonstration purposes — they are *not* derived from or validated against a real regulatory standard (Euro NCAP, ISO 26262 ASIL requirements), and I say this proactively rather than waiting to be asked.

**Trap — "So what's the point of tuning if it's not certified?"** → The point is demonstrating the *engineering process* — a proper tuning/holdout split, avoiding overfitting to known scenarios, defensible closed-form validation — which is the transferable skill; a real production system would additionally need extensive real-world data and regulatory validation this project doesn't have access to.

---

### Q19. Could FCW ever suppress a warning it shouldn't?

**Short:** Yes, potentially, if `ttc_warn_off` is set too leniently (warning clears too easily) or `min_off_frames` too small — this is a real tuning trade-off between chatter-avoidance and warning-persistence, which is exactly why it's evaluated against multiple scenarios (including edge cases like a lead vehicle briefly accelerating then decelerating again) rather than tuned against a single scenario.

---

## 🛑 SECTION C4 — AEB Staged Braking & Override (Q20–Q25)

### Q20. Describe AEB's staged braking design.

**Short:** AEB doesn't jump straight from "no braking" to "full emergency brake" — it has staged severity levels (e.g., Stage 1: partial brake / stronger warning at a shorter TTC than FCW; Stage 2: harder brake at very short TTC) tied to progressively shorter TTC thresholds, mirroring how real AEB systems escalate response proportionally to urgency rather than binary on/off.

**Trap — "Why not just brake at max force the instant AEB triggers at all?"** → Unnecessarily harsh braking has its own risk (rear-end collision from a following vehicle, driver startle) — staged response is a real-world AEB design pattern precisely to balance collision-avoidance against introducing a *new* risk from overly aggressive braking on borderline-necessary triggers.

---

### Q21. What is driver-override suppression and why does it matter?

**Short:** If the driver is actively braking or the accelerator/steering shows clear evasive action already in progress, AEB suppresses/reduces its own intervention rather than fighting the driver's control input — modeled via a simple rule (e.g., driver brake pressure above some threshold suppresses AEB stage escalation), representing the real design principle that automated safety systems should support the driver, not override deliberate action.

**Trap — "How do you simulate 'driver input' in a synthetic scenario?"** → A scenario-level flag/injected signal representing simulated brake-pedal state, since there's no real driver-input hardware in this project — documented explicitly as a simplified stand-in for the real concept, not a claim about doing real driver-monitoring.

---

### Q22. Why is AEB's threshold *stricter* (shorter TTC) than FCW's?

**Short:** FCW's job is to warn early enough for a human to react (needs more lead time, so triggers at a *longer* TTC); AEB is a last-resort automated intervention that should only escalate to actual braking when a collision is imminent and the driver likely hasn't/can't react in time (shorter TTC, later trigger) — the staged relationship between FCW and AEB thresholds is itself a designed safety-layering pattern, not arbitrary.

---

### Q23. How do you test the interaction between FCW and AEB together, not just each in isolation?

**Short:** A scenario where TTC decreases continuously and monotonically — asserting FCW triggers first (at its longer-TTC threshold), then AEB stage 1, then AEB stage 2, in the correct order and with the correct relative timing — validating the *layered* decision behavior, not just each component's isolated logic.

---

### Q24. What's the `reason` string for an AEB stage-2 trigger look like, concretely?

**Short:** Something like `"AEB stage2: TTC=0.6s < threshold=0.8s, no driver override, track_id=42, confidence=fused"` — every field traceable back to the actual pipeline state that caused the decision, supporting the traceability principle discussed in Foundation Q5.

---

### Q25. If an interviewer says "this isn't a real safety system, why does the design rigor matter?"

**Short:** Because the *engineering discipline* (staged response, override suppression, hysteresis, traceable decisions, tuning/holdout separation) is exactly the kind of thinking real safety-relevant systems require — demonstrating I understand *why* these patterns exist, even on a portfolio project without certification, is the signal a technical interviewer is actually looking for; I'm not claiming the artifact itself is certifiable.

---

## 🔗 Related design docs

[knowledge/03d_SENSOR_FUSION_ARCHITECTURE.md](knowledge/03d_SENSOR_FUSION_ARCHITECTURE.md) · [knowledge/03e_TTC_FCW_AEB_DECISION.md](knowledge/03e_TTC_FCW_AEB_DECISION.md)

Next: [Part D — System/Validation](05d_INTERVIEW_SYSTEM_VALIDATION.md)
