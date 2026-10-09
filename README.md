# 🚘 TrackGuard

**A C++17 ADAS Perception-to-Decision Pipeline** — sensor replay → time sync/ego-motion → coordinate transform → Kalman/EKF state estimation → Hungarian data association → multi-object tracking → multi-sensor (camera+radar) fusion → TTC threat assessment → FCW/AEB decision, running on a deterministic fixed-rate real-time engine.

> Solo portfolio project built to demonstrate **end-to-end ADAS software engineering depth** — not a single algorithm demo. Target roles: ADAS Software Engineer · Sensor Fusion Engineer · Perception (classical) Engineer · Autonomous Driving C++ Engineer · Test/Validation Engineer (ADAS).

[![C++17](https://img.shields.io/badge/C%2B%2B-17-blue.svg)](https://en.cppreference.com/w/cpp/17)
[![CMake](https://img.shields.io/badge/build-CMake-informational.svg)](https://cmake.org/)
[![CI](https://img.shields.io/badge/CI-gcc%2Fclang%20%C3%97%20Debug(ASan%2FUBSan%2FTSan)%2FRelease-success.svg)](#-quality--validation)
[![License](https://img.shields.io/badge/license-MIT-lightgrey.svg)](LICENSE)
[![Status](https://img.shields.io/badge/status-in%20active%20development-yellow.svg)](00_MASTER_PLAN.md)

---

## Why this project exists

Most portfolio ADAS projects stop at "I trained a detector" or "I ran a Kalman filter on a toy example." TrackGuard instead clones the **shape of a real L2 ADAS perception/decision stack** — 12 explicit pipeline boundaries, each individually testable, each traceable back to a requirement and (where relevant) a safety goal — using classical, from-scratch estimation and decision-making instead of a deep-learning black box. The point is to demonstrate the foundational math (Kalman theory, combinatorial optimization, kinematics) and the systems engineering discipline (determinism, real-time budgets, requirements traceability, hazard analysis, static analysis) that underpin *any* production ADAS stack, DL-based or not.

## Key Engineering Highlights

- **Numerically-safe Kalman/EKF core** — Joseph-form covariance update (not the textbook simplified form), debug-build PSD/symmetry assertions, closed-form kinematic unit tests, explicit yaw-rate≈0 singularity handling in CTRV.
- **Optimal data association** — Hungarian algorithm (O(n³)) with Mahalanobis (chi-square gated) or IoU cost, not greedy nearest-neighbor.
- **Principled multi-sensor fusion** — track-to-track (late) fusion via Covariance Intersection, chosen deliberately over early fusion for fault isolation and graceful sensor-dropout degradation (documented trade-off, not a default assumption).
- **Anti-chattering decision logic** — FCW hysteresis state machine (asymmetric on/off thresholds + debounce) and staged AEB braking with driver-override suppression; every decision emits a traceable `reason` string.
- **Deterministic-by-design** — every run is seeded (`std::mt19937_64`) through a single `RunConfig`, and every report embeds a SHA-256 `ConfigHash` so results are reproducible and auditable.
- **Real production-plane real-time engine** — fixed-rate 20 Hz scheduler on `steady_clock`, lock-free SPSC queue between the real-time thread and side-effect consumers (same lock-free "DNA" as [EventStreamCore](https://github.com/TranVanL/eventstreamCore)), p50/p95/p99 cycle-time profiling.
- **Anti-overfitting validation methodology** — 20+ scenarios split into a *tuning set* (14) used to choose thresholds and a *holdout set* (6+) used only for final reporting, mirroring train/test-split discipline applied to rule-based thresholds.
- **Requirements-traceable & safety-concept-inspired** — a lightweight Software Requirements Specification + traceability matrix ([06](06_REQUIREMENTS_TRACEABILITY.md)) and an ISO 26262-*inspired* (explicitly **not certified**) Hazard Analysis & Risk Assessment ([07](07_SAFETY_HAZARD_ANALYSIS.md)) back the FCW/AEB design decisions.
- **Automotive-flavored coding standard & static analysis gate** — a MISRA C++/AUTOSAR C++14-*inspired* subset ([08](08_CODING_STANDARDS_STATIC_ANALYSIS.md)) enforced via clang-tidy/cppcheck/Coverity in CI, alongside the full sanitizer matrix (ASan/UBSan/TSan).

## Architecture (12 boundaries)

```
Z Config/Determinism → A Sensor Ingest → B Time Sync/Ego-motion → C Coordinate Transform/Calibration
  → D State Estimation (KF/EKF, stretch UKF/IMM) → E Data Association (Hungarian+gating)
  → F Track Lifecycle → G Multi-Sensor Fusion (Covariance Intersection) → H Threat Assessment (TTC)
  → I Decision Layer (FCW/AEB) → J Real-time Engine → K Validation & Metrics → L Portability/Interop (stretch)
```

Full diagram, class design, and directory layout: [01_ARCHITECTURE.md](01_ARCHITECTURE.md).

## Tech Stack

| Area | Choice |
|---|---|
| Language / Standard | C++17 |
| Build | CMake (matrix: gcc/clang × Debug(ASan/UBSan/TSan)/Release, stretch ARM64+QEMU) |
| Linear algebra | Eigen3 |
| Testing | GoogleTest/gtest (unit + integration + scenario), lcov/gcov coverage |
| Static analysis | clang-tidy, cppcheck, Coverity |
| Dataset | KITTI tracking subset (public) + synthetic scenario generator (seeded) |
| CI | GitHub Actions |
| (Stretch) Interop | ROS2 publisher wrapper (config-gated, off by default) |

## Repository Layout

```
TrackGuard/
├── config/                  RunConfig defaults (YAML)
├── include/trackguard/      Public headers per boundary (sensors/, sync/, transform/,
│                            estimation/, association/, tracking/, fusion/, threat/,
│                            decision/, engine/, replay/, interop/)
├── src/                     Implementation .cpp
├── tests/                   gtest: unit + integration + scenario (150+ tests)
├── scenarios/                YAML scenario library — tuning set + holdout set
├── metrics/                 MOTA/MOTP + FCW/AEB precision-recall calculators
├── viz/                     2D top-down replay renderer (nice-to-have)
├── requirements/             Requirements/traceability working data (see 06)
├── safety/                   Hazard analysis working data (see 07)
├── docs/CODING_STANDARD.md   Full coding standard detail (see 08)
├── toolchains/               ARM64 cross-compile toolchain file (stretch)
└── .github/workflows/       ci.yml (required) + ci-arm64.yml (stretch, allow-failure)
```

## Quality & Validation

- **Testing pyramid**: closed-form unit tests (exact kinematic/Hungarian expectations) → component/integration tests (per-boundary correctness) → 20+ end-to-end scenario tests (MOTA/MOTP + FCW/AEB precision-recall), reported separately for tuning vs. holdout sets.
- **CI matrix**: gcc/clang × Debug (ASan+UBSan+TSan) / Release, plus a stretch allow-failure ARM64+QEMU job.
- **Static analysis gate**: clang-tidy + cppcheck + Coverity as required CI checks (see [08_CODING_STANDARDS_STATIC_ANALYSIS.md](08_CODING_STANDARDS_STATIC_ANALYSIS.md)).
- **Determinism**: seeded RNG + `ConfigHash` on every report — any result can be traced back to the exact config that produced it.

## Documentation Map

| Doc | Purpose |
|---|---|
| [00_MASTER_PLAN.md](00_MASTER_PLAN.md) | Roadmap, KPIs, milestones, positioning vs. other portfolio projects |
| [01_ARCHITECTURE.md](01_ARCHITECTURE.md) | Full pipeline diagram, class design, data model, directory layout |
| [02_PLAN_DETAILED_60DAY.md](02_PLAN_DETAILED_60DAY.md) | Day-by-day build plan |
| [04_PROJECT_SCRIPT.md](04_PROJECT_SCRIPT.md) | Pitch, demo script, positioning, honesty framing, FAQ |
| [06_REQUIREMENTS_TRACEABILITY.md](06_REQUIREMENTS_TRACEABILITY.md) | SRS + requirement → design → test traceability matrix |
| [07_SAFETY_HAZARD_ANALYSIS.md](07_SAFETY_HAZARD_ANALYSIS.md) | ISO 26262-inspired HARA, safety goals, functional safety concept (reference-quality, not certified) |
| [08_CODING_STANDARDS_STATIC_ANALYSIS.md](08_CODING_STANDARDS_STATIC_ANALYSIS.md) | Coding standard subset, static analysis tooling, review checklist, versioning |
| [knowledge/03_KNOWLEDGE_INDEX.md](knowledge/03_KNOWLEDGE_INDEX.md) | Theory hub (sensors, Kalman/EKF/UKF/IMM, association, fusion, TTC/FCW/AEB, real-time, testing) |
| [05_INTERVIEW_PREP.md](05_INTERVIEW_PREP.md) | 150+ Q&A interview prep hub (5 topic files) |

## Honest Scope & Limitations

Stated proactively, not waiting to be asked:

1. **Synthetic radar** — radar detections are generated from KITTI ground truth + injected Gaussian noise, not real hardware. Standard sensor-model-synthesis technique for testing fusion/tracking/decision logic without hardware access.
2. **Reference-quality thresholds, not certified** — FCW/AEB thresholds are tuned via the tuning/holdout methodology for demonstration; they are **not** derived from or validated against a specific OEM/NCAP/ISO 26262 certification process.
3. **HARA/requirements traceability are portfolio-quality artifacts** — they demonstrate understanding of the *process* (hazard identification, safety goal derivation, requirement-to-test traceability), not a certified safety case or audited requirements baseline.
4. **UKF/IMM/ARM64+QEMU/ROS2 are stretch goals** — gated behind all core (v1.0) checkpoints being complete; may or may not be finished depending on project timing.

Full framing: [04_PROJECT_SCRIPT.md §6](04_PROJECT_SCRIPT.md#6-honesty-framing-nói-chủ-động-không-đợi-bị-hỏi).

## Status

In active development, ~2-3h/night, 60-day core plan (+ optional stretch week). See [00_MASTER_PLAN.md](00_MASTER_PLAN.md) for the current milestone.

## License

MIT — see [LICENSE](LICENSE).
