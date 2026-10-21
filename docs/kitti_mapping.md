# KITTI → TrackGuard mapping

Day 3 reference for the KITTI Tracking labels and OXTS ego-motion stream.

## 1. KITTI tracking label (`label_02`)

A standard tracking-label row has 17 fields:

| Index | KITTI field | Type/unit | Meaning |
|---:|---|---|---|
| 0 | `frame` | integer | Frame index within the sequence |
| 1 | `track_id` | integer | Ground-truth object identity; never use as the tracker-generated ID |
| 2 | `type` | string | `Car`, `Pedestrian`, `Cyclist`, `DontCare`, etc. |
| 3 | `truncated` | float, `[0, 1]` | Fraction of the object truncated by the image boundary |
| 4 | `occluded` | integer | Occlusion level: 0 visible, 1 partly, 2 largely, 3 unknown |
| 5 | `alpha` | radians | Observation angle of the object |
| 6 | `bbox_left` | pixel | 2D image bounding-box left edge |
| 7 | `bbox_top` | pixel | 2D image bounding-box top edge |
| 8 | `bbox_right` | pixel | 2D image bounding-box right edge |
| 9 | `bbox_bottom` | pixel | 2D image bounding-box bottom edge |
| 10 | `height` | metres | 3D object height |
| 11 | `width` | metres | 3D object width |
| 12 | `length` | metres | 3D object length |
| 13 | `location_x` | metres | 3D location in the KITTI camera frame, right |
| 14 | `location_y` | metres | 3D location in the KITTI camera frame, down |
| 15 | `location_z` | metres | 3D location in the KITTI camera frame, forward |
| 16 | `rotation_y` | radians | Object yaw in the KITTI camera frame |

KITTI camera-frame convention used here:

```text
x: right, y: down, z: forward
```

`location` is not yet the TrackGuard vehicle frame. A later calibration/transform
stage must map it to the common vehicle frame.

A detector score is not part of the 17-field ground-truth label format. Some
prediction files contain an optional field 17 (`score`); the parser should accept
it only as optional metadata, not assume it exists in ground truth.

## 2. OXTS row (`oxts/<sequence>.txt`)

A standard OXTS row has 30 fields. Units follow the KITTI raw-data convention:

| Index | Field | Type/unit | Meaning |
|---:|---|---|---|
| 0 | `lat` | degrees | GPS latitude |
| 1 | `lon` | degrees | GPS longitude |
| 2 | `alt` | metres | GPS altitude |
| 3 | `roll` | radians | Ego roll |
| 4 | `pitch` | radians | Ego pitch |
| 5 | `yaw` | radians | Ego yaw/heading |
| 6 | `vn` | m/s | Velocity toward geographic North |
| 7 | `ve` | m/s | Velocity toward geographic East |
| 8 | `vf` | m/s | Velocity in the vehicle-forward direction |
| 9 | `vl` | m/s | Velocity in the vehicle-left direction |
| 10 | `vu` | m/s | Velocity in the vehicle-up direction |
| 11 | `ax` | m/s² | Acceleration in the sensor/body x direction |
| 12 | `ay` | m/s² | Acceleration in the sensor/body y direction |
| 13 | `az` | m/s² | Acceleration in the sensor/body z direction |
| 14 | `af` | m/s² | Acceleration in vehicle-forward direction |
| 15 | `al` | m/s² | Acceleration in vehicle-left direction |
| 16 | `au` | m/s² | Acceleration in vehicle-up direction |
| 17 | `wx` | rad/s | Angular rate around x |
| 18 | `wy` | rad/s | Angular rate around y |
| 19 | `wz` | rad/s | Angular rate around z/up; useful yaw-rate value |
| 20 | `wf` | rad/s | Angular rate around vehicle-forward axis |
| 21 | `wl` | rad/s | Angular rate around vehicle-left axis |
| 22 | `wu` | rad/s | Angular rate around vehicle-up axis |
| 23 | `pos_accuracy` | metres | Position accuracy estimate |
| 24 | `vel_accuracy` | m/s | Velocity accuracy estimate |
| 25 | `navstat` | integer | Navigation status |
| 26 | `numsats` | integer | Number of satellites |
| 27 | `posmode` | integer | Position-fix mode |
| 28 | `velmode` | integer | Velocity-fix mode |
| 29 | `orimode` | integer | Orientation-fix mode |

OXTS does not provide a separate timestamp in this text row. For the synced
tracking stream, line index is treated as frame index:

$$
 t_{frame} = frame / 10\text{ Hz}
$$

This is a replay timestamp, not a claim that the original hardware clock was
exactly represented by the text file.

## 3. Runtime timestamp mapping

```text
label frame 0  → timestamp 0 ms
label frame 1  → timestamp 100 ms
label frame 2  → timestamp 200 ms
OXTS line 0    → ego state for frame 0
OXTS line 1    → ego state for frame 1
OXTS line 2    → ego state for frame 2
```

A label file can contain several rows for one frame, while OXTS normally has one
row per frame. Never map OXTS by label-line number.

## 4. Mapping to the planned `RawDetection`

`RawDetection` is a sensor measurement, not a complete KITTI ground-truth row.
The loader should first preserve a richer internal `KittiObjectLabel`, then an
adapter can create `RawDetection` without losing evaluation metadata.

Recommended camera-proxy convention for the planar position:

```text
RawDetection.t          = frame * 100 ms
RawDetection.sensor     = Camera
RawDetection.pos.x      = location_x       # lateral/right in camera frame
RawDetection.pos.y      = location_z       # longitudinal/forward in camera frame
RawDetection.confidence = synthetic/configured value; not present in GT label
```

For the camera adapter, velocity is not directly measured by KITTI's 2D camera
annotation. Do not put OXTS ego velocity into `RawDetection.vel`. Either keep
velocity invalid/optional or derive object apparent velocity from two consecutive
object locations and document that it is a ground-truth-backed proxy.

Keep these fields as metadata for evaluation and later modules:

```text
track_id, type, truncation, occlusion, alpha,
bbox, dimensions, location_y, rotation_y
```

The KITTI `track_id` is used to calculate tracking metrics and association
correctness. The future `TrackManager` must generate its own runtime ID.

## 5. Mapping OXTS to an ego-motion sample

OXTS does not become a `RawDetection`. It should become a separate ego-state
record, conceptually:

```text
EgoMotionSample.t                  = frame * 100 ms
ego.latitude_deg                  = lat
ego.longitude_deg                 = lon
ego.altitude_m                    = alt
ego.roll_rad                      = roll
ego.pitch_rad                     = pitch
ego.yaw_rad                       = yaw
ego.velocity_forward_mps          = vf
ego.velocity_left_mps             = vl
ego.velocity_up_mps               = vu
ego.yaw_rate_rad_per_s            = wz
ego.position_accuracy_m            = pos_accuracy
ego.velocity_accuracy_mps         = vel_accuracy
ego.navstat / numsats / *mode    = quality metadata
```

`vf`, `vl`, and `vu` describe ego motion. They are not the velocity of a detected
object. Later `EgoMotionCompensator` uses these values to compensate an object
measurement when the vehicle reference frame moves.

## 6. Day 3 validation rules

- A valid label row has at least 17 fields.
- A valid OXTS row has at least 30 fields.
- Empty lines are ignored.
- Malformed rows are reported with their source line number.
- `DontCare` is not a trackable object.
- `track_id` remains ground-truth metadata.
- Label and OXTS indices are joined by frame index, not by raw line count.
- All units and coordinate-frame assumptions remain explicit in the loader tests.
