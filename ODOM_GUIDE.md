# X-Drive Odometry — Setup & Usage Guide

> **TL;DR — you only need to edit two files:**
> - `include/xdrive_config.h` → port numbers, wheel sizes, wheel offsets
> - `src/xdrive_config.cpp` → PID gains, motor reversal flags

---

## Step 1 — Set your port numbers

Open [`include/xdrive_config.h`](include/xdrive_config.h) and change the `#define` values at the top to match your V5 brain wiring:

```cpp
// ── Motor ports ─────────────────────────────
#define TOP_LEFT_PORT     1   // ← change me
#define TOP_RIGHT_PORT    2   // ← change me
#define BOTTOM_LEFT_PORT  3   // ← change me
#define BOTTOM_RIGHT_PORT 4   // ← change me

// ── Sensor ports ────────────────────────────
#define IMU_PORT            5 // ← change me
#define VERT_ROTATION_PORT  6 // ← change me (forward/back tracking wheel)
#define HORIZ_ROTATION_PORT 7 // ← change me (left/right tracking wheel)
```

---

## Step 2 — Set your tracking wheel measurements

Still in [`include/xdrive_config.h`](include/xdrive_config.h), set your wheel diameter and offsets:

```cpp
// ── Tracking wheel measurements (inches) ────
#define VERT_WHEEL_DIAMETER   2.75f  // wheel diameter — 2.75" is VEX standard
#define VERT_WHEEL_OFFSET     2.0f   // ← how far RIGHT of robot center (inches)

#define HORIZ_WHEEL_DIAMETER  2.75f
#define HORIZ_WHEEL_OFFSET   -1.5f   // ← how far FORWARD of robot center (negative = behind)
```

### How to measure the offset

```
         FRONT
    ┌────────────┐
    │            │
    │  ←2.0"→   │  ← vertical tracking wheel sits 2" to the right of center
    │     |      │
    │  (center)  │
    │      ↑1.5" │  ← horizontal wheel sits 1.5" behind center → offset = -1.5
    │     ---    │
    └────────────┘
```

- **Vertical wheel offset**: positive = wheel is to the **right** of the robot's center
- **Horizontal wheel offset**: positive = wheel is **in front of** the robot's center

---

## Step 3 — Set your PID gains

Open [`src/xdrive_config.cpp`](src/xdrive_config.cpp) and find this block:

```cpp
// ── PID Gains ───────────────────────────────────────────────────────────────
//                          kP     kI    kD    windupRange
static xdrive::PIDGains lateralGains = { 4.0f, 0.0f, 1.5f, 0.0f };
static xdrive::PIDGains angularGains = { 3.0f, 0.0f, 0.5f, 0.0f };
```

### What each gain does

| Gain | Effect | Start here |
|---|---|---|
| `kP` | How hard the robot drives toward the target. Too high = oscillation. | `4.0` lateral, `3.0` angular |
| `kI` | Corrects lingering error. Leave at `0` until everything else is tuned. | `0.0` |
| `kD` | Dampens oscillation. Increase if the robot overshoots. | `1.5` lateral, `0.5` angular |
| `windupRange` | Only accumulates `kI` when error is smaller than this (inches/degrees). Leave `0` while `kI = 0`. | `0.0` |

### Tuning order (recommended)
1. Set all gains to `0`. Set `kP` lateral only and drive to a point. Increase until it oscillates, then back off 20%.
2. Add `kD` lateral to smooth out the overshoot.
3. Repeat for `kP` / `kD` angular using `turnToHeading()`.
4. Only add `kI` if the robot consistently stops a little short.

---

## Step 4 — Set your starting pose

```cpp
chassis.setPose(0, 0, 0);
//              x  y  heading (degrees)
//                     0° = facing the +y direction (forward)
//                     90° = facing right
//                     180° = facing backward
```

Change `x`, `y`, and heading to wherever your robot starts on the field.

---

## Writing autonomous routines

Put your autonomous code in `autonomous()` in [`src/main.cpp`](src/main.cpp).

### Move to a point (holds current heading)
```cpp
chassis.moveToPoint(24, 48, 3000); // target x, target y, timeout ms
```

### Turn to a heading
```cpp
chassis.turnToHeading(90, 1500);  // degrees CW from north, timeout ms
chassis.turnToPoint(24, 48, 1500); // or just face a field coordinate
```

### Move and arrive at a specific angle (boomerang)
```cpp
xdrive::MoveToPoseParams p;
p.lead = 0.5f; // 0 = straight, 0.5 = smooth curve, 1.0 = wide arc

chassis.moveToPose(24, 48, 90, 3000, p);
//                  x   y   end-heading, timeout, params
```

### Motion chaining (don't fully stop between moves)
```cpp
xdrive::MoveToPointParams chain;
chain.earlyExitRange  = 5.0f;  // exit when within 5 inches of target
chain.minLateralSpeed = 40.0f; // carry momentum — don't slow below this

chassis.moveToPoint(0,  24, 2000, chain); // exits early →
chassis.moveToPoint(24, 24, 2000);        // flows straight into this
```

### Drive backward
```cpp
xdrive::MoveToPointParams p;
p.forwards = false; // robot approaches the point with its back

chassis.moveToPoint(0, 0, 2000, p);
```

---

## Coordinate system reference

```
         0° (North / forward)
              +y
              ↑
              │
  270° ←──── 0 ────→ +x  90° (East)
              │
              ↓ 180° (South)
```

- **x** increases to the right
- **y** increases forward
- **heading** is clockwise from North (0° = forward, 90° = right)
- All distances are in **inches**
