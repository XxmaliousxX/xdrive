#pragma once
#include <cmath>

namespace xdrive {

// ── Coordinate system ────────────────────────────────────────────────────────
// x: positive = right
// y: positive = forward
// theta: degrees, 0 = facing +y (north), clockwise positive [0, 360)

struct Pose {
    float x     = 0;
    float y     = 0;
    float theta = 0;
};

// Normalize angular error to [-180, 180]
float angleError(float target, float current);

// Clamp val to [minVal, maxVal]
float clamp(float val, float minVal, float maxVal);

// Sign of val (-1, 0, or 1)
float sgn(float val);

// Degrees <-> radians
float toRad(float deg);
float toDeg(float rad);

// Slew rate limiter — limits how fast a value changes.
// maxRate: max change per second. 0 = unlimited.
float slewRate(float target, float current, float maxRate, float dt);

// Distance between two poses
float distance(float x1, float y1, float x2, float y2);

} // namespace xdrive
