#include "util.hpp"
#include <algorithm>

namespace xdrive {

float angleError(float target, float current) {
    float err = fmodf(target - current, 360.0f);
    if (err > 180.0f)  err -= 360.0f;
    if (err < -180.0f) err += 360.0f;
    return err;
}

float clamp(float val, float minVal, float maxVal) {
    return std::max(minVal, std::min(maxVal, val));
}

float sgn(float val) {
    if (val > 0) return  1.0f;
    if (val < 0) return -1.0f;
    return 0.0f;
}

float toRad(float deg) { return deg * M_PI / 180.0f; }
float toDeg(float rad) { return rad * 180.0f / M_PI;  }

float slewRate(float target, float current, float maxRate, float dt) {
    if (maxRate <= 0.0f) return target;
    float delta    = target - current;
    float maxDelta = maxRate * dt;
    if (delta >  maxDelta) return current + maxDelta;
    if (delta < -maxDelta) return current - maxDelta;
    return target;
}

float distance(float x1, float y1, float x2, float y2) {
    float dx = x2 - x1;
    float dy = y2 - y1;
    return sqrtf(dx * dx + dy * dy);
}

} // namespace xdrive
