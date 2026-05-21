#pragma once
#include "util.hpp"
#include "pros/rotation.hpp"
#include "pros/imu.hpp"
#include "pros/rtos.hpp"
#include "pros/mutex.hpp"
#include <optional>

namespace xdrive {

struct TrackingWheelConfig {
    pros::Rotation* sensor;   // pointer to pros::Rotation sensor
    float diameter;           // wheel diameter in inches
    float offset;             // distance from tracking center in inches
                              //   vertical wheel:   positive offset = right of center
                              //   horizontal wheel: positive offset = forward of center
};

// ─────────────────────────────────────────────────────────────────────────────
// Odometry
//
// Uses the 5225A tracking algorithm (http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf)
// Heading source priority: IMU > two vertical tracking wheels
//
// Minimum sensor setup:
//   - IMU + one vertical tracking wheel  (no strafing tracking)
//   - IMU + one vertical + one horizontal (full 2D tracking — recommended)
// ─────────────────────────────────────────────────────────────────────────────
class Odometry {
public:
    /**
     * @param imu        Pointer to calibrated pros::Imu. Pass nullptr to derive heading
     *                   from tracking wheels (requires two vertical wheels — not yet supported).
     * @param vertWheel  Vertical (forward/back measuring) tracking wheel.
     * @param horizWheel Horizontal (left/right measuring) tracking wheel.
     *                   Without this the robot's strafing is not tracked.
     */
    Odometry(pros::Imu*                          imu,
             std::optional<TrackingWheelConfig>  vertWheel  = std::nullopt,
             std::optional<TrackingWheelConfig>  horizWheel = std::nullopt);

    // Set the robot's current pose. Safe to call at any time.
    void setPose(Pose pose);

    // Get the current pose (thread-safe).
    Pose getPose();

    // Start the background tracking task. Call once in initialize() AFTER imu calibration.
    // period: update interval in milliseconds (default 10 ms = 100 Hz)
    void startTask(uint32_t period = 10);

private:
    void   taskFn(uint32_t period);
    float  getWheelDist(pros::Rotation& sensor, float diameter);

    pros::Imu*                         m_imu;
    std::optional<TrackingWheelConfig> m_vert;
    std::optional<TrackingWheelConfig> m_horiz;

    float m_vertLast   = 0; // last absolute distance from vertical wheel (inches)
    float m_horizLast  = 0; // last absolute distance from horizontal wheel (inches)
    float m_imuOffset  = 0; // added to raw IMU value to get field heading

    Pose       m_pose;
    pros::Mutex m_mutex;
    std::optional<pros::Task> m_task;
};

} // namespace xdrive
