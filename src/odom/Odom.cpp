#include "Odom.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include <mutex>

namespace xdrive {

Odometry::Odometry(pros::Imu* imu,
                   std::optional<TrackingWheelConfig> vertWheel,
                   std::optional<TrackingWheelConfig> horizWheel)
    : m_imu(imu), m_vert(vertWheel), m_horiz(horizWheel) {}

// pros::Rotation::get_position() returns centidegrees (1/100 of a degree)
// distance = (total_degrees / 360) * π * diameter
float Odometry::getWheelDist(pros::Rotation& sensor, float diameter) {
    return (sensor.get_position() / 36000.0f) * M_PI * diameter;
}

void Odometry::setPose(Pose pose) {
    std::lock_guard<pros::Mutex> lock(m_mutex);
    // Recalculate IMU offset so that the current IMU reading maps to pose.theta.
    // pros::Imu::get_rotation() returns signed accumulated rotation, CCW positive.
    // We negate it to get CW positive (matching our coordinate system).
    if (m_imu) {
        m_imuOffset = pose.theta + m_imu->get_rotation();
    }
    m_pose = pose;
}

Pose Odometry::getPose() {
    std::lock_guard<pros::Mutex> lock(m_mutex);
    return m_pose;
}

void Odometry::startTask(uint32_t period) {
    // Zero out tracking wheels so deltas start from 0
    if (m_vert)  { m_vert->sensor->reset_position();  m_vertLast  = 0; }
    if (m_horiz) { m_horiz->sensor->reset_position(); m_horizLast = 0; }

    // Set initial IMU offset from the current pose
    if (m_imu) m_imuOffset = m_pose.theta + m_imu->get_rotation();

    m_task = pros::Task([this, period] { taskFn(period); });
}

void Odometry::taskFn(uint32_t period) {
    uint32_t prevTime = pros::millis();

    while (true) {
        // ── Step 1: Get tracking wheel deltas ────────────────────────────────
        float dVert = 0, dHoriz = 0;
        float vertOffset = 0, horizOffset = 0;

        if (m_vert) {
            float vertDist = getWheelDist(*m_vert->sensor, m_vert->diameter);
            dVert      = vertDist - m_vertLast;
            m_vertLast = vertDist;
            vertOffset = m_vert->offset;
        }
        if (m_horiz) {
            float horizDist = getWheelDist(*m_horiz->sensor, m_horiz->diameter);
            dHoriz      = horizDist - m_horizLast;
            m_horizLast = horizDist;
            horizOffset = m_horiz->offset;
        }

        // ── Step 2: Get heading ───────────────────────────────────────────────
        // pros::Imu::get_rotation() = signed accumulated rotation, CCW positive.
        // We want CW positive, so negate it, then add offset.
        float theta = m_pose.theta; // fallback: no change
        if (m_imu) {
            theta = m_imuOffset - m_imu->get_rotation();
            // Normalize to [0, 360)
            theta = fmodf(theta, 360.0f);
            if (theta < 0) theta += 360.0f;
        }

        // ── Step 3: Calculate heading change ─────────────────────────────────
        float dTheta = theta - m_pose.theta;
        // Wrap to [-180, 180] to handle 0/360 crossing
        if (dTheta >  180.0f) dTheta -= 360.0f;
        if (dTheta < -180.0f) dTheta += 360.0f;
        float dThetaRad = dTheta * M_PI / 180.0f;

        // ── Step 4: Local displacement (5225A algorithm) ──────────────────────
        // When there's no rotation, displacements are just the raw deltas.
        // When rotating, account for the arc swept during the rotation.
        float localX, localY;
        if (std::fabs(dThetaRad) < 1e-9f) {
            localX = dHoriz;
            localY = dVert;
        } else {
            float sinHalf = 2.0f * sinf(dThetaRad / 2.0f);
            localX = sinHalf * (dHoriz / dThetaRad + horizOffset);
            localY = sinHalf * (dVert  / dThetaRad + vertOffset);
        }

        // ── Step 5: Rotate local displacement to global frame ─────────────────
        // avgTheta = heading at the midpoint of this update step
        // With theta=0 facing +y and CW positive:
        //   globalX += localY * sin(avgTheta) + localX * cos(avgTheta)
        //   globalY += localY * cos(avgTheta) - localX * sin(avgTheta)
        float avgThetaRad = toRad(m_pose.theta + dTheta / 2.0f);
        float globalDX = localY * sinf(avgThetaRad) + localX * cosf(avgThetaRad);
        float globalDY = localY * cosf(avgThetaRad) - localX * sinf(avgThetaRad);

        // ── Step 6: Update pose ───────────────────────────────────────────────
        {
            std::lock_guard<pros::Mutex> lock(m_mutex);
            m_pose.x     += globalDX;
            m_pose.y     += globalDY;
            m_pose.theta  = theta;
        }

        pros::Task::delay_until(&prevTime, period);
    }
}

} // namespace xdrive
