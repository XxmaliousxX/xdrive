#include "xdrive/Chassis.hpp"
#include "xdrive/util.hpp"
#include <cmath>
#include <algorithm>

namespace xdrive {

// ─────────────────────────────────────────────────────────────────────────────
// Constructor / config
// ─────────────────────────────────────────────────────────────────────────────

XDriveChassis::XDriveChassis(pros::Motor& tl, pros::Motor& tr,
                             pros::Motor& bl, pros::Motor& br,
                             Odometry&    odom,
                             PIDGains     lateralGains,
                             PIDGains     angularGains)
    : m_tl(tl), m_tr(tr), m_bl(bl), m_br(br),
      m_odom(odom),
      m_lateralGains(lateralGains),
      m_angularGains(angularGains) {}

void XDriveChassis::setLateralGains(PIDGains gains) { m_lateralGains = gains; }
void XDriveChassis::setAngularGains(PIDGains gains) { m_angularGains = gains; }

void XDriveChassis::setPose(float x, float y, float theta) {
    m_odom.setPose({x, y, theta});
}
Pose XDriveChassis::getPose() { return m_odom.getPose(); }

// ─────────────────────────────────────────────────────────────────────────────
// Low-level drive helpers
// ─────────────────────────────────────────────────────────────────────────────

void XDriveChassis::xDriveMove(float fwd, float strafe, float turn) {
    float tl = fwd + strafe + turn;
    float tr = fwd - strafe - turn;
    float bl = fwd - strafe + turn;
    float br = fwd + strafe - turn;

    // Scale all four proportionally so the largest value is exactly 127
    float maxVal = std::max({std::fabs(tl), std::fabs(tr),
                             std::fabs(bl), std::fabs(br), 127.0f});
    float scale = 127.0f / maxVal;

    m_tl.move((int)(tl * scale));
    m_tr.move((int)(tr * scale));
    m_bl.move((int)(bl * scale));
    m_br.move((int)(br * scale));
}

void XDriveChassis::brake() {
    m_tl.move(0);
    m_tr.move(0);
    m_bl.move(0);
    m_br.move(0);
}

float XDriveChassis::directedAngleError(float target, float current, int direction) {
    float err = angleError(target, current); // in [-180, 180]
    if (direction ==  1 && err < 0) err += 360.0f; // force CW (positive)
    if (direction == -1 && err > 0) err -= 360.0f; // force CCW (negative)
    return err;
}

// ─────────────────────────────────────────────────────────────────────────────
// Motion state
// ─────────────────────────────────────────────────────────────────────────────

bool XDriveChassis::isMoving() {
    if (!m_motionTask) return false;
    uint32_t state = m_motionTask->get_state();
    return state != pros::E_TASK_STATE_DELETED &&
           state != pros::E_TASK_STATE_INVALID;
}

void XDriveChassis::cancelMotion() {
    if (isMoving()) m_motionTask->notify();
}

void XDriveChassis::waitUntilDone() {
    while (isMoving()) pros::delay(5);
}

void XDriveChassis::waitUntilDist(float inches) {
    while (isMoving()) {
        if (m_distTraveled.load() >= inches) return;
        pros::delay(5);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// moveToPoint
//
// Decompose the field-relative error vector into robot-relative fwd/strafe,
// then drive both simultaneously while a second PID corrects any heading drift.
//
// X-drive kinematics let us go straight to any point without pre-turning.
// ─────────────────────────────────────────────────────────────────────────────

void XDriveChassis::moveToPoint(float tx, float ty, float timeoutMs,
                                MoveToPointParams params) {
    waitUntilDone();

    m_motionStart = m_odom.getPose();
    m_distTraveled.store(0.0f);

    m_motionTask = pros::Task([=]() mutable {
        PID lateralPID(m_lateralGains);
        PID angularPID(m_angularGains);

        float prevLateralOut = 0;
        float prevAngularOut = 0;
        bool  settled        = false;

        // Lock in the heading we want to maintain throughout this move
        float targetHeading = m_odom.getPose().theta;

        uint32_t startTime = pros::millis();
        uint32_t prevTime  = startTime;

        while (pros::Task::notify_take(true, 0) == 0) {
            uint32_t now = pros::millis();
            float dt = (now - prevTime) / 1000.0f;
            if (dt <= 0.0f) dt = 0.01f;
            prevTime = now;

            // ── Timeout ──────────────────────────────────────────────────────
            if (now - startTime > (uint32_t)timeoutMs) break;

            Pose pose = m_odom.getPose();

            // ── Distance to target ───────────────────────────────────────────
            float dx   = tx - pose.x;
            float dy   = ty - pose.y;
            float dist = sqrtf(dx * dx + dy * dy);

            // Track how far we've come from the start of this motion
            m_distTraveled.store(distance(m_motionStart.x, m_motionStart.y,
                                          pose.x, pose.y));

            // ── Exit conditions ───────────────────────────────────────────────
            // Motion chaining: exit early so the robot carries momentum into the next motion
            if (params.earlyExitRange > 0 &&
                params.minLateralSpeed > 0 &&
                dist < params.earlyExitRange) break;

            if (dist < 7.5f) settled = true;
            if (settled && dist < params.settleRange) break;

            // ── Decompose error into robot frame ──────────────────────────────
            // Field vector (dx, dy) rotated into robot frame:
            //   robotFwd   = dx*sin(θ) + dy*cos(θ)   (component along robot's forward axis)
            //   robotRight = dx*cos(θ) - dy*sin(θ)   (component along robot's right axis)
            // Verified:
            //   θ=0 (facing +y): fwd=dy, right=dx ✓
            //   θ=90 (facing +x): fwd=dx, right=-dy ✓
            float theta     = toRad(pose.theta);
            float robotFwd  = dx * sinf(theta) + dy * cosf(theta);
            float robotRight= dx * cosf(theta) - dy * sinf(theta);

            float sign = params.forwards ? 1.0f : -1.0f;

            // ── Lateral PID on total distance ────────────────────────────────
            float lateralOut = lateralPID.update(dist, dt);
            lateralOut = clamp(lateralOut, -params.maxLateralSpeed, params.maxLateralSpeed);

            if (params.lateralSlew > 0)
                lateralOut = slewRate(lateralOut, prevLateralOut, params.lateralSlew, dt);

            // Enforce minimum speed (skip when settling so we don't overshoot)
            if (!settled && params.minLateralSpeed > 0) {
                if (lateralOut > 0 && lateralOut < params.minLateralSpeed)
                    lateralOut = params.minLateralSpeed;
                if (lateralOut < 0 && lateralOut > -params.minLateralSpeed)
                    lateralOut = -params.minLateralSpeed;
            }
            prevLateralOut = lateralOut;

            // Decompose into fwd/strafe by projecting the unit error vector
            float fwd    = sign * lateralOut * (robotFwd   / dist);
            float strafe = sign * lateralOut * (robotRight / dist);

            // ── Heading correction ────────────────────────────────────────────
            float headingErr = angleError(targetHeading, pose.theta);
            float turn = angularPID.update(headingErr, dt);
            turn = clamp(turn, -params.maxAngularSpeed, params.maxAngularSpeed);
            if (params.angularSlew > 0)
                turn = slewRate(turn, prevAngularOut, params.angularSlew, dt);
            prevAngularOut = turn;

            xDriveMove(fwd, strafe, turn);
            pros::delay(10);
        }

        brake();
    });

    waitUntilDone();
}

// ─────────────────────────────────────────────────────────────────────────────
// turnToHeading
// ─────────────────────────────────────────────────────────────────────────────

void XDriveChassis::turnToHeading(float heading, float timeoutMs,
                                  TurnParams params) {
    waitUntilDone();

    m_motionStart = m_odom.getPose();
    m_distTraveled.store(0.0f);

    m_motionTask = pros::Task([=]() mutable {
        PID angularPID(m_angularGains);

        float prevOut  = 0;
        bool  settled  = false;
        bool  prevSign = true; // track sign of error for overshoot detection

        uint32_t startTime = pros::millis();
        uint32_t prevTime  = startTime;

        while (pros::Task::notify_take(true, 0) == 0) {
            uint32_t now = pros::millis();
            float dt = (now - prevTime) / 1000.0f;
            if (dt <= 0.0f) dt = 0.01f;
            prevTime = now;

            if (now - startTime > (uint32_t)timeoutMs) break;

            Pose  pose = m_odom.getPose();
            float err  = directedAngleError(heading, pose.theta, params.direction);

            // Motion chaining: exit before fully settling
            if (params.earlyExitRange > 0 &&
                params.minSpeed > 0 &&
                std::fabs(err) < params.earlyExitRange) break;

            // Overshoot detection — if sign flipped, we've passed the target
            bool currSign = (err >= 0);
            if (currSign != prevSign) settled = true;
            prevSign = currSign;

            if (std::fabs(err) < 5.0f) settled = true;
            if (settled && std::fabs(err) < params.settleRange) break;

            float out = angularPID.update(err, dt);
            out = clamp(out, -params.maxSpeed, params.maxSpeed);
            if (params.slew > 0) out = slewRate(out, prevOut, params.slew, dt);

            if (!settled && params.minSpeed > 0) {
                if (out > 0 && out < params.minSpeed) out = params.minSpeed;
                if (out < 0 && out > -params.minSpeed) out = -params.minSpeed;
            }
            prevOut = out;

            // fwd=0, strafe=0 — pure rotation
            xDriveMove(0, 0, out);
            pros::delay(10);
        }

        brake();
    });

    waitUntilDone();
}

// ─────────────────────────────────────────────────────────────────────────────
// turnToPoint
// ─────────────────────────────────────────────────────────────────────────────

void XDriveChassis::turnToPoint(float x, float y, float timeoutMs,
                                TurnParams params) {
    // Calculate the heading that points from current position toward (x, y)
    Pose  pose = m_odom.getPose();
    float dx   = x - pose.x;
    float dy   = y - pose.y;
    // atan2(x_component, y_component) gives angle from +y axis, CW positive
    float targetHeading = toDeg(atan2f(dx, dy));
    if (targetHeading < 0) targetHeading += 360.0f;

    turnToHeading(targetHeading, timeoutMs, params);
}

// ─────────────────────────────────────────────────────────────────────────────
// moveToPose
//
// Boomerang controller — uses a "carrot point" ahead of the target to create
// a curved approach path. The robot simultaneously drives toward the carrot
// and rotates toward the target heading. As distance shrinks, the carrot
// converges to the target, producing a smooth finish.
// ─────────────────────────────────────────────────────────────────────────────

void XDriveChassis::moveToPose(float tx, float ty, float tTheta,
                               float timeoutMs, MoveToPoseParams params) {
    waitUntilDone();

    m_motionStart = m_odom.getPose();
    m_distTraveled.store(0.0f);

    m_motionTask = pros::Task([=]() mutable {
        PID lateralPID(m_lateralGains);
        PID angularPID(m_angularGains);

        float prevLateralOut = 0;
        bool  settled        = false;

        uint32_t startTime = pros::millis();
        uint32_t prevTime  = startTime;

        while (pros::Task::notify_take(true, 0) == 0) {
            uint32_t now = pros::millis();
            float dt = (now - prevTime) / 1000.0f;
            if (dt <= 0.0f) dt = 0.01f;
            prevTime = now;

            if (now - startTime > (uint32_t)timeoutMs) break;

            Pose  pose = m_odom.getPose();
            float dx   = tx - pose.x;
            float dy   = ty - pose.y;
            float dist = sqrtf(dx * dx + dy * dy);

            m_distTraveled.store(distance(m_motionStart.x, m_motionStart.y,
                                          pose.x, pose.y));

            // ── Exit conditions ────────────────────────────────────────────────
            float angErr = angleError(tTheta, pose.theta);

            if (params.earlyExitRange > 0 &&
                params.minLateralSpeed > 0 &&
                dist < params.earlyExitRange) break;

            if (dist < 7.5f) settled = true;
            if (settled &&
                dist < params.settleRange &&
                std::fabs(angErr) < params.settleAngle) break;

            // ── Carrot point ───────────────────────────────────────────────────
            // Place the carrot along the line coming INTO the target from behind,
            // at a distance of lead * dist from the target.
            // When settled we drive directly to the target (carrot == target).
            float carrotX = tx, carrotY = ty;
            if (!settled) {
                float tThetaRad = toRad(tTheta);
                // Target heading points in direction (sin, cos). The carrot is
                // placed opposite — i.e. behind the target from the robot's perspective.
                carrotX = tx - params.lead * dist * sinf(tThetaRad);
                carrotY = ty - params.lead * dist * cosf(tThetaRad);
            }

            float cdx        = carrotX - pose.x;
            float cdy        = carrotY - pose.y;
            float carrotDist = sqrtf(cdx * cdx + cdy * cdy);

            // ── Robot-frame decomposition toward carrot ────────────────────────
            float theta     = toRad(pose.theta);
            float robotFwd  = cdx * sinf(theta) + cdy * cosf(theta);
            float robotRight= cdx * cosf(theta) - cdy * sinf(theta);
            float sign      = params.forwards ? 1.0f : -1.0f;

            // ── Lateral PID (drive toward target, not carrot, for smooth decel) ─
            float lateralOut = 0;
            if (carrotDist > 0.1f) {
                lateralOut = lateralPID.update(dist, dt);
                lateralOut = clamp(lateralOut, -params.maxLateralSpeed, params.maxLateralSpeed);
                if (params.lateralSlew > 0)
                    lateralOut = slewRate(lateralOut, prevLateralOut, params.lateralSlew, dt);
                if (!settled && params.minLateralSpeed > 0) {
                    if (lateralOut > 0 && lateralOut < params.minLateralSpeed)
                        lateralOut = params.minLateralSpeed;
                    if (lateralOut < 0 && lateralOut > -params.minLateralSpeed)
                        lateralOut = -params.minLateralSpeed;
                }
                prevLateralOut = lateralOut;
            }

            float invCarrotDist = (carrotDist > 0.01f) ? 1.0f / carrotDist : 0.0f;
            float fwd    = sign * lateralOut * robotFwd   * invCarrotDist;
            float strafe = sign * lateralOut * robotRight * invCarrotDist;

            // ── Angular PID — always target the final pose heading ─────────────
            // Prioritize angular when close (settled) — slow lateral, full turn.
            float maxAng = settled ? params.maxAngularSpeed : params.maxAngularSpeed * 0.6f;
            float turn = angularPID.update(angErr, dt);
            turn = clamp(turn, -maxAng, maxAng);

            xDriveMove(fwd, strafe, turn);
            pros::delay(10);
        }

        brake();
    });

    waitUntilDone();
}

} // namespace xdrive
