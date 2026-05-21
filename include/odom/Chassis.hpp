#pragma once
#include "util.hpp"
#include "PID.hpp"
#include "Odom.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <optional>
#include <atomic>

namespace xdrive {

// ─────────────────────────────────────────────────────────────────────────────
// Parameter structs
// All speeds are in raw motor units [0, 127].
// ─────────────────────────────────────────────────────────────────────────────

struct MoveToPointParams {
    bool  forwards        = true;  // false = robot approaches with its back
    float maxLateralSpeed = 127;   // max lateral drive power
    float minLateralSpeed = 0;     // min lateral drive power (0 = no minimum)
                                   //   set > 0 for consistent motion chaining speed
    float maxAngularSpeed = 40;    // max heading-correction power
    float earlyExitRange  = 0;     // exit when within X inches of target (motion chaining)
                                   //   only active when minLateralSpeed > 0
    float lateralSlew     = 0;     // max lateral power change per second (0 = unlimited)
    float angularSlew     = 0;     // max angular power change per second (0 = unlimited)
    float settleRange     = 1.0f;  // inches — exit when within this distance and settled
};

struct TurnParams {
    int   direction      = 0;     // 0 = shortest path, 1 = force CW, -1 = force CCW
    float maxSpeed       = 127;
    float minSpeed       = 0;
    float earlyExitRange = 0;     // degrees — for motion chaining
    float slew           = 0;     // max speed change per second
    float settleRange    = 1.0f;  // degrees — exit when within this angle and settled
};

struct MoveToPoseParams {
    bool  forwards        = true;
    float maxLateralSpeed = 127;
    float minLateralSpeed = 0;
    float maxAngularSpeed = 127;
    float earlyExitRange  = 0;    // inches
    float lateralSlew     = 0;
    float lead            = 0.5f; // boomerang carrot lead factor [0, 1]
                                  //   0 = drive straight to target then turn
                                  //   0.5 = balanced curved approach (recommended)
                                  //   1 = very wide arc
    float settleRange     = 1.0f; // inches
    float settleAngle     = 2.0f; // degrees
};

// ─────────────────────────────────────────────────────────────────────────────
// XDriveChassis
// ─────────────────────────────────────────────────────────────────────────────
class XDriveChassis {
public:
    /**
     * @param topLeft/topRight/bottomLeft/bottomRight  Drive motors (pass by reference,
     *        make sure reversed flags are set correctly in robot_config.cpp)
     * @param odom         Odometry instance (startTask() must be called separately)
     * @param lateralGains PID gains for forward/strafe movement
     * @param angularGains PID gains for turning and heading correction
     */
    XDriveChassis(pros::Motor& topLeft,
                  pros::Motor& topRight,
                  pros::Motor& bottomLeft,
                  pros::Motor& bottomRight,
                  Odometry&    odom,
                  PIDGains     lateralGains,
                  PIDGains     angularGains);

    // Hot-swap PID gains without reconstructing the chassis (useful for tuning)
    void setLateralGains(PIDGains gains);
    void setAngularGains(PIDGains gains);

    // ── Pose ──────────────────────────────────────────────────────────────────
    void setPose(float x, float y, float theta);
    Pose getPose();

    // ── Motions (all blocking by default) ────────────────────────────────────

    /**
     * Move to (x, y) while maintaining the robot's current heading.
     * The robot strafes diagonally — no pre-turn required.
     */
    void moveToPoint(float x, float y, float timeoutMs,
                     MoveToPointParams params = {});

    /**
     * Turn in place to an absolute heading (degrees, CW from north).
     */
    void turnToHeading(float heading, float timeoutMs,
                       TurnParams params = {});

    /**
     * Turn in place to face a field point.
     */
    void turnToPoint(float x, float y, float timeoutMs,
                     TurnParams params = {});

    /**
     * Move to (x, y) AND finish at heading theta (degrees).
     * Uses a boomerang carrot approach for a smooth curved path.
     */
    void moveToPose(float x, float y, float theta, float timeoutMs,
                    MoveToPoseParams params = {});

    // ── Motion state ──────────────────────────────────────────────────────────

    // Block until the current motion finishes
    void waitUntilDone();

    // Block until the robot has traveled >= inches from the start of the current motion.
    // Useful when running a motion inside a separate pros::Task.
    void waitUntilDist(float inches);

    // Returns true if a motion task is currently running
    bool isMoving();

    // Interrupt and stop the current motion immediately
    void cancelMotion();

private:
    // Apply fwd/strafe/turn, proportionally scaled so no motor exceeds 127
    void xDriveMove(float fwd, float strafe, float turn);
    void brake();

    // Compute angular error with optional forced direction
    float directedAngleError(float target, float current, int direction);

    pros::Motor& m_tl;
    pros::Motor& m_tr;
    pros::Motor& m_bl;
    pros::Motor& m_br;
    Odometry&    m_odom;

    PIDGains m_lateralGains;
    PIDGains m_angularGains;

    // Distance traveled since the start of the current motion (inches)
    std::atomic<float> m_distTraveled{0.0f};
    Pose               m_motionStart;

    std::optional<pros::Task> m_motionTask;
};

} // namespace xdrive
