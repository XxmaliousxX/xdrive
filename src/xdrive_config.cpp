#include "xdrive_config.h"

// ── Drive motors ────────────────────────────────────────────────────────────
// Positive = forward for all four. Flip with true/false to match your wiring.
pros::Motor top_left    (TOP_LEFT_PORT,     pros::MotorGears::blue, pros::MotorUnits::degrees);
pros::Motor top_right   (TOP_RIGHT_PORT,    pros::MotorGears::blue, pros::MotorUnits::degrees);
pros::Motor bottom_left (BOTTOM_LEFT_PORT,  pros::MotorGears::blue, pros::MotorUnits::degrees);
pros::Motor bottom_right(BOTTOM_RIGHT_PORT, pros::MotorGears::blue, pros::MotorUnits::degrees);

// ── Sensors ─────────────────────────────────────────────────────────────────
pros::Imu      imu(IMU_PORT);
pros::Rotation vert_rotation(VERT_ROTATION_PORT);
pros::Rotation horiz_rotation(HORIZ_ROTATION_PORT);

// ── Controller ──────────────────────────────────────────────────────────────
pros::Controller master(pros::E_CONTROLLER_MASTER);

// ── Tracking wheel configs ──────────────────────────────────────────────────
// These structs tell Odometry how to interpret each tracking wheel.
// Adjust the #defines in xdrive_config.h to match your robot.
static xdrive::TrackingWheelConfig vertWheelCfg  = {
    &vert_rotation,
    VERT_WHEEL_DIAMETER,
    VERT_WHEEL_OFFSET
};
static xdrive::TrackingWheelConfig horizWheelCfg = {
    &horiz_rotation,
    HORIZ_WHEEL_DIAMETER,
    HORIZ_WHEEL_OFFSET
};

// ── Odometry ────────────────────────────────────────────────────────────────
xdrive::Odometry odom(&imu, vertWheelCfg, horizWheelCfg);

// ── PID Gains ───────────────────────────────────────────────────────────────
//                          kP     kI    kD    windupRange
// Start with just kP, then add kD once kP is close.
// Only add kI if you have persistent steady-state error.
static xdrive::PIDGains lateralGains = { 4.0f, 0.0f, 1.5f, 0.0f };
static xdrive::PIDGains angularGains = { 3.0f, 0.0f, 0.5f, 0.0f };

// ── Chassis ─────────────────────────────────────────────────────────────────
xdrive::XDriveChassis chassis(
    top_left, top_right, bottom_left, bottom_right,
    odom,
    lateralGains,
    angularGains
);
