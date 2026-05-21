#pragma once
#include "pros/motors.hpp"
#include "pros/imu.hpp"
#include "pros/rotation.hpp"
#include "pros/adi.hpp"
#include "pros/misc.hpp"
#include "odom/Chassis.hpp"

// ── Motor ports (change to match your wiring) ──────────────────────────────
#define TOP_LEFT_PORT     1
#define TOP_RIGHT_PORT    2
#define BOTTOM_LEFT_PORT  3
#define BOTTOM_RIGHT_PORT 4

// ── Sensor ports ────────────────────────────────────────────────────────────
#define IMU_PORT          5
#define VERT_ROTATION_PORT  6
#define HORIZ_ROTATION_PORT 7

// ── Tracking wheel measurements (inches) ────────────────────────────────────
// diameter: the diameter of your tracking wheel (e.g., 2.75 for standard)
// offset:   distance from the robot's tracking center
//   vertical wheel:   positive = wheel is to the RIGHT of center
//   horizontal wheel: positive = wheel is FORWARD of center
#define VERT_WHEEL_DIAMETER   2.75f
#define VERT_WHEEL_OFFSET     2.0f   // TODO: measure on your robot
#define HORIZ_WHEEL_DIAMETER  2.75f
#define HORIZ_WHEEL_OFFSET   -1.5f   // TODO: measure on your robot

extern pros::Motor top_left;
extern pros::Motor top_right;
extern pros::Motor bottom_left;
extern pros::Motor bottom_right;

extern pros::Imu      imu;
extern pros::Rotation  vert_rotation;
extern pros::Rotation  horiz_rotation;

extern pros::Controller master;

extern xdrive::Odometry       odom;
extern xdrive::XDriveChassis   chassis;

