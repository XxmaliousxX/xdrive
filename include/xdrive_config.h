#pragma once
#include "pros/motors.hpp"
#include "pros/imu.hpp"
#include "pros/adi.hpp"
#include "pros/misc.hpp"

// ── Motor ports (change to match your wiring) ──────────────────────────────
#define TOP_LEFT_PORT     1
#define TOP_RIGHT_PORT    2
#define BOTTOM_LEFT_PORT  3
#define BOTTOM_RIGHT_PORT 4


extern pros::Motor top_left;
extern pros::Motor top_right;
extern pros::Motor bottom_left;
extern pros::Motor bottom_right;

extern pros::Controller master;

void initializeRobot();