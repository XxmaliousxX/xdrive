#include "xdrive_config.h"

// Positive = forward for all four. Flip with true/false to match your wiring.
pros::Motor top_left    (TOP_LEFT_PORT,     pros::MotorGears::blue, pros::MotorUnits::degrees);
pros::Motor top_right   (TOP_RIGHT_PORT,    pros::MotorGears::blue, pros::MotorUnits::degrees);
pros::Motor bottom_left (BOTTOM_LEFT_PORT,  pros::MotorGears::blue, pros::MotorUnits::degrees);
pros::Motor bottom_right(BOTTOM_RIGHT_PORT, pros::MotorGears::blue, pros::MotorUnits::degrees);



pros::Controller master(pros::E_CONTROLLER_MASTER);

void initializeRobot() {
    // Reverse motors that spin "backwards" relative to forward on your bot.
    // Typically the right side motors need reversing — adjust to match.
    top_right.set_reversed(true);
    bottom_right.set_reversed(true);

}