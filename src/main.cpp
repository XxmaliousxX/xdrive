#include "main.h"
#include "xdrive_config.h"
#include <cmath>
#include <algorithm>
#include <string>

// ── Drive curve ─────────────────────────────────────────────────────────────
static float driveCurve(float input, float deadband = 5.f,
                         float minOutput = 15.f, float gain = 1.3f) {
    if (std::fabs(input) <= deadband) return 0.f;
    float sign   = input > 0.f ? 1.f : -1.f;
    float x      = (std::fabs(input) - deadband) / (127.f - deadband);
    float curved = std::pow(x, gain);
    return sign * (minOutput + (127.f - minOutput) * curved);
}

// ── X-drive mixer ───────────────────────────────────────────────────────────
// fwd:    left stick Y  (+127 = forward)
// strafe: left stick X  (+127 = right)
// turn:   right stick X (+127 = clockwise)
//
// All four values are scaled together so no motor ever exceeds 127.
static void xDrive(float fwd, float strafe, float turn) {
    float tl = fwd + strafe + turn;
    float tr = fwd - strafe - turn;
    float bl = fwd - strafe + turn;
    float br = fwd + strafe - turn;

    float maxVal = std::max({std::fabs(tl), std::fabs(tr),
                             std::fabs(bl), std::fabs(br), 127.f});
    float scale = 127.f / maxVal;

    top_left    .move((int)(tl * scale));
    top_right   .move((int)(tr * scale));
    bottom_left .move((int)(bl * scale));
    bottom_right.move((int)(br * scale));
}

// ── initialize ──────────────────────────────────────────────────────────────
void initialize() {
    // Calibrate IMU — this blocks until done (typically ~2 seconds)
    imu.reset(true); // true = block until calibration is complete

    // Set the robot's starting pose (x, y, heading in degrees)
    // 0, 0, 0 = origin, facing forward (+y direction)
    chassis.setPose(0, 0, 0);

    // Start the odometry background task at 10 ms (100 Hz)
    odom.startTask(10);
}

// ── autonomous ──────────────────────────────────────────────────────────────
void autonomous() {
    // Write your autonomous routine here using the chassis object.
    // Example:
    //   chassis.moveToPoint(0, 24, 2000);
    //   chassis.turnToHeading(90, 1500);
}

// ── opcontrol ───────────────────────────────────────────────────────────────
void opcontrol() {

    uint32_t lastTempCheck    = 0;
    uint32_t lastBatteryCheck = 0;
    bool     lowBatteryWarned = false;

    // --- IMU Correction Variables ---
    // We use get_rotation() because it returns total degrees (e.g. 720) instead of wrapping at 360
    float targetHeading = imu.get_rotation(); 
    float kP = 3.0f; // TUNE THIS: Start at 1.0, increase if it drifts, decrease if it oscillates/shakes

    while (true) {
        // ── Drive ──────────────────────────────────────────────────────────
        float fwd    = driveCurve( master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
        float strafe = driveCurve( master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
        float turn   = driveCurve( master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));

        // ── IMU Heading Correction Logic ───────────────────────────────────
        if (std::fabs(turn) > 0) {
            // If the driver is actively commanding a turn, update our target to where we are currently facing.
            targetHeading = imu.get_rotation();
        } 
        else if (std::fabs(fwd) > 0 || std::fabs(strafe) > 0) {
            // If the driver is NOT turning, but IS trying to drive straight/strafe,
            // we calculate the error between where we want to point and where we are actually pointing.
            float currentHeading = imu.get_rotation();
            float error = targetHeading - currentHeading;
            
            // Apply Proportional control to auto-correct our heading
            float correction = error * kP;
            
            // Feed this correction into our turn variable
            turn = correction;
        } 
        else {
            // If the robot is completely stopped, keep updating the target heading.
            // This prevents the bot from violently snapping back if it gets pushed while sitting still.
            targetHeading = imu.get_rotation();
        }

        xDrive(fwd, strafe, turn);

        // ── Motor temp warning (every 2 s) ─────────────────────────────────
        if (pros::millis() - lastTempCheck > 2000) {
            lastTempCheck = pros::millis();

            double maxTemp = 0;
            std::string hotMotor;

            auto checkMotor = [&](pros::Motor& m, const char* name) {
                double t = m.get_temperature();
                if (t > maxTemp) { maxTemp = t; hotMotor = name; }
            };
            checkMotor(top_left,     "TL");
            checkMotor(top_right,    "TR");
            checkMotor(bottom_left,  "BL");
            checkMotor(bottom_right, "BR");

            if (maxTemp >= 50)
                master.print(0, 0, "HOT: %s %.0fC   ", hotMotor.c_str(), maxTemp);
        }

        // ── Low battery warning (every 5 s) ───────────────────────────────
        if (pros::millis() - lastBatteryCheck > 5000) {
            lastBatteryCheck = pros::millis();
            int batt = pros::battery::get_capacity();
            if (batt <= 10 && !lowBatteryWarned) {
                master.rumble("---");
                master.print(1, 0, "LOW BATTERY: %d%%", batt);
                lowBatteryWarned = true;
            }
        }

        pros::delay(20);
    }
}