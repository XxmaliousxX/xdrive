#include "subsystems/outtake.h"
#include "subsystems/intake.h"
#include "robot_config.h"

OuttakeControl::OuttakeControl()
    : comboMode(false),
      L1_lastState(false),
      midScoringMode(false), X_lastState(false),
      unjamStartTime(0), isUnjamming(false) {}

void OuttakeControl::update(IntakeControl& intake) {
    // Handle unjam sequence
    if (isUnjamming) {
        if (pros::millis() - unjamStartTime >= 100) {  // Time of delay
            isUnjamming = false;
            // Don't set intake here - let it fall through to mid-scoring mode logic below
        } else {
            Intake.move_velocity(600);   // Unjam (reverse) - velocity control
            Outtake.move_velocity(-600); // Make sure outtake also runs during unjam
            return; // Exit early during unjam
        }
    }

    // Mid Scoring Toggle (Button X)
    bool X_current = master.get_digital(pros::E_CONTROLLER_DIGITAL_X);
    if (X_current && !X_lastState) {
        midScoringMode = !midScoringMode;
        
        if (midScoringMode) {
            // ENTERING mid-scoring mode - turn off combo mode
            comboMode = false;
            MidScoring.set_value(true); // Retract piston
            isUnjamming = true;
            unjamStartTime = pros::millis();
        } else {
            // EXITING mid-scoring mode
            MidScoring.set_value(false); // Extend piston
            isUnjamming = false;
            Intake.move_velocity(0); // Stop intake
            Outtake.move_velocity(0); // Stop outtake
            // Reset toggles so they start fresh
            comboMode = false;
            X_lastState = X_current; // Update state BEFORE returning
            return; // Exit function immediately after turning off mid-scoring
        }
    }
    X_lastState = X_current;

    int velocity = 0;

    if (midScoringMode && !isUnjamming) {
        // Mid-scoring mode AFTER unjam: run intake forward and outtake reverse
        Intake.move_velocity(-600);  // This will run continuously during mid-scoring
        velocity = -600;
        // Update last states to prevent "stored" button presses
        L1_lastState = master.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
    } else if (!midScoringMode) {
        // Normal mode: L1 = combo toggle (outtake reverse + intake forward)
        bool L1_current = master.get_digital(pros::E_CONTROLLER_DIGITAL_L1);

        // L1: Toggle combo mode
        if (L1_current && !L1_lastState) {
            comboMode = !comboMode;
            if (comboMode) {
                // Activating combo - cancel any intake-only toggles
                intake.cancelToggles();
            } else {
                // Deactivating combo - stop both motors
                Intake.move_velocity(0);
                // velocity stays 0, will be applied below
            }
        }
        
        L1_lastState = L1_current;

        // Set motor velocities based on current mode
        if (comboMode) {
            velocity = 600;             // Outtake reverse
            Intake.move_velocity(-600);  // Intake reverse
        } else if (intake.isReverseToggled()) {
            velocity = -600;            // Outtake reverse
        }
        // If combo not active, velocity stays 0 (intake handled by IntakeControl)
    }

    Outtake.move_velocity(velocity);
}

int OuttakeControl::getVelocity() {
    if (midScoringMode) return -600;
    if (comboMode) return 600;
    return 0;
}

bool OuttakeControl::isMidScoring() {
    return midScoringMode;
}

bool OuttakeControl::isComboMode() {
    return comboMode;
}

void OuttakeControl::cancelCombo() {
    comboMode = false;
    Outtake.move_velocity(0); // Stop outtake immediately when combo is cancelled
}
