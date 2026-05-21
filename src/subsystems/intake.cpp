#include "subsystems/intake.h"
#include "subsystems/outtake.h"
#include "robot_config.h"

IntakeControl::IntakeControl() 
    : toggleForward(false), toggleReverse(false),
      R1_lastState(false), A_lastState(false) {}

void IntakeControl::update(OuttakeControl& outtake) {
    bool isBlocked = outtake.isMidScoring();
    
    bool R1_current = master.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
    bool A_current = master.get_digital(pros::E_CONTROLLER_DIGITAL_A);

    // Only process intake toggles when NOT in mid-scoring mode
    if (!isBlocked) {
        if (R1_current && !R1_lastState) {
            // If combo mode is active, cancel it first
            if (outtake.isComboMode()) {
                outtake.cancelCombo();
            }
            toggleReverse = !toggleReverse;
            if (toggleReverse) toggleForward = false;
        }
        if (A_current && !A_lastState) {
            // If combo mode is active, cancel it first
            if (outtake.isComboMode()) {
                outtake.cancelCombo();
            }
            toggleForward = !toggleForward;
            if (toggleForward) toggleReverse = false;
        }
    }
    
    R1_lastState = R1_current;
    A_lastState = A_current;

    // Only move intake here if NOT blocked by outtake (mid-scoring or combo mode)
    if (!isBlocked && !outtake.isComboMode()) {
        Intake.move_velocity(getVelocity());
    }
}

int IntakeControl::getVelocity() {
    if (toggleForward) return 600;   // Blue cartridge max RPM
    if (toggleReverse) return -600;
    return 0;
}

void IntakeControl::cancelToggles() {
    toggleForward = false;
    toggleReverse = false;
}