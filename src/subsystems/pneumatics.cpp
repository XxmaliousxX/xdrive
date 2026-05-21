#include "subsystems/pneumatics.h"
#include "robot_config.h"

PneumaticControl::PneumaticControl()
    : descoreState(false), unloaderState(false),midDescoreState(false),
    Y_lastState(false), R2_lastState(false), L2_lastState(false) {}

void PneumaticControl::update() {
    // Descore (Button R2) - hold to retract, release to extend
    bool R2_current = master.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
    Descore.set_value(!R2_current);
    descoreState = R2_current;

    // Unloader (Button L2)
    bool L2_current = master.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
    if (L2_current && !L2_lastState) {
        unloaderState = !unloaderState;
        Unloader.set_value(unloaderState);
    }
    L2_lastState = L2_current;

    // Mid Descore (Button Y) - toggle
    bool Y_current = master.get_digital(pros::E_CONTROLLER_DIGITAL_Y);
    if (Y_current && !Y_lastState) {
        midDescoreState = !midDescoreState;
        MidDescore.set_value(midDescoreState);
    }
    Y_lastState = Y_current;
}

bool PneumaticControl::getDescoreState() {
    return descoreState;
}

bool PneumaticControl::getUnloaderState() {
    return unloaderState;
}
