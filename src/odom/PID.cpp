#include "PID.hpp"
#include <cmath>

namespace xdrive {

PID::PID(PIDGains gains) : m_gains(gains) {}

void     PID::setGains(PIDGains gains) { m_gains = gains; }
PIDGains PID::getGains() const         { return m_gains;  }

float PID::update(float error, float dt) {
    // Derivative — skip first call (no previous error to diff against)
    float derivative = 0;
    if (!m_firstCall && dt > 0) derivative = (error - m_prevError) / dt;
    m_firstCall = false;

    // Integral with anti-windup range
    bool inWindupRange = (m_gains.windupRange <= 0) || (std::fabs(error) < m_gains.windupRange);
    if (inWindupRange) m_integral += error * dt;

    // Sign-flip reset — clear integral if error crossed zero
    if (m_prevError != 0 && (error > 0) != (m_prevError > 0)) m_integral = 0;

    m_prevError = error;
    return m_gains.kP * error + m_gains.kI * m_integral + m_gains.kD * derivative;
}

void PID::reset() {
    m_integral  = 0;
    m_prevError = 0;
    m_firstCall = true;
}

} // namespace xdrive
