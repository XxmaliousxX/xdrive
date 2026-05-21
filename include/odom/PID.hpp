#pragma once

namespace xdrive {

struct PIDGains {
    float kP          = 0;
    float kI          = 0;
    float kD          = 0;
    float windupRange = 0; // integral zeroed when |error| > this. 0 = disabled.
};

class PID {
public:
    explicit PID(PIDGains gains);

    void     setGains(PIDGains gains);
    PIDGains getGains() const;

    // Call once per loop. dt in seconds.
    float update(float error, float dt);

    // Reset integral and previous error (call between motions if reusing)
    void reset();

private:
    PIDGains m_gains;
    float    m_integral  = 0;
    float    m_prevError = 0;
    bool     m_firstCall = true;
};

} // namespace xdrive
