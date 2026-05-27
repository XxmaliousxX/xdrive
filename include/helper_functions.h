#pragma once

// ============================================================================
// distance_reset.h
// Header for distance sensor position and heading reset functions (LemLib)
//
// SETUP:
//   1. Declare your sensors and offsets globally in your robot config:
//
//      pros::Distance back_sensor_left(PORT);
//      pros::Distance back_sensor_right(PORT);
//      pros::Distance left_sensor(PORT);
//      pros::Distance right_sensor(PORT);
//
//      const double BACK_SENSOR_LEFT_OFFSET  = ?; // tracking center to left back sensor face (inches)
//      const double BACK_SENSOR_RIGHT_OFFSET = ?; // tracking center to right back sensor face (inches)
//      const double BACK_SENSOR_SPACING      = ?; // horizontal distance between two back sensor faces (inches)
//      const double LEFT_SENSOR_OFFSET       = ?; // tracking center to left sensor face (inches)
//      const double RIGHT_SENSOR_OFFSET      = ?; // tracking center to right sensor face (inches)
//
//   2. Add to your autonomous.cpp:
//      #include "distance_reset.h"
//
//   3. Example usage:
//      driveUntilDistance(back_sensor_left, 3.0, 50, false, 3000);
//      resetPositionAndHeadingBack(back_sensor_left, back_sensor_right,
//                                  BACK_SENSOR_SPACING,
//                                  BACK_SENSOR_LEFT_OFFSET, BACK_SENSOR_RIGHT_OFFSET);
//      resetPositionLeft(left_sensor, LEFT_SENSOR_OFFSET);
//      resetPositionRight(right_sensor, RIGHT_SENSOR_OFFSET);
// ============================================================================

#include "xdrive_config.h"
#include "pros/distance.hpp"
#include "pros/rtos.hpp" // IWYU pragma: keep
#include <cmath>

// ============================================================================
// resetPositionAndHeadingBack
// Resets BOTH Y (or X) position AND heading using two back-facing sensors.
// Uses atan2 to calculate heading offset from difference in readings.
// Uses cosine correction to get true perpendicular distance to wall.
//
// Parameters:
//   back_left      - back-left distance sensor
//   back_right     - back-right distance sensor
//   sensor_spacing - horizontal distance between the two sensor faces (inches)
//   left_offset    - tracking center to left sensor face (inches)
//   right_offset   - tracking center to right sensor face (inches)
//   field_half     - half field size in inches (72.0 default)
// ============================================================================
void resetPositionAndHeadingBack(pros::Distance& back_left, pros::Distance& back_right,
                                  double sensor_spacing = 10.5,
                                  double left_offset = 4.875,    double right_offset = 4.875,
                                  double field_half = 72.0);

// ============================================================================
// resetPositionLeft
// Resets X or Y position using a single left-facing distance sensor.
// Applies cosine trig correction for small angle errors.
//
// Parameters:
//   sensor         - left-facing distance sensor
//   sensor_offset  - tracking center to sensor face (inches)
//   field_half     - half field size in inches (72.0 default)
// ============================================================================
void resetPositionLeft(pros::Distance& sensor, double sensor_offset = 5.8125,
                       double field_half = 72.0);

// ============================================================================
// resetPositionRight
// Resets X or Y position using a single right-facing distance sensor.
// Applies cosine trig correction for small angle errors.
//
// Parameters:
//   sensor         - right-facing distance sensor
//   sensor_offset  - tracking center to sensor face (inches)
//   field_half     - half field size in inches (72.0 default)
// ============================================================================
void resetPositionRight(pros::Distance& sensor, double sensor_offset = 5.8125,
                        double field_half = 72.0);

// ============================================================================
// driveUntilDistance
// Drives the robot until a distance sensor reads below a threshold, then stops.
//
// Parameters:
//   sensor        - distance sensor facing the wall you're driving toward
//   threshold_in  - stop when sensor reads at or below this value (inches)
//   speed         - motor speed 0-127 (default 60)
//   forwards      - true = drive forward, false = drive backward
//   timeout_ms    - emergency stop time in milliseconds (default 3000)
// ============================================================================
void driveUntilDistance(pros::Distance& sensor, double threshold_in,
                        int speed = 60, bool forwards = true, int timeout_ms = 3000);

void park();
void score();
void shakeBot(int durationMs);
void score_ms(int ms);