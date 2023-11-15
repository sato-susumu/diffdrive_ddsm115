#ifndef DIFFDRIVE_ARDUINO_WHEEL_HPP
#define DIFFDRIVE_ARDUINO_WHEEL_HPP

#include <string>
#include <cmath>


class Wheel
{
    public:

    std::string name = "";
    int id = 0;
    double cmd = 0;
    double pos = 0;
    double pos_rads = 0;
    double vel = 0;
    double accumulated_pos = 0;

    Wheel() = default;

    Wheel(const std::string &wheel_name, int motor_id)
    {
      setup(wheel_name, motor_id);
    }

    
    void setup(const std::string &wheel_name, int motor_id)
    {
      name = wheel_name;
      id = motor_id;
    }

    double degrees_to_radians(double degrees) 
    {
      return degrees * (M_PI / 180.0);
    }

    double rpm_to_rad_per_sec(double rpm) 
    {
        return rpm * 0.10472;
    }

    double calculate_accumulated_position(double current_pos) 
    {
      int change = pos - current_pos;

      // Detect direction
      if (change > 180) {
        change -= 360;
      } else if (change < -180) {
        change += 360;
      }

      // Add the change to the accumulated position
      accumulated_pos += change;

      // Update pos to current pos
      pos = current_pos;

      return accumulated_pos;
    }

};


#endif // DIFFDRIVE_DDSM115_WHEEL_HPP
