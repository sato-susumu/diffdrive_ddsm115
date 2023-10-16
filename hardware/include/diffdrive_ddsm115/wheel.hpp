#ifndef DIFFDRIVE_ARDUINO_WHEEL_HPP
#define DIFFDRIVE_ARDUINO_WHEEL_HPP

#include <string>
#include <cmath>


class Wheel
{
    public:

    std::string name = "";
    int id = 0;
    int enc = 0;
    double cmd = 0;
    double pos = 0;
    double vel = 0;
    double rads_per_count = 0;

    Wheel() = default;

    Wheel(const std::string &wheel_name, int counts_per_rev, int motor_id)
    {
      setup(wheel_name, counts_per_rev, motor_id);
    }

    
    void setup(const std::string &wheel_name, int counts_per_rev, int motor_id)
    {
      name = wheel_name;
      rads_per_count = (2*M_PI)/counts_per_rev;
      id = motor_id;
    }

    double calc_enc_angle()
    {
      return enc * rads_per_count;
    }

    double degrees_to_radians(double degrees) 
    {
      return degrees * (M_PI / 180.0);
    }

    // Function to convert RPM to rad/s
    double rpm_to_rad_per_sec(double rpm) {
        // Conversion factor: 1 RPM = 2 * π rad/s
        //return rpm * 2.0 * M_PI / 60.0;
        return rpm * 0.10472;
    }

};


#endif // DIFFDRIVE_ARDUINO_WHEEL_HPP
