#ifndef DIFFDRIVE_ARDUINO_IMU_HPP
#define DIFFDRIVE_ARDUINO_IMU_HPP

#include <string>
#include <cmath>


class IMU
{
    public:

    std::string name = "";
    int id = 0;
    double orientation_x = 0.0;
    double orientation_y = 0.0;
    double orientation_z = 0.0;
    double orientation_w = 0.0;

    double angular_velocity_x = 0.0;
    double angular_velocity_y = 0.0;
    double angular_velocity_z = 0.0;

    double linear_acceleration_x = 0.0;
    double linear_acceleration_y = 0.0;
    double linear_acceleration_z = 0.0;
    
    double cmd = 0;
    double pos = 0;
    double pos_rads = 0;
    double vel = 0;
    double accumulated_pos = 0;

    IMU() = default;

    IMU(const std::string &imu_name, int imu_id)
    {
      setup(imu_name, imu_id);
    }

    
    void setup(const std::string &imu_name, int imu_id)
    {
      name = imu_name;
      id = imu_id;
    }

    double degrees_to_radians(double degrees) 
    {
      return degrees * (M_PI / 180.0);
    }

    double rpm_to_rad_per_sec(double rpm) 
    {
        return rpm * 0.10472;
    }

};


#endif // DIFFDRIVE_DDSM115_IMU_HPP
