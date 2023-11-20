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

    double imu_temperature = 0.0;

    const double gravity = 9.81; // Acceleration due to gravity
    struct Quaternion 
    {
        double w, x, y, z;
    };

    struct Euler 
    {
        double roll, pitch, yaw;
    };

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

    bool string_to_values(std::string data) 
    {
      std::istringstream iss(data);
      std::string token;

      // Parsing each comma-separated value
      try 
      { 
        std::getline(iss, token, ','); linear_acceleration_x = std::stod(token);
        std::getline(iss, token, ','); linear_acceleration_y = std::stod(token);
        std::getline(iss, token, ','); linear_acceleration_z = std::stod(token);
        std::getline(iss, token, ','); angular_velocity_x = std::stod(token);
        std::getline(iss, token, ','); angular_velocity_y = std::stod(token);
        std::getline(iss, token, ','); angular_velocity_z = std::stod(token);
        std::getline(iss, token, ','); imu_temperature = std::stod(token);
        //
        // Process Quaternion values via some conversions
        accelerationToAttitude(linear_acceleration_x, linear_acceleration_y, linear_acceleration_z);
      }
      catch (const std::exception& e) 
      {
        std::cerr << "An error occurred: " << e.what() << std::endl;
        return 0;
      }
      return 1;
    }

    double degrees_to_radians(double degrees) 
    {
      return degrees * (M_PI / 180.0);
    }

    Quaternion euler_to_quaternion(double roll, double pitch, double yaw) 
    {
      Quaternion q;
      orientation_w = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
      orientation_x = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
      orientation_y = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2);
      orientation_z = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2);

      return q;
    }

    Euler quaternion_to_euler(double q_w, double q_x, double q_y, double q_z) 
    {
      Euler e;

      e.roll = atan2(2 * (q_w * q_x + q_y * q_z), 1 - 2 * (q_x * q_x + q_y * q_y));
      e.pitch = asin(2 * (q_w * q_y - q_z * q_x));
      e.yaw = atan2(2 * (q_w * q_z + q_x * q_y), 1 - 2 * (q_y * q_y + q_z * q_z));

      // Converting radians to degrees
      e.roll = e.roll * 180.0 / M_PI;
      e.pitch = e.pitch * 180.0 / M_PI;
      e.yaw = e.yaw * 180.0 / M_PI;

      return e;
    }

    Euler accelerationToAttitude(double accelerometer_x, double accelerometer_y, double accelerometer_z) 
    {
      Euler e;

      e.roll = asin(accelerometer_x / gravity);
      e.pitch = -asin(accelerometer_y / (gravity * cos(e.roll)));
      e.yaw = 0; // Yaw is zero as it cannot be determined from accelerometer data

      euler_to_quaternion(e.roll, e.pitch, e.yaw);

      return e;
    }
};


#endif // DIFFDRIVE_DDSM115_IMU_HPP
