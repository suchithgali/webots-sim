#ifndef MY_PACKAGE__MY_ROBOT_DRIVER_HPP_
#define MY_PACKAGE__MY_ROBOT_DRIVER_HPP_

#include <array>
#include <unordered_map>

#include <webots/types.h>
#include <webots_ros2_driver/PluginInterface.hpp>
#include <webots_ros2_driver/WebotsNode.hpp>

namespace my_robot_driver {

class MyRobotDriver : public webots_ros2_driver::PluginInterface
{
public:
  void init(webots_ros2_driver::WebotsNode * node,
            std::unordered_map<std::string, std::string> & parameters) override;
  void step() override;

private:
  WbDeviceTag gps_;
  WbDeviceTag gyro_;
  WbDeviceTag imu_;
  WbDeviceTag propellers_[4];
  int timestep_ms_;

  double vertical_ref_;         
  double linear_x_integral_;
  double linear_y_integral_;

  double prev_x_;
  double prev_y_;
  bool   prev_valid_;           

  int  current_wp_index_;     
  int  log_tick_;

  void run_inner_loop(double desired_vx, double desired_vy,
                    double vx,         double vy,
                    double roll,       double pitch,
                    double roll_v,     double pitch_v,
                    double z);
};

}  

#endif 