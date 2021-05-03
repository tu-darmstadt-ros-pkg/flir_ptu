#pragma once

#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <ros/callback_queue.h>

#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>

#include <flir_ptu_driver/driver.h>
#include <serial/serial.h>

namespace flir_ptu_driver
{

enum class ControlMode
{
  None, Position, Velocity, EmergencyStop
};

class PtuHwInterface : public hardware_interface::RobotHW
{
public:
  PtuHwInterface();

  ~PtuHwInterface();

  void read( const ros::Time &t, const ros::Duration &d ) override;

  void write( const ros::Time &t, const ros::Duration &d ) override;

  bool init( ros::NodeHandle &nh, ros::NodeHandle &pnh ) override;

  bool prepareSwitch( const std::list<hardware_interface::ControllerInfo> &start_list,
                      const std::list<hardware_interface::ControllerInfo> &stop_list ) override;

  void doSwitch( const std::list<hardware_interface::ControllerInfo> &start_list,
                 const std::list<hardware_interface::ControllerInfo> &stop_list ) override;

  bool resetRequired() const;
  void clearResetRequired();

private:

  hardware_interface::JointStateInterface jnt_state_interface_;
  hardware_interface::PositionJointInterface jnt_pos_interface_;
  hardware_interface::VelocityJointInterface jnt_vel_interface_;

  void parseControllerList( const std::list<hardware_interface::ControllerInfo> &start_list,
                            bool &use_pos_interface, bool &use_vel_interface );

  std::vector<double> pos_offset_;

  std::vector<double> goal_pos_;
  std::vector<double> goal_vel_;

  std::vector<double> cur_pos_;
  std::vector<double> cur_vel_;
  std::vector<double> cur_eff_;

  std::vector<std::string> joint_names_;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  bool first_cycle_;
  bool reset_required_;

  ControlMode control_mode_;

  serial::Serial serial_;
  std::shared_ptr<PTU> ptu_;

};
} // namespace
