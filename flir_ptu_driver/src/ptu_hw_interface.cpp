#include <flir_ptu_driver/ptu_hw_interface.h>

namespace flir_ptu_driver
{

PtuHwInterface::PtuHwInterface() :
  first_cycle_( true ), reset_required_(false), control_mode_( ControlMode::None )
{
  // initialize vector first, so that we can use addresses of members
  goal_pos_.resize( 2, 0 );
  goal_vel_.resize( 2, 0 );
  cur_vel_.resize( 2, 0 );
  cur_pos_.resize( 2, 0 );
  cur_eff_.resize( 2, 0 );
  joint_names_ = std::vector<std::string>{ "sensor_head_yaw_joint", "sensor_head_pitch_joint" };
}

bool PtuHwInterface::init( ros::NodeHandle &nh, ros::NodeHandle &pnh )
{
  nh_ = nh;
  pnh_ = pnh;

  for ( int i = 0; i < joint_names_.size(); i++ )
  {
    hardware_interface::JointStateHandle state_handle( joint_names_[i], &cur_pos_[i], &cur_vel_[i], &cur_eff_[i] );
    jnt_state_interface_.registerHandle( state_handle );

    hardware_interface::JointHandle pos_handle( state_handle, &goal_pos_[i] );
    jnt_pos_interface_.registerHandle( pos_handle );

    hardware_interface::JointHandle vel_handle( state_handle, &goal_vel_[i] );
    jnt_vel_interface_.registerHandle( vel_handle );
  }

  registerInterface( &jnt_state_interface_ );
  registerInterface( &jnt_pos_interface_ );
  registerInterface( &jnt_vel_interface_ );

  // Query for serial configuration
  std::string port;
  int32_t baud;
  ros::param::param<std::string>("~port", port, PTU_DEFAULT_PORT);
  ros::param::param<int32_t>("~baud", baud, PTU_DEFAULT_BAUD);
  ros::param::param<double>("~min_move_vel", min_move_vel_, 0.005);


  // Connect to the PTU
  ROS_INFO_STREAM("Attempting to connect to FLIR PTU on " << port);

  try
  {
    serial_.setPort(port);
    serial_.setBaudrate(baud);
    serial::Timeout to = serial::Timeout(200, 200, 0, 200, 0);
    serial_.setTimeout(to);
    serial_.open();
  }
  catch (serial::IOException& e)
  {
    ROS_ERROR_STREAM("Unable to open port " << port);
    return false;
  }

  ROS_INFO_STREAM("FLIR PTU serial port opened, now initializing.");

  ptu_ = std::make_shared<PTU>(&serial_);

  if (!ptu_->initialize())
  {
    ROS_ERROR_STREAM("Could not initialize FLIR PTU on " << port);
    ptu_.reset();
    return false;
  }

  ROS_INFO("FLIR PTU initialized.");

  first_cycle_ = false;

  return true;
}

PtuHwInterface::~PtuHwInterface()
{
}

void PtuHwInterface::read( const ros::Time &t, const ros::Duration &d )
{
  if (!ptu_)
  {
    reset_required_ = true;
    return;
  }

  cur_pos_[0] = -ptu_->getPosition(PTU_PAN);
  cur_pos_[1] = -ptu_->getPosition(PTU_TILT);
  cur_vel_[0] = -ptu_->getSpeed(PTU_PAN);
  cur_vel_[1] = -ptu_->getSpeed(PTU_TILT);
  cur_eff_[0] = 0;
  cur_eff_[1] = 0;

  ROS_DEBUG_STREAM( "pos 0: " << cur_pos_[0] << " pos 1: " << cur_pos_[1] <<
                             " vel 0: " << cur_vel_[0] << " vel 1: " << cur_vel_[1] <<
                             " eff 0: " << cur_eff_[0] << " eff 1: " << cur_eff_[1] );
}

void PtuHwInterface::write( const ros::Time &t, const ros::Duration &d )
{
  if ( first_cycle_ || control_mode_ == ControlMode::None || control_mode_ == ControlMode::EmergencyStop || !ptu_ )
  {
    return;
  }

  if ( control_mode_ == ControlMode::Position )
  {
    ROS_DEBUG_STREAM( "commands: " << " pos 0: " << goal_pos_[0] << " pos 1: " << goal_pos_[1] );
    ptu_->setPosition(PTU_PAN, -goal_pos_[0]);
    ptu_->setPosition(PTU_TILT, -goal_pos_[1]);
    ptu_->setSpeed(PTU_PAN, 0.25);
    ptu_->setSpeed(PTU_TILT, 0.25);
  }
  else if ( control_mode_ == ControlMode::Velocity )
  {
    ROS_DEBUG_STREAM( "commands: " << " vel 0: " << goal_vel_[0] << " vel 1: " << goal_vel_[1] );

    if (std::abs(goal_vel_[0]) >= min_move_vel_)
      ptu_->setSpeed(PTU_PAN, -goal_vel_[0]);
    else
      ptu_->setSpeed(PTU_PAN, 0);

    if (std::abs(goal_vel_[1]) >= min_move_vel_)
      ptu_->setSpeed(PTU_TILT, -goal_vel_[1]);
    else
      ptu_->setSpeed(PTU_TILT, 0);
  }
}

bool PtuHwInterface::prepareSwitch( const std::list<hardware_interface::ControllerInfo> &start_list,
                                        const std::list<hardware_interface::ControllerInfo> &stop_list )
{
  if ( !RobotHW::prepareSwitch( start_list, stop_list ))
  {
    ROS_ERROR( "RobotHW reported an error during preparation of the controller switch" );
    return false;
  }

  bool use_position_interface = false;
  bool use_velocity_interface = false;

  parseControllerList( start_list, use_position_interface, use_velocity_interface );

  // check how many interfaces (0-n) interfaces were requested
  int num_true_statements = 0;
  num_true_statements += (int) use_position_interface;
  num_true_statements += (int) use_velocity_interface;

  // we allow one interface at most
  return num_true_statements <= 1;
}

void PtuHwInterface::doSwitch( const std::list<hardware_interface::ControllerInfo> &start_list,
                                   const std::list<hardware_interface::ControllerInfo> &stop_list )
{
  RobotHW::doSwitch( start_list, stop_list );
  bool use_position_interface = false;
  bool use_velocity_interface = false;

  parseControllerList( start_list, use_position_interface, use_velocity_interface );

  if ( use_position_interface )
  {
    ROS_INFO_STREAM( "switched to position interface" );
    control_mode_ = ControlMode::Position;
    ptu_->setMode(PTU_POSITION);
  }
  else if ( use_velocity_interface )
  {
    ROS_INFO_STREAM( "switched to velocity interface" );
    control_mode_ = ControlMode::Velocity;
    ptu_->setMode(PTU_VELOCITY);
  }
  else
  {
    ROS_INFO_STREAM( "switched to no control interface" );
    control_mode_ = ControlMode::None;
  }
}

void PtuHwInterface::parseControllerList( const std::list<hardware_interface::ControllerInfo> &start_list,
                                              bool &use_pos_interface, bool &use_vel_interface)
{
  use_pos_interface = false;
  use_vel_interface = false;

  for ( const auto &ctrl_info : start_list )
  {
    const hardware_interface::InterfaceResources &iface_res = ctrl_info.claimed_resources.front();
    for ( const auto &res : iface_res.resources )
    {
      if ( iface_res.hardware_interface == "hardware_interface::PositionJointInterface" )
      {
        use_pos_interface = true;
      }
      else if ( iface_res.hardware_interface == "hardware_interface::VelocityJointInterface" )
      {
        use_vel_interface = true;
      }
    }
  }
}

bool PtuHwInterface::resetRequired() const
{
  return reset_required_;
}

void PtuHwInterface::clearResetRequired()
{
  reset_required_ = false;
}

} // namespace
