#include <flir_ptu_driver/ptu_hw_interface.h>
#include <controller_manager/controller_manager.h>

int main( int argc, char **argv )
{
  ros::init( argc, argv, "flir_ptu_hw_interface" );
  ros::NodeHandle nh;
  ros::NodeHandle pnh( "~" );

  ros::CallbackQueue queue;
  nh.setCallbackQueue( &queue );
  ros::AsyncSpinner spinner( 1, &queue );
  spinner.start();

  flir_ptu_driver::PtuHwInterface ptu_hw_interface;

  if ( !ptu_hw_interface.init( nh, pnh ))
  {
    ROS_ERROR_STREAM( "Failed to initialize hardware interface." );
    return 0;
  }

  controller_manager::ControllerManager cm( &ptu_hw_interface, nh );

  ros::Time time_now = ros::Time::now();
  ros::Duration elapsed_time;

  ros::Rate control_rate( 50 );

  while ( ros::ok())
  {
    elapsed_time = ros::Time::now() - time_now;
    time_now = ros::Time::now();

    ptu_hw_interface.read( time_now, elapsed_time );

    cm.update( time_now, elapsed_time, ptu_hw_interface.resetRequired() );
    ptu_hw_interface.clearResetRequired();

    ptu_hw_interface.write( time_now, elapsed_time );

    control_rate.sleep();
    ros::spinOnce();
  }

  spinner.stop();

  return 0;
}
