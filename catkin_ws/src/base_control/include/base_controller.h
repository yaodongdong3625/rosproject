#ifndef _BASE_CONTROLLER_H
#define _BASE_CONTROLLER_H

#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/tfMessage.h>
#include <tf/tf.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include "odometry.h"

namespace base_controller
{
class BaseController
{
public:
	BaseController();
	~BaseController();

 /**
     * \brief Initialize controller
     * \param hw            Velocity joint interface for the wheels
     * \param root_nh       Node handle at root namespace
     * \param controller_nh Node handle inside the controller namespace
     */
    bool init(ros::NodeHandle& root_nh);

    /**
     * \brief Updates controller, i.e. computes the odometry and sets the new velocity commands
     * \param time   Current time
     * \param period Time since the last called to update
     */
    void update(const ros::Time& time, const ros::Duration& period, double& left_pose, double& right_pose);

 	/**
     * \brief Starts controller
     * \param time Current time
     */
    void starting(const ros::Time& time);

    /**
     * \brief Stops controller
     * \param time Current time
     */
    void stopping(const ros::Time& /*time*/);
    void getOdometryMsg(nav_msgs::Odometry& msg_);

    void computeMotorCmd(const double& v, const double &w, double &left_motor, double &right_motor);

 private:

    /// Odometry related:
    ros::Duration publish_period_;
    ros::Time last_state_publish_time_;
    //realtime_tools::RealtimeBuffer<Commands> command_;
    //Commands command_struct_;
    ros::Subscriber sub_command_;

    /// Publish executed commands
    boost::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::TwistStamped> > cmd_vel_pub_;

    /// Odometry related:
    boost::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry> > odom_pub_;
    boost::shared_ptr<realtime_tools::RealtimePublisher<tf::tfMessage> > tf_odom_pub_;
    Odometry odometry_;

    /// Wheel separation, wrt the midpoint of the wheel width:
    double wheel_separation_;

    /// Wheel radius (assuming it's the same for the left and right wheels):
    double wheel_radius_;

    /// Wheel separation and radius calibration multipliers:
    double wheel_separation_multiplier_;
    double left_wheel_radius_multiplier_;
    double right_wheel_radius_multiplier_;
    //
    double wheel_reduction_ratio; // a value > 1
    /// Timeout to consider cmd_vel commands old:
    double cmd_vel_timeout_;

    /// Whether to allow multiple publishers on cmd_vel topic or not:
    bool allow_multiple_cmd_vel_publishers_;

    /// Frame to use for the robot base:
    std::string base_frame_id_;

    /// Frame to use for odometry and odom tf:
    std::string odom_frame_id_;

    /// Whether to publish odometry to tf or not:
    bool enable_odom_tf_;

    /// Number of wheel joints:
    size_t wheel_joints_size_;

    /// Speed limiters:
    //Commands last1_cmd_;
    //Commands last0_cmd_;
    //SpeedLimiter limiter_lin_;
    //SpeedLimiter limiter_ang_;

    /// Publish limited velocity:
    bool publish_cmd_;

	
};



}//end of namespace
#endif