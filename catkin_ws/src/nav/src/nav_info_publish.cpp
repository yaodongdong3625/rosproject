#include <ros/ros.h>
#include "app/evt.h"
#include "nav_msgs/Path.h"
#include "nav/targetpos.h"
#include "nav/robotStatus.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "aux/direction.h"

#define nav_actived 1
#define nav_sleep 2
#define begin_time 10
namespace nav_info{
ros::Publisher appEvtPublisher;
ros::Timer timer_global;
ros::Timer timer_local;
ros::Timer timer_begin;
ros::Timer timer_report;
app::evt navinfopublishEVT;
int robot_status_flag;
nav::targetpos robot_targetpose;
bool timer_begin_flag = false;
bool timer_report_interal_flag = false;
ros::Publisher debugPublisher;
std_msgs::Int32 debug_data;
nav::targetpos robot_init_pose;
float robot_x;
float robot_y;
float robot_theta;
int num = 0;
enum count_error{
    none_,local_error,global_error,revolve_error
}nav_info_status;
void manualMotionSpeedCallback(const aux::direction & pSpeed)
{
    num = 0;
    timer_local.setPeriod(ros::Duration(5),true);
    timer_global.setPeriod(ros::Duration(10),true);
}
void publish_appEvt_path_error()
{
    if(timer_report_interal_flag == false){
        app::evt navlocalinfopublishEVT;
        navlocalinfopublishEVT.p00 = "nav_abnormal_info";
        navlocalinfopublishEVT.p03 = "nav_info.spinaround";
        appEvtPublisher.publish(navlocalinfopublishEVT);
        debugPublisher.publish(debug_data);
        timer_report_interal_flag = true;
        timer_report.start();
    }
}
void publish_appEvt_target_error(){
    if(timer_report_interal_flag == false){
        app::evt navlocalinfopublishEVT;
        navlocalinfopublishEVT.p00 = "nav_abnormal_info";
        navlocalinfopublishEVT.p04 = "nav_info.target_occupied";
        appEvtPublisher.publish(navlocalinfopublishEVT);
        debugPublisher.publish(debug_data);
        timer_report_interal_flag = true;
        timer_report.start();
    }
}

void record_init_pose()
{
    robot_init_pose.img_u = robot_targetpose.img_u;
    robot_init_pose.img_v = robot_targetpose.img_v;
    robot_init_pose.theta = robot_targetpose.theta;
}
void local_plan_(const nav_msgs::Path &path)
{
    timer_local.setPeriod(ros::Duration(5),true);
    if(nav_info_status == local_error){
        nav_info_status = none_;
    }
}

void global_plan_(const nav_msgs::Path &path)
{
    timer_global.setPeriod(ros::Duration(10),true);
    if(nav_info_status == global_error){
        nav_info_status = none_;
    }
}



void time_global_(const ros::TimerEvent& num)
{
    app::evt navglobalinfopublishEVT;
    navglobalinfopublishEVT.p00 = "nav_abnormal_info";
    navglobalinfopublishEVT.p05 = "nav_info.path_error";
    appEvtPublisher.publish(navglobalinfopublishEVT);

    debug_data.data = 1;
    debugPublisher.publish(debug_data);
    nav_info_status = global_error;
}

void timer_local_(const ros::TimerEvent& num)
{
    float distance_to_target =  pow(robot_x - robot_targetpose.img_u,2)+pow(robot_y - robot_targetpose.img_v,2);
    if((distance_to_target>200)&&(distance_to_target<900)){
        debug_data.data = 2;
        publish_appEvt_target_error();
    }else if(distance_to_target>=900){
        debug_data.data = 3;
        publish_appEvt_path_error();
    }
}

void nav_arrive_target(const nav::targetpos &targetpos)
{
    robot_targetpose.img_u = targetpos.img_u;
    robot_targetpose.img_v = targetpos.img_v;
    robot_targetpose.theta = targetpos.theta;
}

void nav_robotstatus(const nav::robotStatus &robot_status)
{
    if(robot_status.status == nav::robotStatus::ACTIVED){
        robot_status_flag = nav_actived;
        record_init_pose();
        timer_global.start();
        timer_local.start();
        timer_begin.start();
    }else if(robot_status.status == nav::robotStatus::ABORTING){
        robot_status_flag = nav_sleep;
        timer_begin_flag = false;
        timer_global.stop();
        timer_local.stop();
        timer_begin.stop();
    }else if(robot_status.status == nav::robotStatus::SUCCEEDED){
        robot_status_flag = nav_sleep;
        timer_begin_flag = false;
        timer_global.stop();
        timer_local.stop();
        timer_begin.stop();
    }
}

void get_robot_status(const nav::targetpos &pose)
{
    static int num = 0;
    robot_x = pose.img_u;
    robot_y = pose.img_v;
    robot_theta = pose.theta;
    if(robot_status_flag == nav_actived){

    }
}

void timer_begin_(const ros::TimerEvent& num)
{
    timer_begin_flag = true;
    timer_begin.stop();
}

void get_robot_nav_speed(const geometry_msgs::Twist &cmd_vel)
{
    if(num>=1){
        //std::cout<<num<<std::endl;
    }
    if(robot_status_flag == nav_actived){
        if((cmd_vel.linear.x<0.06)&&(cmd_vel.linear.x>-0.06)){
            if((cmd_vel.angular.z>0.09)||(cmd_vel.angular.z<-0.09)){
                if(timer_begin_flag == true){
                    num++;
                }
            }
        }else{
            num = 0;
        }
    }else if(robot_status_flag == nav_sleep){
        num = 0;
    }
    if(num > 35){
        num = 0;
        float distance_ = pow(robot_x - robot_targetpose.img_u , 2)+pow(robot_y - robot_targetpose.img_v , 2);
        float distance_init = pow(robot_x - robot_init_pose.img_u , 2)+pow(robot_y - robot_init_pose.img_v , 2);
        if(distance_init < 200){

        }
        if((distance_>200)&&(distance_<900)){
            debug_data.data = 4;
            publish_appEvt_target_error();
        }
        else if(distance_>=900){
            debug_data.data = 5;
            publish_appEvt_path_error();
        }
    }
}
void nav_info_pause(const std_msgs::Int32 &num)
{
    if(num.data == 1){
        robot_status_flag = nav_sleep;
        timer_begin_flag = false;
        timer_global.stop();
        timer_local.stop();
        timer_begin.stop();
    }else if(num.data == 0){
        robot_status_flag = nav_actived;
        record_init_pose();
        timer_global.start();
        timer_local.start();
        timer_begin.start();
    }
}

void timer_report_(const ros::TimerEvent& num){
    timer_report_interal_flag = false;
    timer_report.stop();
}

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "nav_info_publish");
    ros::NodeHandle Node;
    ros::Subscriber plan_local_sub = Node.subscribe("/move_base/DWAPlannerROS/local_plan",5,nav_info::local_plan_);
    ros::Subscriber plan_global_sub = Node.subscribe("/move_base/GlobalPlanner/plan",5,nav_info::global_plan_);
    ros::Subscriber nav_target = Node.subscribe("/app/nav_target",5,nav_info::nav_arrive_target);
    ros::Subscriber nav_arrive_target = Node.subscribe("/nav/robotStatus",5,nav_info::nav_robotstatus);
    ros::Subscriber robot_status_pose = Node.subscribe("/robot_image_pose",5,nav_info::get_robot_status);
    ros::Subscriber robot_nav_cmdvel = Node.subscribe("/cmd_vel",5,nav_info::get_robot_nav_speed);
    ros::Subscriber nav_pause = Node.subscribe("/nav_info/pause",5,nav_info::nav_info_pause);
    ros::Subscriber subManualMotionSpeedControl = Node.subscribe("/aux/manual_motion_dircontrol", 100, nav_info::manualMotionSpeedCallback);

    nav_info::appEvtPublisher = Node.advertise<app::evt>("appEvt", 20);
    nav_info::debugPublisher = Node.advertise<std_msgs::Int32>("/nav_info/debug",20);

    nav_info::timer_global = Node.createTimer(ros::Duration(10),nav_info::time_global_);
    nav_info::timer_local = Node.createTimer(ros::Duration(5),nav_info::timer_local_);
    nav_info::timer_begin = Node.createTimer(ros::Duration(begin_time),nav_info::timer_begin_);
    nav_info::timer_report = Node.createTimer(ros::Duration(5),nav_info::timer_report_);


    nav_info::timer_global.stop();
    nav_info::timer_local.stop();
    nav_info::timer_begin.stop();
    nav_info::timer_report.stop();


    ros::spin();
    nav_info::timer_global.stop();
    nav_info::timer_local.stop();
    nav_info::timer_begin.stop();
    nav_info::timer_report.stop();
    return 0;
}
