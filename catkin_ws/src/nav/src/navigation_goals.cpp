#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <boost/thread.hpp>
#include "nav/targetpos.h"
#include "nav/robotStatus.h"
#include "std_msgs/Int32.h"
#include "std_srvs/Empty.h"
#include "nav_msgs/GetMap.h"
#include <boost/assign.hpp>
#include <tf/transform_listener.h>
#define MAX_POINTS 1000 

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


ros::Publisher pubRobotStatus;
ros::Publisher appInitPos;
ros::Publisher pubRobotPose;
ros::Publisher nav_info_publisher;

ros::Subscriber navIndexSubscriber;
ros::Subscriber appCmdRobotSubscriber;
ros::Subscriber appInitialSubscriber;
ros::Subscriber robotAmclPoseSubscriber;

nav::robotStatus navstate;
nav::targetpos pos[MAX_POINTS]; 

float image_height = 800;
float image_width  = 800;
float map_resolution =0.05;
float map_origin_x=0.0;
float map_origin_y=0.0;
int target_index=-1;
int INDEX_MAX=0;
float time_out= 30;
int auto_mode=0;
bool nexttarget=false;
int app_cmd=0;
bool pause_robot =false;
geometry_msgs::PoseWithCovarianceStamped initposmsg;
ros::ServiceClient clearlocalcostmapclient;
std_srvs::Empty clearcostmapsrv;
namespace nav_goal{
void transformImage2Map(float u,float v,float *x,float *y)
{
    *x=map_origin_x + map_resolution * u;
    *y=map_origin_y + map_resolution * (image_height - v);
}

void transformMap2Image(float x,float y,float *u,float *v)
{
    *u=(x - map_origin_x)/map_resolution;
    *v=image_height - (y - map_origin_y)/map_resolution;
}
}

void spinThread(){
    ros::spin();
}

// Called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result)
{
    if(!pause_robot){
        ROS_INFO("Finished in state [%s]", state.toString().c_str());
        nexttarget=false;

#if 1
        if(state==actionlib::SimpleClientGoalState::SUCCEEDED)
            navstate.status = nav::robotStatus::SUCCEEDED;
        else if(state==actionlib::SimpleClientGoalState::ABORTED)
            navstate.status = nav::robotStatus::ABORTING;
        else if(state==actionlib::SimpleClientGoalState::PREEMPTED)
            navstate.status = nav::robotStatus::PREEMPTED;
        else if(state==actionlib::SimpleClientGoalState::LOST){
            //navstate.status = nav::robotStatus::ABORTING;
            nexttarget=true;
        }
        else
            navstate.status = nav::robotStatus::SUCCEEDED;
#endif    
        navstate.targetindex=target_index;
        pubRobotStatus.publish(navstate);
        sleep(1);

        if(auto_mode ==1){
            target_index++;
            if(target_index==(INDEX_MAX))
            {
                target_index=0;
            }
            nexttarget=true;
        }
        ROS_INFO("%d %d\n",target_index,nexttarget);
    }else{
        ROS_INFO("Robot paused in state [%s]", state.toString().c_str());
        std_msgs::Int32 flag_pause;
        flag_pause.data = 1;
        nav_info_publisher.publish(flag_pause);
    }
}

// Called once when the goal becomes active
void activeCb()
{
    if(!pause_robot){
        ROS_INFO("Goal just went active");
        navstate.status = nav::robotStatus::ACTIVED;
        navstate.targetindex=target_index;
        pubRobotStatus.publish(navstate);
    }
}

// Called every time feedback is received for the goal
void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
{
    float tempu ,tempv;
    if(!pause_robot){
        navstate.status=nav::robotStatus::DOING;
        navstate.targetindex=target_index;

        nav_goal::transformMap2Image(feedback->base_position.pose.position.x,feedback->base_position.pose.position.y,&tempu ,&tempv);
        navstate.x =tempu;
        navstate.y =tempv;
        navstate.theta =tf::getYaw(feedback->base_position.pose.orientation);
        ROS_INFO("map_pos: (%.3f %.3f %.3f) image_pose:(%.1f %.1f)\n", feedback->base_position.pose.position.x,feedback->base_position.pose.position.y,navstate.theta,tempu,tempv);
        pubRobotStatus.publish(navstate);
    }
}
void navTargetCallback(const nav::targetpos & goal_pos)
{
#if 0    
    // set the linear and angular speed demand
    if(clearlocalcostmapclient.exists())
    {
        if (clearlocalcostmapclient.call(clearcostmapsrv))
        {
            ROS_INFO("Successful call service clear_costmap");
        }
        else
        {
            ROS_ERROR("Failed to call service clear_costmap");
        }
    }
    else
    {
        ROS_ERROR("service clear_costmap don't exists");
    }
#endif
    target_index =0;
    pos[target_index].img_u=goal_pos.img_u;
    pos[target_index].img_v=goal_pos.img_v;
    pos[target_index].theta=goal_pos.theta;
    if(target_index>=INDEX_MAX||target_index<0){
        ROS_ERROR("target index is out of the scope!\n");
    }
    if(!pause_robot) //if the robot pause, don't move the robot
        nexttarget=true;
    ROS_INFO("target to %.1f  %.1f  %.3f\n",goal_pos.img_u,goal_pos.img_v,goal_pos.theta);
    

}
void appCmdRobotCallback(const std_msgs::Int32 & cmd_robot)
{
    // set the linear and angular speed demand
    if(cmd_robot.data==1)
    {
        app_cmd=1;
        navstate.status = nav::robotStatus::ABORTING;
        navstate.targetindex=target_index;
        pubRobotStatus.publish(navstate);
        pause_robot =false;
    }else if(cmd_robot.data ==5) //pause the robot
    {
        app_cmd =1;
        pause_robot =true;

    }else if(cmd_robot.data ==6) //continue to process the previous point
    {
        app_cmd =0;
        pause_robot =false;
        nexttarget=true; //continue to process the previous point
        std_msgs::Int32 flag_pause;
        flag_pause.data = 0;
        nav_info_publisher.publish(flag_pause);
    }


}
//using the inital postion sent by app to initialze the robot position
void appCmdInitialPoseCallback(const nav::targetpos & initialpose)
{
    static uint32_t seq=0;
    double theta =0;
    float tempx =0,tempy=0,x=0,y=0;
    
    initposmsg.header.seq=seq;
    initposmsg.header.stamp =ros::Time::now();
    initposmsg.header.frame_id ="map";
    tempx =initialpose.img_u;
    tempy =initialpose.img_v;
    theta =initialpose.theta;
    nav_goal::transformImage2Map(tempx,tempy,&x,&y);
    initposmsg.pose.pose.position.x =x;
    initposmsg.pose.pose.position.y =y;
    initposmsg.pose.pose.position.z =0;
    initposmsg.pose.pose.orientation=tf::createQuaternionMsgFromYaw(theta);
    initposmsg.pose.covariance = boost::assign::list_of
            (0.25) (0)  (0)  (0)  (0)  (0)
            (0)  (0.25) (0)  (0)  (0)  (0)
            (0)  (0)  (0) (0)  (0)  (0)
            (0)  (0)  (0)  (0) (0)  (0)
            (0)  (0)  (0)  (0)  (0) (0)
            (0)  (0)  (0)  (0)  (0)  (0.06853891945200942);
    appInitPos.publish(initposmsg);
    seq++;
}
//publish the robot position measured by map image
void robotAmclPose(double x,double y,tf::Quaternion q)
{

    float tempu=0,tempv=0,temptheta=0;
    nav::targetpos robot_image_pose;

    nav_goal::transformMap2Image(x,y,&tempu ,&tempv);
    robot_image_pose.img_u=tempu;
    robot_image_pose.img_v=tempv;
    temptheta =tf::getYaw(q);
    robot_image_pose.theta=temptheta;
    //    ROS_INFO("map_pos: (%.3f %.3f %.3f) image_pose:(%.1f %.1f)\n", robotmappose.pose.pose.position.x,robotmappose.pose.pose.position.y,temptheta,tempu,tempv);
    pubRobotPose.publish(robot_image_pose);

}

int main(int argc, char** argv){

    ros::init(argc, argv, "navigation_goals");
    ros::NodeHandle n("~");
    XmlRpc::XmlRpcValue map_pts;
    tf::TransformListener listener;
    tf::StampedTransform transform;
    //call global localization service
    ros::ServiceClient localclient = n.serviceClient<std_srvs::Empty>("/global_localization");


    std_srvs::Empty srv;
    if(localclient.exists())
    {
        if (localclient.call(srv))
        {
            ROS_INFO("Successful call service global_location");
        }
        else
        {
            ROS_ERROR("Failed to call service global_location");
        }
    }
    else
    {
        ROS_ERROR("service global_location don't exists");
    }
    sleep(1);
    //call static map service to acquire the basic information of map
    ros::ServiceClient mapclient = n.serviceClient<nav_msgs::GetMap>("/static_map");
    nav_msgs::GetMap mapinfo;
    ros::ServiceClient gmappingclient = n.serviceClient<nav_msgs::GetMap>("/dynamic_map");
    nav_msgs::GetMap gmmapinginfo;
//    if(mapclient.exists()&&!gmappingclient.exists())
//    {
//        if (mapclient.call(mapinfo))
//        {
//             ROS_INFO("Successful call service static_map");
//            map_resolution = mapinfo.response.map.info.resolution;
//            image_height =mapinfo.response.map.info.height;
//            image_width  = mapinfo.response.map.info.width;
//            map_origin_x =mapinfo.response.map.info.origin.position.x;
//            map_origin_y=mapinfo.response.map.info.origin.position.y;
//        }
//        else
//        {
//            ROS_ERROR("Failed to call service static_map");
//        }
//    }
    sleep(1);
#if 0    
    //call clear_costmap service to acquire the basic information of map
    clearlocalcostmapclient = n.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
    if(clearlocalcostmapclient.exists())
    {
        if (clearlocalcostmapclient.call(clearcostmapsrv))
        {
            ROS_INFO("Successful call service clear_costmap");
        }
        else
        {
            ROS_ERROR("Failed to call service clear_costmap");
        }
    }
    else
    {
        ROS_ERROR("service clear_costmap don't exists");
    }
#endif

    n.getParam("targetpos", map_pts);
    n.getParam("time_out", time_out);
    n.getParam("auto_mode", auto_mode);


    INDEX_MAX=map_pts.size();
    float x=0.0, y=0.0;
    int tempx=0,tempy=0;
    double temptheta=0;
    bool finished_before_timeout=false;
    pubRobotStatus = n.advertise<nav::robotStatus>("/nav/robotStatus", 100);
    navIndexSubscriber =n.subscribe("/app/nav_target", 50, navTargetCallback);
    appCmdRobotSubscriber=n.subscribe("/app/cmd_robot", 5, appCmdRobotCallback);
    appInitialSubscriber=n.subscribe("/app/initial_pose", 5, appCmdInitialPoseCallback);
    appInitPos =n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose"       , 5);

    pubRobotPose =n.advertise<nav::targetpos>("/robot_image_pose"       , 5);
    nav_info_publisher = n.advertise<std_msgs::Int32>("/nav_info/pause", 20);
    double robot_x=0, robot_y=0;
    tf::Quaternion q;
    for(int i =0; i < INDEX_MAX; i++)
    {
        tempx =map_pts[i][0];
        tempy =map_pts[i][1];
        temptheta =map_pts[i][2];
        pos[i].img_u =tempx;
        pos[i].img_v =tempy;
        pos[i].theta =temptheta;
        ROS_INFO("%d,%d,%.3f\n",tempx,tempy,temptheta);
    }

    //   boost::thread spin_thread = boost::thread(boost::bind(&spinThread));
    MoveBaseClient ac("move_base",true);

    //give some time for connections to register
    sleep(3.0);
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    if(auto_mode ==1){
        target_index=0;
        nexttarget=true;
    }
    ros::Rate r(5);
    while(ros::ok())
    {
        if(app_cmd==1)
        {
            ac.cancelGoal();
            ROS_INFO("cancel goal!");
            app_cmd=0;
        }
        if((nexttarget==true)&&(target_index>=0)&&(target_index<INDEX_MAX)){
            nav_goal::transformImage2Map(pos[target_index].img_u,pos[target_index].img_v,&x,&y);
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose.position.x = x;
            goal.target_pose.pose.position.y = y;
            ROS_INFO("target to %.1f  %.1f  %.3f  %.3f\n",pos[target_index].img_u,pos[target_index].img_v,x,y);
            goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(pos[target_index].theta);
            ROS_INFO("Sending move_base goal");
            ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
            nexttarget=false;
            actionlib::SimpleClientGoalState state = ac.getState();
            ROS_INFO("Action STSTUS: %s",state.toString().c_str());
        }
        // get the robotpose through tf transform from map to base_link
        try{
            ros::Time now = ros::Time::now();
            listener.waitForTransform("/map", "/base_link",now, ros::Duration(1.0));
            listener.lookupTransform("/map", "/base_link",now, transform);
        }
        catch (tf::TransformException &ex){
            ROS_ERROR("%s",ex.what());
            //     ros::Duration(1.0).sleep();
        }
        // the map_resolution, height, width,origin should update periodically
        if(finished_before_timeout == false){
            if(mapclient.exists()&&!gmappingclient.exists())
            {
                if (mapclient.call(mapinfo))
                {
     //               ROS_INFO("Successful call service static_map");
                    map_resolution = mapinfo.response.map.info.resolution;
                    image_height =mapinfo.response.map.info.height;
                    image_width  = mapinfo.response.map.info.width;
                    map_origin_x =mapinfo.response.map.info.origin.position.x;
                    map_origin_y=mapinfo.response.map.info.origin.position.y;
                }
                else
                {
                    ROS_ERROR("Failed to call service static_map");
                }
            }
            finished_before_timeout = true;
        }
        // if robot switch to gmapping mode, the map_resolution, height, width,origin should update periodically
        if(gmappingclient.exists()&&!mapclient.exists())
        {
            if(gmappingclient.call(gmmapinginfo))
            {
//                ROS_INFO("Successful call service dynamic_map");
                map_resolution = gmmapinginfo.response.map.info.resolution;
                image_height =gmmapinginfo.response.map.info.height;
                image_width  = gmmapinginfo.response.map.info.width;
                map_origin_x =gmmapinginfo.response.map.info.origin.position.x;
                map_origin_y=gmmapinginfo.response.map.info.origin.position.y;
            }
            else
            {
                ROS_ERROR("Failed to call service dynamic_map");
            }
            finished_before_timeout = false;
        }
        robot_x =transform.getOrigin().x(), robot_y =transform.getOrigin().y();
        q= transform.getRotation();
        robotAmclPose(robot_x,robot_y,q);

        ros::spinOnce();
        r.sleep();
    }


    return 0;
}
