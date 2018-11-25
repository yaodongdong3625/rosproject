#include <ros/ros.h>
#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"
#include <fstream>
#include <iomanip>
#include "BmpWriter.h"
#include "nav/bitmap.h"
#include "std_srvs/Trigger.h"
namespace {

uint64_t publish_map_req, publish_map_error;
bool status_report(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &res)
{
    res.success = true;
    boost::format fmt("PUBLISH MAP INFO:\n  total recived request: %1%\n  total error: %2%\n");
    fmt % publish_map_req % publish_map_error;
    res.message = fmt.str();
    return true;
}

}


class MapSave
{
public:
    MapSave(const std::string& mapname) :  saved_map_(false)
    {  
        ros::NodeHandle nh;
        publish_map_req++;
        //map_sub_ = nh.subscribe("map", 1, &MapSave::mapCallback, this);
        ros::ServiceClient static_mapclient = nh.serviceClient<nav_msgs::GetMap>("/static_map");
        ros::ServiceClient gmapping_mapclient = nh.serviceClient<nav_msgs::GetMap>("/dynamic_map");
        nav_msgs::GetMap mapinfo;
        if(static_mapclient.exists())
        {
            if (static_mapclient.call(mapinfo))
            {
                ROS_INFO("Successful call service static_map");
                mapCallback(mapinfo,mapname);
                type_map = "static_map";
            }
            else
            {
                ROS_ERROR("Failed to call service static_map");
                publish_map_error++;
            }
        }
        else if(gmapping_mapclient.exists())
        {
            if (gmapping_mapclient.call(mapinfo))
            {
                ROS_INFO("Successful call service dynamic_map");
                mapCallback(mapinfo,mapname);
                type_map = "dynamic_map";
            }
            else
            {
                ROS_ERROR("Failed to call service dynamic_map");
            }
        }

    }
    void mapCallback(const nav_msgs::GetMap& map,const std::string& mapname)
    {
        //FILE* out = fopen(mapdatafile.c_str(), "w");
        TBmpWriter8BppGrey bmp(map.response.map.info.width,map.response.map.info.height);
        for (size_t h = 0; h < bmp.Height; h++) {
            uint8_t *row = bmp.rowPtr(h);
            for (size_t w = 0; w < bmp.Width; w++) {
                unsigned int i = w + (map.response.map.info.height - h - 1) * map.response.map.info.width;
                if (map.response.map.data[i] == 0) { //occ [0,0.1)
                  row[w] = 254;
                } else if (map.response.map.data[i] == +100) { //occ (0.65,1]
                  row[w] = 0;
                } else { //occ [0.1,0.65]
                  row[w] = 205;
                }
            }
        }
        {
            const void *head;
            const void *body;
            size_t head_size, body_size;

            bmp.GetDataPtr(head, head_size, body, body_size);

            FILE *f = fopen(mapname.c_str(), "wb");
            if(f==NULL)
            {
                saved_map_ = false;
            }
            else{
                fwrite(head, 1, head_size, f);
                fwrite(body, 1, body_size, f);
                fclose(f);
                saved_map_ = true;
            }
        }
    }
    std::string mapname_;
    ros::Subscriber map_sub_;
    bool saved_map_;
    std::string type_map;
};

bool get_map(nav::bitmap::Request &req,nav::bitmap::Response &res)
{
    std::string req_ = req.map_name;
    MapSave mg(req_);
    if(mg.saved_map_ == true)
    {
        res.type = mg.type_map;
        res.res = true;
     }
     else
     {
        res.res = false;
     }
}

int main(int argc, char** argv)
{

    int image_height,image_width;
    ros::init(argc, argv, "publish_map");
    ros::NodeHandle n;
    long int *a;
    ros::ServiceServer service = n.advertiseService("get_map_service",get_map);
    ros::ServiceServer status_report_service = n.advertiseService(ros::this_node::getName()  + "/get_status", status_report);
    ros::spin();
    status_report_service.shutdown();
    return 0;
}
