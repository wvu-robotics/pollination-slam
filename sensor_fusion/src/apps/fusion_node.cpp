#include <ros/ros.h>
#include "sensor_fusion/fusion/fusion_flow.hpp"

using namespace sensor_fusion;

int main(int argc, char *argv[]){

    ros::init(argc, argv, "sensor_fusion_node");
    ros::NodeHandle nh;

    std::shared_ptr<FusionFlow> fusion_flow_ptr = std::make_shared<FusionFlow>(nh);

    ros::Rate rate(100);
    while(ros::ok()){
        ros::spinOnce();
        fusion_flow_ptr->run();

        rate.sleep();
    }

    return 0;
}