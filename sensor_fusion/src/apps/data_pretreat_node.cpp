#include <ros/ros.h>

#include "sensor_fusion/data_pretreat/data_pretreat_flow.hpp"

using namespace sensor_fusion;

int main(int argc, char **argv) {
    ros::init(argc, argv, "data_pretreat_node");
    ros::NodeHandle nh;

    std::shared_ptr<DataPretreatFlow> data_pretreat_flow_ptr = std::make_shared<DataPretreatFlow>(nh);

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        data_pretreat_flow_ptr->Run();

        rate.sleep();
    }

    return 0;
}