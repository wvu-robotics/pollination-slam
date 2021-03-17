#ifndef SENSOR_FUSION_SENSOR_DATA_POSE_DATA_HPP_
#define SENSOR_FUSION_SENSOR_DATA_POSE_DATA_HPP_

#include <Eigen/Dense>

namespace sensor_fusion{
class PoseData{
    public:
    struct Position{
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    struct Orientation{
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double w = 0.0;
    };
    
    double time = 0.0;
    Position position;
    Orientation orientation;
        
};
}

#endif