#ifndef SENSOR_FUSION_SENSOR_DATA_IMU_DATA_HPP_
#define SENSOR_FUSION_SENSOR_DATA_IMU_DATA_HPP_

#include <Eigen/Dense>

namespace sensor_fusion{
class IMUData{
    public:
        struct LinearAcceleration{
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
        };

        struct AngularVeclocity{
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
        LinearAcceleration linear_acceleration;
        AngularVeclocity angular_velocity;
        Orientation orientation;  
    };
}

#endif