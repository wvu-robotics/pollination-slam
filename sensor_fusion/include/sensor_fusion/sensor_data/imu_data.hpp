#ifndef SENSOR_FUSION_SENSOR_DATA_IMU_DATA_HPP_
#define SENSOR_FUSION_SENSOR_DATA_IMU_DATA_HPP_

#include <Eigen/Dense>
#include <cmath>
#include <deque>

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

        class Orientation{
            public:
                double x = 0.0;
                double y = 0.0;
                double z = 0.0;
                double w = 0.0;
            public:
                void Normlize(){
                    double norm = sqrt(pow(x, 2.0) + pow(y, 2.0) + pow(z, 2.0) + pow(w, 2.0));
                    x /= norm;
                    y /= norm;
                    z /= norm;
                    w /= norm;
                }
        };

        double time = 0.0;
        LinearAcceleration linear_acceleration;
        AngularVeclocity angular_velocity;
        Orientation orientation;

    public:
        static bool SyncData(std::deque<IMUData>& unsyncedData, std::deque<IMUData>& SyncedData, double sync_time);  
    };
}

#endif