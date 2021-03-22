#ifndef SENSOR_FUSION_SENSOR_DATA_POSE_DATA_HPP_
#define SENSOR_FUSION_SENSOR_DATA_POSE_DATA_HPP_

#include <Eigen/Dense>
#include <cmath>
#include <deque>

namespace sensor_fusion{
class PoseData{
    public:
        struct Position{
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
        Position position;
        Orientation orientation;

    public:
        static bool SyncData(std::deque<PoseData>& UnsyncedData, std::deque<PoseData>& SyncedData, double sync_time);
};
}

#endif