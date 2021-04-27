#ifndef SENSOR_FUSION_SENSOR_DATA_ODOMETRY_DATA_HPP_
#define SENSOR_FUSION_SENSOR_DATA_ODOMETRY_DATA_HPP_

#include <Eigen/Dense>
#include <cmath>
#include <deque>

namespace sensor_fusion{
class OdometryData{
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

	struct Linear{
		double x = 0.0;
		double y = 0.0;
		double z = 0.0;
	};

	struct Angular{
		double x = 0.0;
		double y = 0.0;
		double z = 0.0;
	};
        
        double time = 0.0;
        Position position;
        Orientation orientation;
	Linear linear;
	Angular angular;

    public:
        static bool SyncData(std::deque<OdometryData>& UnsyncedData, std::deque<OdometryData>& SyncedData, double sync_time);
};
}

#endif
