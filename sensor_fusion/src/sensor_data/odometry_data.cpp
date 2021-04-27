#include "sensor_fusion/sensor_data/odometry_data.hpp"

namespace sensor_fusion{

bool OdometryData::SyncData(std::deque<OdometryData>& UnsyncedData, std::deque<OdometryData>& SyncedData, double sync_time){
    while (UnsyncedData.size() >= 2) {
        // UnsyncedData.front().time should be <= sync_time:
        if (UnsyncedData.front().time > sync_time) 
            return false;
        // sync_time should be <= UnsyncedData.at(1).time:
        if (UnsyncedData.at(1).time < sync_time) {
            UnsyncedData.pop_front();
            continue;
        }

        // sync_time - UnsyncedData.front().time should be <= 0.2:
        if (sync_time - UnsyncedData.front().time > 0.2) {
            UnsyncedData.pop_front();
            return false;
        }
        // UnsyncedData.at(1).time - sync_time should be <= 0.2
        if (UnsyncedData.at(1).time - sync_time > 0.2) {
            UnsyncedData.pop_front();
            return false;
        }
        break;
    }
    if (UnsyncedData.size() < 2)
        return false;

    OdometryData front_data = UnsyncedData.at(0);
    OdometryData back_data = UnsyncedData.at(1);
    OdometryData synced_data;

    double front_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);
    double back_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);
    synced_data.time = sync_time;

    synced_data.position.x = front_data.position.x * front_scale + back_data.position.x * back_scale;
    synced_data.position.y = front_data.position.y * front_scale + back_data.position.y * back_scale;
    synced_data.position.z = front_data.position.z * front_scale + back_data.position.z * back_scale;

    synced_data.orientation.x = front_data.orientation.x * front_scale + back_data.orientation.x * back_scale;
    synced_data.orientation.y = front_data.orientation.y * front_scale + back_data.orientation.y * back_scale;
    synced_data.orientation.z = front_data.orientation.z * front_scale + back_data.orientation.z * back_scale;
    synced_data.orientation.w = front_data.orientation.w * front_scale + back_data.orientation.w * back_scale;

    synced_data.orientation.Normlize();

    synced_data.linear.x = front_data.linear.x * front_scale + back_data.linear.x * back_scale;
    synced_data.linear.y = front_data.linear.y * front_scale + back_data.linear.y * back_scale;
    synced_data.linear.z = front_data.linear.z * front_scale + back_data.linear.z * back_scale;
    synced_data.angular.x = front_data.angular.x * front_scale + back_data.angular.x * back_scale;
    synced_data.angular.y = front_data.angular.y * front_scale + back_data.angular.y * back_scale;
    synced_data.angular.z = front_data.angular.z * front_scale + back_data.angular.z * back_scale;
   
    SyncedData.push_back(synced_data);

    return true;
}
}
