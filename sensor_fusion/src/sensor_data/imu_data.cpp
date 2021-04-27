#include "sensor_fusion/sensor_data/imu_data.hpp"

namespace sensor_fusion{

bool IMUData::SyncData(std::deque<IMUData>& UnsyncedData, std::deque<IMUData>& SyncedData, double sync_time){
    while(UnsyncedData.size() >= 2){
        if(UnsyncedData.front().time > sync_time)
            return false;
        if(UnsyncedData.at(1).time < sync_time){
            UnsyncedData.pop_front();
            continue;
        }

        if(sync_time - UnsyncedData.front().time > 0.2){
            UnsyncedData.pop_front();
            return false;
        }
        if(UnsyncedData.at(1).time - sync_time > 0.2){
            UnsyncedData.pop_front();
            return false;
        }
        break;
    }
    if(UnsyncedData.size() < 2)
        return false;

    IMUData front_data = UnsyncedData.at(0);
    IMUData back_data = UnsyncedData.at(1);
    IMUData synced_data;

    double front_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);
    double back_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);
    synced_data.time = sync_time;
    synced_data.linear_acceleration.x = front_data.linear_acceleration.x * front_scale + back_data.linear_acceleration.x * back_scale;
    synced_data.linear_acceleration.y = front_data.linear_acceleration.y * front_scale + back_data.linear_acceleration.y * back_scale;
    synced_data.linear_acceleration.z = front_data.linear_acceleration.z * front_scale + back_data.linear_acceleration.z * back_scale;
    synced_data.angular_velocity.x = front_data.angular_velocity.x * front_scale + back_data.angular_velocity.x * back_scale;
    synced_data.angular_velocity.y = front_data.angular_velocity.y * front_scale + back_data.angular_velocity.y * back_scale;
    synced_data.angular_velocity.z = front_data.angular_velocity.z * front_scale + back_data.angular_velocity.z * back_scale;
    synced_data.orientation.x = front_data.orientation.x * front_scale + back_data.orientation.x * back_scale;
    synced_data.orientation.y = front_data.orientation.y * front_scale + back_data.orientation.y * back_scale;
    synced_data.orientation.z = front_data.orientation.z * front_scale + back_data.orientation.z * back_scale;
    synced_data.orientation.w = front_data.orientation.w * front_scale + back_data.orientation.w * back_scale;
    synced_data.orientation.Normlize();

    SyncedData.push_back(synced_data);

    return true;
}
}
