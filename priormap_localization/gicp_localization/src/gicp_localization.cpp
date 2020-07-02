#include <ros/ros.h>
#include <gicp_localization/GICPLocalization.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "gicp_localization_node");
	ros::NodeHandle n("~");

	tf::TransformBroadcaster transformed_states_tf_broad;
	ros::Rate r(50);	//10 Hz.
	GICPLocalization gicplocalization;
	if(!gicplocalization.Initialize(n)){
		ROS_ERROR("%s: Failed to initialize gicp localization.",
			ros::this_node::getName().c_str());
		return EXIT_FAILURE;
	}

	while(ros::ok()){
		transformed_states_tf_broad.sendTransform(gicplocalization.GetTf());
        // std::cout<<"published tf!"<<std::endl;
		r.sleep();
        ros::spinOnce();
	}


	return EXIT_SUCCESS;
}
