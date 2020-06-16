#include "lidar/hough_transform.hpp"

int main(int argc, char **argv)
{

	ros::init(argc, argv, "hough_transform_node");

	ros::NodeHandle nh;
	ros::Rate loop_rate(20);

	Hough hough;

	while(ros::ok())
	{
		// hough.GetPointcloud();
		// hough.GeneratePointcloud();
		if(hough.newLocalmapAvailable())
		{

			hough.setPreviousCounters();

			hough.clearVariables();

			hough.GetBoundary();

			hough.Transform(hough.cloud);

			hough.DetectUShape();

			while(1)
			{
				if(hough.u_shape.size() < 1)
				{
					std::cout<<"can not find u shape, decrease threshold"<<std::endl;
					hough.threshold = hough.threshold - 5;

					hough._accu = 0;
					hough._accu_w = 0; hough._accu_h = 0; hough._img_w = 0; hough._img_h = 0;

					hough.lines.clear();
					hough.line_info.clear();
					hough.u_shape.clear();

					hough.Transform(hough.cloud);
					hough.DetectUShape();
				}

				if(hough.threshold < 25)
				{
					std::cout<<"can not find u shape with low threshold"<<std::endl;
					break;
				}

				if(hough.u_shape.size() == 1)
				{
					// std::cout<<"find u shape"<<std::endl;

					// for(int i=0; i<hough.u_shape.size(); i++)
					// std::cout<<"u_shape: "<<hough.u_shape.size()<<std::endl;
					// std::cout<<"threshold: "<<hough.threshold<<std::endl;

					// hough.DrawLines();

					hough.GetPose();
					// std::cout<<"x: "<<hough.global_x<<std::endl;
					// std::cout<<"y: "<<hough.global_y<<std::endl;
					// std::cout<<"theta: "<<hough.global_theta<<std::endl;

					// // save to data file
					// std::ofstream datafile("data.txt", ios::app);
					
					
					// 	datafile << hough.global_x << " " << hough.global_y << " " << 0 << " "<< 1<<"\n";
					// std::cout << hough.global_x << " " << hough.global_y<<std::endl;
					hough.DrawUShape();
					break;
				}
				// else
				// 	break;
			}
		

		}
		



		loop_rate.sleep();
		ros::spinOnce();
	}
	


	return 0;
}