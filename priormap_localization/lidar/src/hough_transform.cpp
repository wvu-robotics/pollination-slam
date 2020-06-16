#include "lidar/hough_transform.hpp"

Hough::Hough()
{
	// initialization
	_accu = 0;
	_accu_w = 0; _accu_h = 0; _img_w = 0; _img_h = 0;

	global_x = 0; global_y = 0; global_theta = 0;

	_registration_counter = 0;
	_registration_counter_prev = 0;

	_registration_new = false;

	threshold = 45;

	cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
	lines.clear();
	line_info.clear();
	u_shape.clear();

	_sub_localmap = _nh.subscribe("/lidar/lidarfilteringnode/localmap", 1, &Hough::GetPointcloud, this);
}
// 
Hough::~Hough()
{
	if(_accu)
		free(_accu);
}

// read pointcloud from pcd file
void Hough::GetPointcloud(pcl::PointCloud<pcl::PointXYZ> const &raw_cloud)
{
	std::cout<<_registration_counter<<std::endl;
	_registration_counter = _registration_counter + 1;
	// std::string filepath;
	// filepath = "/home/chizhao/pointcloud1.pcd";	//path to pcd file
	// // filepath = "/home/chizhao/modified_pcd.pcd";
	pcl::PointCloud<pcl::PointXYZ> _raw_cloud_temp;
	pcl::PointCloud<pcl::PointXYZ>::Ptr _raw_cloud ( new pcl::PointCloud<pcl::PointXYZ>);
	// if(pcl::io::loadPCDFile<pcl::PointXYZ>(filepath, *_raw_cloud) == -1)	// load pcd file
	// {
	// 	std::cout<<"Can't load pcd file!"<<std::endl;
	// }
	_raw_cloud_temp = raw_cloud;
	*_raw_cloud = _raw_cloud_temp;
	// Create the filtering for downsample
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud (_raw_cloud);
	sor.setLeafSize(0.05f, 0.05f, 0.01f);
	sor.filter(*cloud);

	// Enlarge pointcloud with 100 times, from meter to cm resolution
	for(int i=0; i<cloud->points.size(); i++)
	{
		cloud->points[i].x = cloud->points[i].x * 100;
		cloud->points[i].y = cloud->points[i].y * 100;
		cloud->points[i].z = cloud->points[i].z * 100;
	}
}

bool Hough::newLocalmapAvailable()
{
	if(_registration_counter != _registration_counter_prev)
	{
		_registration_new = true;
		return true;
	}
	else
	{
		_registration_new = false;
		return false;
	}
}

void Hough::setPreviousCounters()
{
	_registration_counter_prev = _registration_counter;
}

void Hough::clearVariables()
{
	// initialization
	_accu = 0;
	_accu_w = 0; _accu_h = 0; _img_w = 0; _img_h = 0;

	global_x = 0; global_y = 0; global_theta = 0;

	threshold = 45;
	
	lines.clear();
	line_info.clear();
	u_shape.clear();
}
// hough transform to detect lines
void Hough::Transform(pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud)
{
	// vector for store distance and theta
	std::vector<int> r_t;

	// Create the accumulator
	// find hough_h, max distance from center (robot)
	double max_topright = sqrt(max_x * max_x + max_y * max_y);
	double max_bottomright = sqrt(max_x * max_x + min_y * min_y);
	double max_bottomleft = sqrt(min_x * min_x + min_y * min_y);
	double max_topleft = sqrt(min_x * min_x + max_y * max_y);

	double hough_h = max_topright;
	if(hough_h < max_bottomright)
				hough_h = max_bottomright;
	if(hough_h < max_bottomleft)
					hough_h = max_bottomleft;
	if(hough_h < max_topleft)
				hough_h = max_topleft;

	// accumulator size
	_accu_h = hough_h * 2.0;	// -r -> +r
	_accu_w = 180;
	_accu = (unsigned int*)calloc(_accu_h * _accu_w, sizeof(unsigned int));

	// Go over all points, get r (distance) and t (theta) assign into accumulator
	for(int i=0; i<_cloud->points.size();i++)
	{
		for(int t=0; t<180; t++)
		{
			double r = (((double)_cloud->points[i].x) * cos((double)t * deg2rad)) + (((double)_cloud->points[i].y) * sin((double)t * deg2rad));
			if((r+hough_h) > _accu_h)	//make sure round can not out of the range of the accumulator
								_accu[(int)((round(r+hough_h-1) * 180.0)) + t]++;
			else if((r+hough_h) < 0)
								_accu[(int)((round(r+hough_h+1) * 180.0)) + t]++;
			else
				_accu[(int)((round(r+hough_h) * 180.0)) + t]++;
		}	
	}

	// std::cout<<hough_h<<std::endl;
	if(_accu == 0)
	{
		// std::cout<<"Found no lines"<<std::endl;
	}

	// Go over the accumulator
	for(int r = 0; r < _accu_h; r++)
	{
		for(int t=0; t<_accu_w; t++)
		{
			if((int)_accu[(r*_accu_w) +t] >= threshold)
			{
				int max = _accu[(r*_accu_w) +t];

				// Check is this poitn a local maxima (9*9)
				for(int ly=-4; ly <= 4; ly++)
				{
					for(int lx=-4; lx<=4; lx++)
					{
						if( (ly+r>=0 && ly+r<_accu_h) && (lx+t>=0 && lx+t<_accu_w) )
						{
							if( (int)_accu[( (r+ly)*_accu_w) + (t+lx)] > max )
							{
								max = _accu[( (r+ly)*_accu_w) + (t+lx)];
								ly = lx = 5;
							}
						}
					}
				}

				if(max > (int)_accu[(r*_accu_w) + t])
					continue;
				// std::cout<<"intestive: "<<max<<std::endl;
				// std::cout<<"distance: "<<r-hough_h<<std::endl;
				// std::cout<<"theta: "<<t<<std::endl;
				
				// Get lines in x y coordinate, end points of lines
				int x1, y1, x2, y2;
				x1 = y1 = x2 = y2 = 0;

				if(t >=45 && t <= 135)	// in case of sin or cos to be 0
				{
					//y = (r - x cos(t)) / sin(t)
					x1 = min_x;
					y1 = ((double)(r-hough_h) - x1 * cos(t * deg2rad)) / sin(t * deg2rad);
					x2 = max_x;
					y2 = ((double)(r-hough_h) - x2 * cos(t * deg2rad)) / sin(t * deg2rad);
				}
				else
				{
					//x = (r - y sin(t)) / cos(t);
					y1 = min_y;
					x1 = ((double)(r-hough_h) - y1 * sin(t * deg2rad)) / cos(t * deg2rad);
					y2 = max_y;
					x2 = ((double)(r-hough_h) - y2 * sin(t * deg2rad)) / cos(t * deg2rad);
				}

				//	vector lines is used for drawlines
				lines.push_back(std::pair< std::pair<int, int>, std::pair<int, int> >(std::pair<int, int>(x1,y1), std::pair<int, int>(x2,y2)));
				// std::cout<<"lines: "<< lines.size()<<std::endl;

				// store lines info. for U shape detection 0:distance 1:theta 2:intensitive
				r_t.clear();
				r_t.push_back(r-hough_h);
				r_t.push_back(t);
				r_t.push_back(max);
				line_info.push_back(r_t);
			}
		}
	}
}


// Obtain the size of pointcloud, max and min in x and y
// void Hough::GetBoundary(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
void Hough::GetBoundary()
{
	min_x = cloud->points[0].x;
	min_y = cloud->points[0].y;
	max_x = cloud->points[0].x;
	max_y = cloud->points[0].y;

	for(int i=0; i<cloud->points.size(); i++)
	{
		if(cloud->points[i].x <= min_x)
			min_x = cloud->points[i].x;
		else if(cloud->points[i].x >= max_x)
			max_x = cloud->points[i].x;

		if(cloud->points[i].y <= min_y)
			min_y = cloud->points[i].y;
		else if(cloud->points[i].y >= max_y)
			max_y = cloud->points[i].y;
	}
}

void Hough::DrawLines()	// view pointcloud with line using pcl
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Result"));

	viewer->addPointCloud<pcl::PointXYZ> (cloud,"raw cloud");

	viewer->addSphere (pcl::PointXYZ(0,0,0), 0.1, 0, 255, 0, "original point");

	std::vector< std::pair< std::pair<int, int>, std::pair<int, int> > >::iterator it;
	int i = 0;
	for(it=lines.begin();it!=lines.end();it++)
	{
		std::string line = "line" + (char)i;
		viewer->addLine<pcl::PointXYZ>( pcl::PointXYZ(it->first.first, it->first.second, 0), pcl::PointXYZ(it->second.first, it->second.second, 0), 255, 0 , 0,line);
		i++;
	}

	while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
	
}

void Hough::GeneratePointcloud()	// generate fake 2d point cloud for testing 
{
	srand(time(0));	// random work
	// Fill in the cloud data, to create fake pointcloud to test
	cloud->is_dense = false;
	cloud->width = 20;
	cloud->height = 1;
	cloud->points.resize (cloud->width * cloud->height);
	// Create a straigh line
	for (size_t i = 0; i < cloud->points.size()-15; ++i)
	{
		cloud->points[i].x = (rand() % (100 + 1));
		cloud->points[i].y = 10;
		cloud->points[i].z = 0;
	}
	// Add nosie points
	for (size_t i = cloud->points.size()-15; i < cloud->points.size()-10; ++i)
	{
		cloud->points[i].x = 10;
		cloud->points[i].y = (rand() % (100 + 1));
		cloud->points[i].z = 0;
	}


	cloud->points[cloud->points.size()-10].x = 10;
	cloud->points[cloud->points.size()-10].y = 10;
	cloud->points[cloud->points.size()-10].z = 0;

	for (size_t i = cloud->points.size()-9; i < cloud->points.size()-4; ++i)
	{
		cloud->points[i].x = (rand() % (100 + 1));
		cloud->points[i].y = 90;
		cloud->points[i].z = 0;
	}

	cloud->points[cloud->points.size()-4].x = 10;
	cloud->points[cloud->points.size()-4].y = 90;
	cloud->points[cloud->points.size()-4].z = 0;

	for (size_t i = cloud->points.size()-2; i < cloud->points.size(); ++i)
	{
		cloud->points[i].x = (rand() % (100 + 1));
		cloud->points[i].y = (rand() % (100 + 1));
		cloud->points[i].z = 0;
	}

	cloud->points[cloud->points.size()-3].x = -10;
	cloud->points[cloud->points.size()-3].y = -10;
	cloud->points[cloud->points.size()-3].z = 0;
}

void Hough::DetectUShape()
{
	// find parallel

	std::vector<std::pair<int, int> > line_parallel;
	for(int i=0; i<line_info.size(); i++)
	{
		for(int j=i+1; j<line_info.size(); j++)
		{
			if(line_info[i][1] == line_info[j][1])
			{
				int distance = abs(line_info[i][0] - line_info[j][0]);
				// check distance between parallel lines, keep distance between 7.25 and 7.20 meter
				if(distance < 725 && distance > 720)
					line_parallel.push_back(std::pair<int, int> (i,j));
			}
		}
	}
	// std::vector<std::iterator> index_erase;
	// index_erase.clear();
	
	// for(int i=0; i<line_parallel.size(); i++)
	// {
	// 	std::cout<<"i:"<<i<<std::endl;
	// 	int distance = abs(line_info[line_parallel[i].first][0] - line_info[line_parallel[i].second][0]);
	// 	std::cout<<"distance: "<<distance<<std::endl;
	// 	if(distance > 725 || distance < 720)
	// 		index_erase.push_back(i);
	// }

	// for(int i=0; i<index_erase.size(); i++)
	// {
	// 	line_parallel.erase(line_parallel.begin() + index_erase[i]);
	// }

	std::vector<int> single_u_shape;
	single_u_shape.clear();
	// check rest line to detect right angle
	for(int i=0; i<line_parallel.size(); i++)
	{
		for(int j=0; j<line_info.size(); j++)
		{
			// 1000 to make sure the line cannot get from other side of the green house
			if((j != line_parallel[i].first) && (j != line_parallel[i].second) && abs(line_info[j][0]) < 1000)
			{
				int angle1 = abs(line_info[line_parallel[i].first][1] - line_info[j][1]);
				int angle2 = abs(line_info[line_parallel[i].second][1] - line_info[j][1]);
				if(angle1 < 92 && angle1 > 88 && angle2 < 92 && angle2 > 88)
				{// u shape vector r t r t (parallel first two) r t
					single_u_shape.push_back(line_info[line_parallel[i].first][0]);
					single_u_shape.push_back(line_info[line_parallel[i].first][1]);
					single_u_shape.push_back(line_info[line_parallel[i].first][2]);
					single_u_shape.push_back(line_info[line_parallel[i].second][0]);
					single_u_shape.push_back(line_info[line_parallel[i].second][1]);
					single_u_shape.push_back(line_info[line_parallel[i].second][2]);
					single_u_shape.push_back(line_info[j][0]);
					single_u_shape.push_back(line_info[j][1]);
					single_u_shape.push_back(line_info[j][2]);

					u_shape.push_back(single_u_shape);
					single_u_shape.clear();
				}

			}
		}
	}

	// std::cout<<"u shape size: "<<u_shape.size()<<std::endl;
	int u_shape_index = 0;
	if(u_shape.size() > 1)	// check intensitive to get one u shape
	{
		int max = u_shape[0][2] + u_shape[0][5] + u_shape[0][8];
		// int u_shape_index = 0;

		for(int i=1; i<u_shape.size(); i++)
		{
			if(max < (u_shape[i][2] + u_shape[i][5] + u_shape[i][8]))
			{
				u_shape_index = i;
				max = (u_shape[i][2] + u_shape[i][5] + u_shape[i][8]);
			}
				
		}
		
	}

	if(u_shape.size() >0)	//make sure there is u shape detected
	{	// keep the highest intensitive u shape
		single_u_shape.clear();
	
		single_u_shape.push_back(u_shape[u_shape_index][0]);
		single_u_shape.push_back(u_shape[u_shape_index][1]);
		single_u_shape.push_back(u_shape[u_shape_index][3]);
		single_u_shape.push_back(u_shape[u_shape_index][4]);
		single_u_shape.push_back(u_shape[u_shape_index][6]);
		single_u_shape.push_back(u_shape[u_shape_index][7]);

		u_shape.clear();
		u_shape.push_back(single_u_shape);
	}

	// std::cout<<"u shape size: "<<u_shape.size()<<std::endl;
}

void Hough::GetPose()
{
	//original point is the bottom left of the green house
	if(u_shape[0][4] <= 0)
	{
		if(u_shape[0][0] > 0)
		{
			global_x = abs(u_shape[0][2]) / 100.0;
		}
		else
		{
			global_x = abs(u_shape[0][0]) / 100.0;
		}

		if(u_shape[0][5] <= 90 && u_shape[0][5] > 0)
		{
			global_theta = u_shape[0][5] - 90;
		}
		else if(u_shape[0][5] < 180 && u_shape[0][5] > 90)
		{
			global_theta = u_shape[0][5];
		}

	}
	else if(u_shape[0][4] > 0)
	{
		if(u_shape[0][0] > 0)
		{
			global_x = abs(u_shape[0][0]) / 100.0;
		}
		else
		{
			global_x = abs(u_shape[0][2]) / 100.0;
		}

		if(u_shape[0][5] <= 90 && u_shape[0][5] > 0)
		{
			global_theta = u_shape[0][5] + 90;
		}
		else if(u_shape[0][5] < 180 && u_shape[0][5] > 90)
		{
			global_theta = 270 - u_shape[0][5];
		}
	}

	global_y = abs(u_shape[0][4]) / 100.0;
}

void Hough::DrawUShape()
{

	// Get lines in x y coordinate, end points of lines
	int x1, y1, x2, y2;
	x1 = y1 = x2 = y2 = 0;
	int r, t;
	r=t=0;

	std::vector< std::pair< std::pair<int, int>, std::pair<int, int> > > ushapelines;
	for (int i=0; i<u_shape[0].size(); i=i+2)	//get 3 lines of u shape
	{
		r = u_shape[0][i];
		t = u_shape[0][i+1];

		if(t >=45 && t <= 135)	// in case of sin or cos to be 0
		{
			//y = (r - x cos(t)) / sin(t)
			x1 = min_x;
			y1 = ((double)(r) - x1 * cos(t * deg2rad)) / sin(t * deg2rad);
			x2 = max_x;
			y2 = ((double)(r) - x2 * cos(t * deg2rad)) / sin(t * deg2rad);
		}
		else
		{
			//x = (r - y sin(t)) / cos(t);
			y1 = min_y;
			x1 = ((double)(r) - y1 * sin(t * deg2rad)) / cos(t * deg2rad);
			y2 = max_y;
			x2 = ((double)(r) - y2 * sin(t * deg2rad)) / cos(t * deg2rad);
		}

		//	vector lines is used for drawlines
		ushapelines.push_back(std::pair< std::pair<int, int>, std::pair<int, int> >(std::pair<int, int>(x1,y1), std::pair<int, int>(x2,y2)));
		// std::cout<<"ushapelines: "<< ushapelines.size()<<std::endl;
	}
	


	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Result"));

	viewer->addPointCloud<pcl::PointXYZ> (cloud,"raw cloud");

	// viewer->addSphere (pcl::PointXYZ(0,0,0), 0.1, 0, 255, 0, "original point");

	std::vector< std::pair< std::pair<int, int>, std::pair<int, int> > >::iterator it;
	int i = 0;
	for(it=ushapelines.begin();it!=ushapelines.end();it++)
	{
		std::string line = "line" + (char)i;
		viewer->addLine<pcl::PointXYZ>( pcl::PointXYZ(it->first.first, it->first.second, 0), pcl::PointXYZ(it->second.first, it->second.second, 0), 255, 0 , 0,line);
		i++;
	}

	while (!viewer->wasStopped ())
  	{
	    viewer->spinOnce (100);
	    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
}