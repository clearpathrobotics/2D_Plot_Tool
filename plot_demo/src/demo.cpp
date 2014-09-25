#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "ecl/geometry.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "plot_tool/PlotPath.h"
#include "plot_tool/PlotPose.h"

ros::ServiceClient path_srv_h;
ros::ServiceClient pose_srv_h;

/**
 * Construct_Path_Msg function
 * Used to populate a nav_msgs::Path given a list of x and y coordinates
 * @param x double* pointer to array containing x coordinates
 * @param y double* pointer to array containing y coordinates
 * @return msg the constructed nav_msgs::Path message
 */
nav_msgs::Path Construct_Path_Msg(double* x, double *y, int length)
{
	nav_msgs::Path msg;
	std::vector<geometry_msgs::PoseStamped> poses(length);
	for (int i = 0; i < length; i++)
	{
		poses.at(i).pose.position.x = x[i];
		poses.at(i).pose.position.y = y[i];
	}
	msg.poses = poses;
	return msg;
}

/**
 * run_demo function
 * This function plots what you see in the demo
 * This is an ugly hard coded mess because its only purpose is to demo plot_tool
 * It is slapped together from code used to test viability of ECL's spline interpolator
 */
void run_demo()
{
	plot_tool::PlotPath path_srv;

	//clear graph
	path_srv.request.append=false;
	for (int i = 0; i < 7; i++)
	{
		path_srv.request.series=i;
		path_srv_h.call(path_srv);
	}

	ros::Duration(2).sleep();

	//draw walls using nav_msg::Path
	double wall_x[] = {0, 0, 4, 4, 10, 10, 6, 6, 0};
	double wall_y[] = {0, 2, 2, 10, 10, 8, 8, 0, 0};
	nav_msgs::Path wall_msg = Construct_Path_Msg(wall_x, wall_y, sizeof(wall_x)/sizeof(double));
	//draw walls
	path_srv.request.msg = wall_msg;
	path_srv.request.series=6;
	path_srv.request.append=false;
	path_srv.request.symbol='-';
	path_srv.request.symbol_size=10;
	path_srv_h.call(path_srv);

	ros::Duration(2).sleep();

	//draw initial wps using nav_msg::Path
	int wp_total = 4;
	double * wp_x = (double*)malloc(sizeof(double)*wp_total);
	double * wp_y = (double*)malloc(sizeof(double)*wp_total);
	wp_x[0] = 1; wp_x[1] = 5; wp_x[2] = 5; wp_x[3] = 9;
	wp_y[0] = 1; wp_y[1] = 1; wp_y[2] = 9; wp_y[3] = 9;
	nav_msgs::Path wp_msg = Construct_Path_Msg(wp_x, wp_y, wp_total);

	//draw wps as points
	path_srv.request.msg = wp_msg;
	path_srv.request.series=0;
	path_srv.request.append=false;
	path_srv.request.symbol='o';
	path_srv.request.symbol_size=10;
	path_srv_h.call(path_srv);

	ros::Duration(2).sleep();

	//connect wp as a line
	path_srv.request.symbol='-';
	path_srv.request.append=true;
	path_srv_h.call(path_srv);

	ros::Duration(2).sleep();

	//spline fitting
	bool safe = false;
	double * x_ptr;
	double * y_ptr;
	x_ptr = wp_x;
	y_ptr = wp_y;
	int spline_series = 2;
	nav_msgs::Path spline_msg;
	ecl::CubicSpline spline_x;
	ecl::CubicSpline spline_y;
	//iterate until we get a collision free spline
	while (!safe)
	{
		safe = true;

		//find a spline to fit the linear path
		ecl::Array<double> t_set(wp_total);
		ecl::Array<double> x_set(wp_total);
		ecl::Array<double> y_set(wp_total);
		for (int i = 0; i < wp_total; i++)
		{
			t_set[i] = (double)i;
			x_set[i] = x_ptr[i];
			y_set[i] = y_ptr[i];
		}

		//draw points used to construct spline
		nav_msgs::Path spline_pts_msg = Construct_Path_Msg(x_ptr, y_ptr, wp_total);
		path_srv.request.msg = spline_pts_msg;
		path_srv.request.series=(++spline_series%5)+1;
		path_srv.request.append=false;			
		path_srv.request.symbol='d';
		path_srv.request.symbol_size=10;
		path_srv_h.call(path_srv);

		ros::Duration(1).sleep();

		//spline fit with ECL geometry
		spline_x = ecl::CubicSpline::Natural(t_set,x_set);
		spline_y = ecl::CubicSpline::Natural(t_set,y_set);
		int spline_pts = (wp_total-1)*100+1;
		double sx[spline_pts];
		double sy[spline_pts];
		for (int i = 0; i < spline_pts; i++)
		{
			sx[i] = spline_x(i/100.0);
			sy[i] = spline_y(i/100.0);

			//crappy implicit collision check, this should be replaced with something better
			if (sx[i] <=4.1 && sy[i] >= 1.9)
			{
				safe = false;
			}
			else if (sx[i] > 5.9 && sy[i] <= 8.1)
			{
				safe = false;
			}
			else if (sx[i] <= 0.1 || sx[i] >= 9.9)
			{
				safe = false;
			}
			else if (sy[i] <= 0.1 || sy[i] >= 9.9)
			{
				safe = false;
			}
		}
		spline_msg = Construct_Path_Msg(sx, sy, sizeof(sx)/sizeof(double));
	
		//draw spline
		path_srv.request.msg = spline_msg;
		path_srv.request.series=(spline_series%5)+1;
		path_srv.request.append=true;			
		path_srv.request.symbol='-';
		path_srv.request.symbol_size=10;
		path_srv_h.call(path_srv);

		if (!safe)
		{
			//add more points to force spline closer to path
			wp_total = wp_total*2-1;
			double * new_wpx = (double*)malloc(sizeof(double)*wp_total);
			double * new_wpy = (double*)malloc(sizeof(double)*wp_total);
			for (int i = 0; i < wp_total; i++)
			{
				if (i%2 == 0)
				{
					new_wpx[i] = x_ptr[i/2];
					new_wpy[i] = y_ptr[i/2];
				}	
				else
				{
					new_wpx[i] = (x_ptr[(int)i/2]+x_ptr[(int)(i/2)+1])/2.0;
					new_wpy[i] = (y_ptr[(int)i/2]+y_ptr[(int)(i/2)+1])/2.0;
				}
			}

			free(x_ptr);
			free(y_ptr);
			x_ptr = new_wpx;
			y_ptr = new_wpy;
		}

		ros::Duration(2).sleep();
	}

	//clear graph except the spline
	nav_msgs::Path empty_path_msg;
	path_srv.request.append=false;
	for (int i = 0; i < 6; i++)
	{
		if (i!=(spline_series%5)+1)
		{
			path_srv.request.msg=empty_path_msg;
		}
		else
		{
			path_srv.request.msg=spline_msg;
		}
		path_srv.request.series=i;
		path_srv_h.call(path_srv);
	}

	ros::Duration(2).sleep();

	//draw the vehicle traversing the path using pose messages
	geometry_msgs::Pose vehicle_msg;

	plot_tool::PlotPose pose_srv;
	pose_srv.request.msg = vehicle_msg;
	pose_srv.request.series=1;
	pose_srv.request.append=false;			
	pose_srv.request.symbol='s';
	pose_srv.request.symbol_size=50;

	int t_max = wp_total-1;
	int steps = t_max*20+1;
	
	//interpolate existing segments to add more control points for a tighter spline
	for (int i = 0; i < steps; i++)
	{
		pose_srv.request.msg.position.x = spline_x(i/20.0);
		pose_srv.request.msg.position.y = spline_y(i/20.0);

		pose_srv_h.call(pose_srv);

		ros::Duration(0.05).sleep();
	}

	ros::Duration(2).sleep();
}

/**
 * Main Function 
 * Used to bring up the ROS node and subscribe to plotting services.
 */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "cubic_spline_interpolation");
  	ros::NodeHandle n;

	//subscribe to the plotting services
  	path_srv_h = n.serviceClient<plot_tool::PlotPath>("plot_tool/draw_path");
	pose_srv_h = n.serviceClient<plot_tool::PlotPose>("plot_tool/draw_pose");

	while (!ros::isShuttingDown())
	{
		run_demo();
		ros::spinOnce();
	}

  	return 0;
}
