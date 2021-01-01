#include <iostream>
#define _USE_MATH_DEFINES
#include <math.h>

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/LaserScan.h>
//#include <tf/transform_listener.h>
//#include <move_base_msgs/MoveBaseAction.h>

#define PI 3.1415926535

//#define DEBUG_LIDAR
//#define DEBUG_LOOP

#define FRONTAL_DIST_THR 0.3 		// [m], stop region distance
#define FRONTAL_DIST_THR_2 0.6 		// [m], slowing region distance
#define HALF_FRONTAL_WIDTH 0.3 		// [m], stop region width
#define HALF_FRONTAL_WIDTH_2 0.4	// [m], slowing region width	
//#define TF_WATCHDOG_COUNT 20 				// [counts], cycles of tf aging
#define MAX_LIDAR_POINTS 2000 		// turtlebot lidar:360 points, YDLIDAR G4: 1524 points
//#define CLUTTER_ANGLE 100 					// [degs], points filtered
//#define CLUTTER_RANGE 5 					// [m], points filtered

ros::Subscriber lidar_subscriber;
ros::Publisher lidar_publisher;
ros::Publisher coeff_publisher;

sensor_msgs::LaserScan scan_msg;
std_msgs::Float32 dist_coeff;

float lidar_dist_x[MAX_LIDAR_POINTS];
float lidar_dist_y[MAX_LIDAR_POINTS];
float lidar_angle[MAX_LIDAR_POINTS];
float lidar_angle_min;
float lidar_angle_inc;
float lidar_angle_rad;
float min_frontal_dist = 1.0;
float scaling_const = FRONTAL_DIST_THR_2/5000;
int mode = 0;


void lidar_callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
	
	scan_msg = *msg;
	
	int i=0;
	int n_points = (scan_msg.angle_max-scan_msg.angle_min)/scan_msg.angle_increment;
	int point_mid = n_points/2;

	#ifdef DEBUG_LIDAR
		ROS_INFO("Received: \n");
		ROS_INFO("angle min %f", scan_msg.angle_min);
		ROS_INFO("angle max %f", scan_msg.angle_max);
		ROS_INFO("angle increment %f", scan_msg.angle_increment);
		ROS_INFO("test value %f", scan_msg.ranges[20]);
		i=0;
		ROS_INFO("At i = %d, front: %f, intensity: %f", i, scan_msg.ranges[i], scan_msg.intensities[i]);
		i=n_points/4;
		ROS_INFO("At i = %d, front: %f, intensity: %f", i, scan_msg.ranges[i], scan_msg.intensities[i]);
		i=n_points/2;
		ROS_INFO("At i = %d, front: %f, intensity: %f", i, scan_msg.ranges[i], scan_msg.intensities[i]);
		i=n_points*3/4;
		ROS_INFO("At i = %d, front: %f, intensity: %f", i, scan_msg.ranges[i], scan_msg.intensities[i]);
	#endif

	min_frontal_dist=1.0;
	
	for(i=0;i<n_points;i++) {
		//convert coordinates from polar to cartesian representation 
		lidar_angle[i] = scan_msg.angle_min + i*scan_msg.angle_increment;
		lidar_dist_x[i] = cos(lidar_angle[i])*scan_msg.ranges[i];
		lidar_dist_y[i] = sin(lidar_angle[i])*scan_msg.ranges[i];
		if(scan_msg.intensities[i] > 0) {
			//Adjust Intensities
			if((lidar_dist_y[i]<HALF_FRONTAL_WIDTH_2)&&(lidar_dist_y[i]>-HALF_FRONTAL_WIDTH_2)&&(lidar_dist_x[i]<FRONTAL_DIST_THR_2)) {
				scan_msg.intensities[i] = lidar_dist_x[i]/scaling_const;
				if(lidar_dist_x[i]<min_frontal_dist) {
				   min_frontal_dist = lidar_dist_x[i];		
				}
			}	
		}
	}
	
	if(min_frontal_dist<=FRONTAL_DIST_THR){
		dist_coeff.data=0;
	} else if ((min_frontal_dist>FRONTAL_DIST_THR)&&(min_frontal_dist<=FRONTAL_DIST_THR_2)) {
		dist_coeff.data= (min_frontal_dist-FRONTAL_DIST_THR)/(FRONTAL_DIST_THR_2-FRONTAL_DIST_THR);
	} else if (min_frontal_dist>FRONTAL_DIST_THR_2){
		dist_coeff.data=1.0;
	} else {
		dist_coeff.data=0;
	}
	scan_msg.intensities[point_mid] = FRONTAL_DIST_THR_2/scaling_const; //fondoscala
	
	lidar_publisher.publish(scan_msg);
	coeff_publisher.publish(dist_coeff);
}

int main(int argc, char **argv){
	
	ros::init(argc,argv,"lidar_supervisor");
	ros::NodeHandle node_obj;

	lidar_subscriber = node_obj.subscribe("/scan", 10, lidar_callback);
	lidar_publisher = node_obj.advertise<sensor_msgs::LaserScan>("/scan_filter",4);
	coeff_publisher = node_obj.advertise<std_msgs::Float32>("/dist_coeff",5);

    ros::Rate rate(10.0);
	
    while (node_obj.ok()) {
		#ifdef DEBUG_LOOP
			ROS_INFO("loop test min_dist %f", min_frontal_dist);	
		#endif
		rate.sleep();
		ros::spinOnce();
    }
}

