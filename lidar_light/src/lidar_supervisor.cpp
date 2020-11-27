#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>

#define PI 3.1415926535

//#define DEBUG_LIDAR
#define DEBUG_LOOP
#define FRONTAL_DIST_THR 0.3 //metri, soglia di arresto frontale
#define FRONTAL_DIST_THR_2 0.6
#define HALF_FRONTAL_WIDTH 0.3 //metri, larghezza fronte libero veicolo
#define HALF_FRONTAL_WIDTH_2 0.4
#define TF_WATCHDOG_COUNT 20 //counts, numero di cicli invecchiamento tf
#define MAX_LIDAR_POINTS 4000 //turtlebot lidar:360 punti, YDLIDAR G4: 1524 punti
#define CLUTTER_ANGLE 100 // in gradi, punti esclusi dal lidar
#define CLUTTER_RANGE 5 // in metri, punti esclusi dal lidar

ros::Publisher cmd_vel_publisher;
ros::Subscriber cmd_vel_subscriber;
ros::Subscriber lidar_subscriber;
ros::Publisher lidar_publisher;
geometry_msgs::Twist twist_ref_keyboard, twist_ref_autonomous, twist_ref;
sensor_msgs::LaserScan scan_msg;


float lidar_dist_x[MAX_LIDAR_POINTS];
float lidar_dist_y[MAX_LIDAR_POINTS];
float lidar_angle[MAX_LIDAR_POINTS];
float lidar_angle_min;
float lidar_angle_inc;
float lidar_angle_rad;
float min_frontal_dist = 1.0;
int mode = 0;


void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
	twist_ref_keyboard = *msg;	
	//ROS_INFO("callback vel test msg x=%f keyboard x = %f", msg-> linear.x, twist_ref_keyboard.linear.x);
}

void lidar_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
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

	//calcola distanze punti rispetto piano frontale (asse y)

	for(i=0;i<n_points;i++) //converti punti lidar da polari a cartesiani
	{	
		
		lidar_angle[i] = scan_msg.angle_min + i*scan_msg.angle_increment;//*PI/180;
		lidar_dist_x[i] = cos(lidar_angle[i])*scan_msg.ranges[i];
		lidar_dist_y[i] = sin(lidar_angle[i])*scan_msg.ranges[i];
	}

	//trova punti su fascia centrale
	min_frontal_dist = 1.0;
	for(i=0;i<n_points;i++)
	{
		if(scan_msg.intensities[i] > 0)
		{

			//marca soglia esterna
			if((lidar_dist_y[i]<HALF_FRONTAL_WIDTH_2)&&(lidar_dist_y[i]>-HALF_FRONTAL_WIDTH_2)&&(lidar_dist_x[i]<FRONTAL_DIST_THR_2))
			{
				scan_msg.intensities[i] = 2000;
			}

			//marca soglia interna
			if((lidar_dist_y[i]<HALF_FRONTAL_WIDTH)&&(lidar_dist_y[i]>-HALF_FRONTAL_WIDTH)&&(lidar_dist_x[i]<FRONTAL_DIST_THR))
			{
				scan_msg.intensities[i] = 3000;
			}


			if((lidar_dist_y[i]<HALF_FRONTAL_WIDTH)&&(lidar_dist_y[i]>-HALF_FRONTAL_WIDTH)&&(lidar_dist_x[i]>0))
			{
				if(lidar_dist_x[i]<min_frontal_dist)
				{
				   min_frontal_dist = lidar_dist_x[i];
				   //ROS_INFO("min_dist %f, l_x %f, l_y %f, i %d", min_frontal_dist, lidar_dist_x[i], lidar_dist_y[i], i );		
				}
			}	
		}

		scan_msg.intensities[point_mid] = 5000; //fondoscala
	}

	lidar_publisher.publish(scan_msg);

}

int main(int argc, char **argv){
	ros::init(argc,argv,"danbot_control");
	ros::NodeHandle node_obj;

	//tf::TransformListener tf_listener;

	//ros::Subscriber number_subscriber = node_obj.subscribe("/numbers",10,number_callback);
	cmd_vel_publisher = node_obj.advertise<geometry_msgs::Twist>("/cmd_vel",1);
	cmd_vel_subscriber = node_obj.subscribe("/cmd_vel_keyboard", 10, cmd_vel_callback);
	lidar_subscriber = node_obj.subscribe("/scan", 10, lidar_callback);
	lidar_publisher = node_obj.advertise<sensor_msgs::LaserScan>("/scan_filter",4);

     tf::TransformListener listener;
   
     ros::Rate rate(10.0);
     ROS_INFO("avvio loop");
     while (node_obj.ok())
     {
	

	twist_ref = twist_ref_keyboard;
	if(min_frontal_dist < FRONTAL_DIST_THR)
	{	
	 	if(twist_ref.linear.x > 0)
		{
			twist_ref.linear.x = 0;
		}
	}
	cmd_vel_publisher.publish(twist_ref);
	#ifdef DEBUG_LOOP
		ROS_INFO("loop test min_dist %f, ref keyboard=%f", min_frontal_dist, twist_ref_keyboard.linear.x);	
	#endif
	rate.sleep();
	ros::spinOnce();
     }


}

