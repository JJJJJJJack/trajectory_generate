#include "ros/ros.h"
#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>

#include <fstream>

#define pi 3.14159265354

#define TRAJEC_CIRCLE           1
#define TRAJEC_SCAN             2
#define TRAJEC_STEP             3
#define TRAJEC_LINE_X           4
#define TRAJEC_CHIRP            5
#define TRAJEC_FF               6
#define TRAJEC_L1_CIRCLE        7
#define FLIP_OVER               8
#define TRAJEC_LINE_Y           9
#define TRAJEC_SQUARE           10
#define TRAJEC_AGGRESSIVEEIGHT  11
#define TRAJEC_FLOWER           12
#define TRAJEC_FIGUREEIGHT      13

using namespace std;

double param_x, param_y, param_z, param_radius, param_yaw, param_round_period;
int param_rate, param_trajectory_style;
bool param_enable_tracking;
bool ready_to_track = false;
nav_msgs::Odometry sub_CurrPose;
float last_x,last_y,last_z;

void Subscribe_CurrPose(const nav_msgs::Odometry& CurrPose)
{
    //FIXME
    sub_CurrPose = CurrPose;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Trajectory_generate");

  ros::NodeHandle n("~");
  ros::NodeHandle nh;
  ros::Publisher Trajectory_pub = nh.advertise<geometry_msgs::PoseStamped>("goal", 1000);
  ros::Publisher Trajectory_velocity_pub = nh.advertise<geometry_msgs::Vector3>("goal_velocity", 1000);
  ros::Subscriber CurrPose_Sub = nh.subscribe("pose", 1000, Subscribe_CurrPose);

  n.getParam("x", param_x);
  n.getParam("y", param_y);
  n.getParam("z", param_z);
  n.getParam("yaw", param_yaw);
  n.getParam("rate", param_rate);
  n.getParam("radius", param_radius);
  n.getParam("enable_tracking", param_enable_tracking);
  n.getParam("round_period", param_round_period);
  n.getParam("trajectory_style", param_trajectory_style);
  
  ifstream in("/home/darc/catkin_ws/src/trajectory_generate/data/chirp.txt"); 
  ifstream in_FF("/home/darc/catkin_ws/src/trajectory_generate/data/FF_Trajectory.txt"); 

  ros::Rate loop_rate(param_rate);

  if(param_enable_tracking)
  {
      if(param_trajectory_style == TRAJEC_CIRCLE)
          ROS_INFO("\nTracking Trajectory\nradius: %f Rate: %d\n", param_radius, param_rate);
      if(param_trajectory_style == TRAJEC_SCAN)
          ROS_INFO("\n\nSCANNING....\n\n");
      if(param_trajectory_style == TRAJEC_L1_CIRCLE)
          ROS_INFO("\n\nL1 Tracking Circle\n\n");
      if(param_trajectory_style == TRAJEC_FF)
          ROS_INFO("\n\nFeed-forward Tracking\n\n");
  }else{
      ROS_INFO("\nPosition Hold\n Goal Point: %f, %f, %f\nYaw: %f", param_x, param_y, param_z, param_yaw * 57.3);
  }

  double devide_variable = param_round_period * param_rate / 2.0 / pi;

cout<<devide_variable<<endl;

  int count = 0;
  while (ros::ok())
  {
    tf::Quaternion quaternion;
    geometry_msgs::PoseStamped msg;
    geometry_msgs::Vector3     msg_velocity;
    msg.header.seq += 1;
    msg.header.stamp = ros::Time::now();
    if(ready_to_track == false)
    {
        if(sub_CurrPose.pose.pose.position.z > 0.8 * param_z && count > 30 * param_rate){
	    ready_to_track = true;
	    ROS_INFO("\nReach height. Start tracking trajectory\n");
	    count = 0;
	}
    }
    if(param_enable_tracking == false || ready_to_track == false)
    {
        msg.pose.position.x = param_x;
        msg.pose.position.y = param_y;
        msg.pose.position.z = param_z;
	msg_velocity.x = 0;
        quaternion.setRPY(0, 0, param_yaw);
        msg.pose.orientation.x = quaternion.x();
        msg.pose.orientation.y = quaternion.y();
        msg.pose.orientation.z = quaternion.z();
        msg.pose.orientation.w = quaternion.w();
        Trajectory_pub.publish(msg);
	Trajectory_velocity_pub.publish(msg_velocity);
    }else if(ready_to_track == true && param_trajectory_style == TRAJEC_LINE_X){
        msg.pose.position.x = param_radius * sin(count / devide_variable);
	//msg.pose.position.x = 0;
        msg.pose.position.y = 0;
	//msg.pose.position.y = param_radius * (1.0 - cos(count / devide_variable));
        msg.pose.position.z = param_z;
	float yaw = count / devide_variable;
        quaternion.setRPY(0, 0, yaw);
        msg.pose.orientation.x = quaternion.x();
        msg.pose.orientation.y = quaternion.y();
        msg.pose.orientation.z = quaternion.z();
        msg.pose.orientation.w = quaternion.w();
        Trajectory_pub.publish(msg);
    }else if(ready_to_track == true && param_trajectory_style == TRAJEC_CIRCLE){
	//double circle = fabs(cos(0.04 * count / 50.0));
        msg.pose.position.x = param_radius * sin(count / devide_variable);
        msg.pose.position.y = param_radius * (-cos(count / devide_variable));
        msg.pose.position.z = param_z;
	float yaw = count / devide_variable;
        quaternion.setRPY(0, 0, 0);
        msg.pose.orientation.x = quaternion.x();
        msg.pose.orientation.y = quaternion.y();
        msg.pose.orientation.z = quaternion.z();
        msg.pose.orientation.w = quaternion.w();
        Trajectory_pub.publish(msg);
    }else if(ready_to_track == true && param_trajectory_style == TRAJEC_STEP){
        msg.pose.position.z = param_z;
	msg.pose.position.x = param_x + 0.5;
	msg.pose.position.y = param_y + 0.5;
	float yaw = count / devide_variable;
        quaternion.setRPY(0, 0, yaw);
        msg.pose.orientation.x = quaternion.x();
        msg.pose.orientation.y = quaternion.y();
        msg.pose.orientation.z = quaternion.z();
        msg.pose.orientation.w = quaternion.w();
        Trajectory_pub.publish(msg);
    }else if(ready_to_track == true && param_trajectory_style == TRAJEC_SCAN){
	if((int)floor(count / 200.0) % 2 == 0){
	    msg.pose.position.x = ((float)count - floor(count / 200.0) * 200.0) / 400.0;
	}else{
	    msg.pose.position.x = 0.5 - ((float)count - floor(count / 200.0) * 200.0) / 400.0;
	}
	if((int)floor(count / 800.0) % 2 == 0){
	    msg.pose.position.y = ((float)count - floor(count / 800.0) * 800.0) / 1600.0;
	}else{
	    msg.pose.position.y = 0.5 - ((float)count - floor(count / 800.0) * 800.0) / 1600.0;
	}
        msg.pose.position.z = param_z;
	float yaw = count / devide_variable;
        quaternion.setRPY(0, 0, yaw);
        msg.pose.orientation.x = quaternion.x();
        msg.pose.orientation.y = quaternion.y();
        msg.pose.orientation.z = quaternion.z();
        msg.pose.orientation.w = quaternion.w();
        Trajectory_pub.publish(msg);
    }else if(ready_to_track == true && param_trajectory_style == TRAJEC_CHIRP){
	char buffer[256];
	double x_temp;
	if(!in.eof()){
	    in.getline (buffer,100);
	    x_temp = 0.05 * atof(buffer);
	}else{
	    x_temp = param_x;
	}	    
        msg.pose.position.x = param_x;
        msg.pose.position.y = x_temp;
        msg.pose.position.z = param_z;
	float yaw = count / devide_variable;
        quaternion.setRPY(0, 0, yaw);
        msg.pose.orientation.x = quaternion.x();
        msg.pose.orientation.y = quaternion.y();
        msg.pose.orientation.z = quaternion.z();
        msg.pose.orientation.w = quaternion.w();
        Trajectory_pub.publish(msg);
    }else if(ready_to_track == true && param_trajectory_style == TRAJEC_FF){
	char buffer[256];
	float x_temp,y_temp,z_temp;
	if(!in_FF.eof()){
	    in_FF.getline (buffer,100);
	    sscanf(buffer,"%f %f %f",&x_temp, &y_temp, &z_temp);
	    last_x = x_temp;
	    last_y = y_temp;
	    last_z = (z_temp)+0.7;
	}else{
	    x_temp = last_x;
	    y_temp = last_y;
	    z_temp = last_z;
	}	    
        msg.pose.position.x = x_temp;
        msg.pose.position.y = y_temp;
        msg.pose.position.z = last_z;
	float yaw = count / devide_variable;
        quaternion.setRPY(0, 0, yaw);
        msg.pose.orientation.x = quaternion.x();
        msg.pose.orientation.y = quaternion.y();
        msg.pose.orientation.z = quaternion.z();
        msg.pose.orientation.w = quaternion.w();
        Trajectory_pub.publish(msg);
    }else if(ready_to_track == true && param_trajectory_style == TRAJEC_L1_CIRCLE){
        msg.pose.position.x = param_radius * sin(count / devide_variable);
        msg.pose.position.y = param_radius * (- cos(count / devide_variable));
        msg.pose.position.z = param_z;
	msg_velocity.x = 2.0 * pi * param_radius / param_round_period;
	float yaw = count / devide_variable;
        quaternion.setRPY(0, 0, yaw);
        msg.pose.orientation.x = quaternion.x();
        msg.pose.orientation.y = quaternion.y();
        msg.pose.orientation.z = quaternion.z();
        msg.pose.orientation.w = quaternion.w();
        Trajectory_pub.publish(msg);
	Trajectory_velocity_pub.publish(msg_velocity);
    }else if(ready_to_track == true && param_trajectory_style == FLIP_OVER){
        msg.pose.position.x = param_radius * 0.001 * sin(count / devide_variable);
        msg.pose.position.y = param_radius * 0.001 * (1.0 - cos(count / devide_variable));
        msg.pose.position.z = param_z;
	float yaw = count / devide_variable;
        quaternion.setRPY(0, 0, yaw);
        msg.pose.orientation.x = quaternion.x();
        msg.pose.orientation.y = quaternion.y();
        msg.pose.orientation.z = quaternion.z();
        msg.pose.orientation.w = quaternion.w();
        Trajectory_pub.publish(msg);
    }else if(ready_to_track == true && param_trajectory_style == TRAJEC_LINE_Y){
        //msg.pose.position.x = param_radius * sin(count / devide_variable);
	msg.pose.position.x = 0;
        //msg.pose.position.y = 0;
	msg.pose.position.y = param_radius * (- cos(count / devide_variable));
        msg.pose.position.z = param_z;
	float yaw = count / devide_variable;
        quaternion.setRPY(0, 0, yaw);
        msg.pose.orientation.x = quaternion.x();
        msg.pose.orientation.y = quaternion.y();
        msg.pose.orientation.z = quaternion.z();
        msg.pose.orientation.w = quaternion.w();
        Trajectory_pub.publish(msg);
    }else if(ready_to_track == true && param_trajectory_style == TRAJEC_SQUARE){
	float Tf = param_round_period * param_rate;
	if(count <= Tf/4){
	    msg.pose.position.x = -param_radius + 8 * param_radius * count / Tf;
	    msg.pose.position.y = -param_radius;
	}else if(count <= Tf/2){
	    msg.pose.position.x = param_radius;
	    msg.pose.position.y = -param_radius + 8 * param_radius * (count - Tf/4) / Tf;
	}else if(count <= 3*Tf/4){
	    msg.pose.position.x = param_radius - 8 * param_radius * (count - Tf/2) / Tf;
	    msg.pose.position.y = param_radius;
	}else if(count < Tf){
	    msg.pose.position.x = -param_radius;
	    msg.pose.position.y = param_radius - 8 * param_radius * (count - 3*Tf/4) / Tf;
	}else if(count == Tf){
	    msg.pose.position.x = -param_radius;
	    msg.pose.position.y = param_radius - 8 * param_radius * (count - 3*Tf/4) / Tf;
	    count = 0;
	}
        msg.pose.position.z = param_z;
        quaternion.setRPY(0, 0, 0);
        msg.pose.orientation.x = quaternion.x();
        msg.pose.orientation.y = quaternion.y();
        msg.pose.orientation.z = quaternion.z();
        msg.pose.orientation.w = quaternion.w();
        Trajectory_pub.publish(msg);
    }else if(ready_to_track == true && param_trajectory_style == TRAJEC_AGGRESSIVEEIGHT){
	msg.pose.position.x = param_radius * sin((count - 0.5 * param_round_period * param_rate) / devide_variable) * cos((count - 0.5 * param_round_period * param_rate) / devide_variable);
	msg.pose.position.y = param_radius * cos((count - 0.5 * param_round_period * param_rate) / devide_variable);
        msg.pose.position.z = param_z;
        quaternion.setRPY(0, 0, 0);
        msg.pose.orientation.x = quaternion.x();
        msg.pose.orientation.y = quaternion.y();
        msg.pose.orientation.z = quaternion.z();
        msg.pose.orientation.w = quaternion.w();
        Trajectory_pub.publish(msg);
    }else if(ready_to_track == true && param_trajectory_style == TRAJEC_FLOWER){
	msg.pose.position.x = 1.414213562 * param_radius * (cos(2.0 * count / devide_variable)>=0?1.0:-1.0) * sqrt(fabs(cos(2.0*count / devide_variable))) * cos(count / devide_variable);
	msg.pose.position.y = 1.414213562 * param_radius * (cos(2.0 * count / devide_variable)>=0?1.0:-1.0) * sqrt(fabs(cos(2.0*count / devide_variable))) * sin(count / devide_variable);
        msg.pose.position.z = param_z;
        quaternion.setRPY(0, 0, 0);
        msg.pose.orientation.x = quaternion.x();
        msg.pose.orientation.y = quaternion.y();
        msg.pose.orientation.z = quaternion.z();
        msg.pose.orientation.w = quaternion.w();
        Trajectory_pub.publish(msg);
    }else if(ready_to_track == true && param_trajectory_style == TRAJEC_FIGUREEIGHT){
	msg.pose.position.x = param_radius * sin(count / devide_variable) * cos(count / devide_variable) / (1.0 + sin(count / devide_variable) * sin(count / devide_variable));
	msg.pose.position.y = param_radius * cos(count / devide_variable) / (1.0 + sin(count / devide_variable) * sin(count / devide_variable));
        msg.pose.position.z = param_z;
        quaternion.setRPY(0, 0, 0);
        msg.pose.orientation.x = quaternion.x();
        msg.pose.orientation.y = quaternion.y();
        msg.pose.orientation.z = quaternion.z();
        msg.pose.orientation.w = quaternion.w();
        Trajectory_pub.publish(msg);
    }
    ros::spinOnce();

    loop_rate.sleep();
    
    ++count;
  }

  return 0;
}
