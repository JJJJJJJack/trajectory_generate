#include "ros/ros.h"
#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>

#include <fstream>

#include <Eigen/Eigen>

#define DEBUG_TRAJECTORY

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
#define TRAJEC_MINSNAP          14

using namespace std;

double param_x, param_y, param_z, param_radius, param_yaw, param_round_period;
int param_rate, param_trajectory_style, current_seg_num;
bool param_enable_tracking;
double param_vehicle_weight, param_vehicle_Jyy, param_vehicle_arm_length;

bool ready_to_track = false, poly_ready = false, poly_coeff_ready = false, poly_time_ready = false;
nav_msgs::Odometry sub_CurrPose;
float last_x,last_y,last_z;
std_msgs::Float64MultiArray poly_coeff, time_alloc;

double saturate(double input, double min, double max)
{
  if(input <= min)
    return min;
  if(input >= max)
    return max;
  return input;
}

void Subscribe_CurrPose(const nav_msgs::Odometry& CurrPose)
{
  //FIXME
  sub_CurrPose = CurrPose;
}

void Subscribe_PolyCoeff(const std_msgs::Float64MultiArray& poly_coeff_msg)
{
  poly_coeff_ready = true;
  poly_coeff = poly_coeff_msg;
}

void Subscribe_TimeAlloc(const std_msgs::Float64MultiArray& time_alloc_msg)
{
  poly_time_ready = true;
  time_alloc = time_alloc_msg;
}

Eigen::Vector3d getMinSnapPose(double time_in_trajectory, int poly_order, int coeff_segment_base)
{
  Eigen::Vector3d pose(0,0,0);
  for(int order = 0; order < poly_order; order++){
    pose(0) += poly_coeff.data[coeff_segment_base + poly_order - order - 1]*pow(time_in_trajectory, poly_order - order - 1);
    pose(1) += poly_coeff.data[coeff_segment_base + poly_order*1 + poly_order - order - 1]*pow(time_in_trajectory, poly_order - order - 1);
    pose(2) += poly_coeff.data[coeff_segment_base + poly_order*2 + poly_order - order - 1]*pow(time_in_trajectory, poly_order - order - 1);
  }
  return pose;
}

Eigen::Vector3d getMinSnapAcc(double time_in_trajectory, int poly_order, int coeff_segment_base)
{
  Eigen::Vector3d acc(0,0,0);
  for(int order = 0; order < poly_order-2; order++){
    acc(0) += (poly_order - order - 1)*(poly_order - order - 2)*poly_coeff.data[coeff_segment_base + poly_order*0 + poly_order - order - 1]*pow(time_in_trajectory, poly_order - order - 3);
    acc(1) += (poly_order - order - 1)*(poly_order - order - 2)*poly_coeff.data[coeff_segment_base + poly_order*1 + poly_order - order - 1]*pow(time_in_trajectory, poly_order - order - 3);
    acc(2) += (poly_order - order - 1)*(poly_order - order - 2)*poly_coeff.data[coeff_segment_base + poly_order*2 + poly_order - order - 1]*pow(time_in_trajectory, poly_order - order - 3);
  }
  return acc;
}

double getMinSnapYawRate(double time_in_trajectory, int poly_order, int coeff_segment_base)
{
  double minsnap_yaw_rate=0;
  for(int order = 0; order < poly_order-1; order++){
    minsnap_yaw_rate += (poly_order - order - 1)*poly_coeff.data[coeff_segment_base + poly_order*3 + poly_order - order - 1]*pow(time_in_trajectory, poly_order - order - 2);
  }
  return minsnap_yaw_rate;
}

double getMinSnapYaw(double time_in_trajectory, int poly_order, int coeff_segment_base)
{
  double minsnap_yaw=0;
  for(int order = 0; order < poly_order; order++){
    minsnap_yaw += poly_coeff.data[coeff_segment_base + poly_order*3 + poly_order - order - 1]*pow(time_in_trajectory, poly_order - order - 1);
  }
  return minsnap_yaw;
}

Eigen::Matrix3d getR_N_W(double time_in_trajectory, int poly_order, int coeff_segment_base)
{
  double minsnap_yaw = getMinSnapYaw(time_in_trajectory, poly_order, coeff_segment_base);
  Eigen::Vector3d minsnap_acc;
  minsnap_acc = getMinSnapAcc(time_in_trajectory, poly_order, coeff_segment_base);
  // calculate frame N from acceration
  Eigen::Vector3d acc_W, z_N_W, x_Y_W, y_N_W, x_N_W, Theta_N;
  acc_W = minsnap_acc;
  acc_W(2) -= 9.8;
  //cerr<<acc_W.norm()<<endl;
  z_N_W = -acc_W/acc_W.norm();
  x_Y_W << cos(minsnap_yaw), sin(minsnap_yaw), 0;
  y_N_W = z_N_W.cross(x_Y_W);
  y_N_W.normalize();
  x_N_W = y_N_W.cross(z_N_W);
  Eigen::Matrix3d R_N_W;
  x_N_W.normalize();
  y_N_W.normalize();
  z_N_W.normalize();
  R_N_W.col(0) = x_N_W;
  R_N_W.col(1) = y_N_W;
  R_N_W.col(2) = z_N_W;
  return R_N_W;
}

Eigen::Vector3d getTheta_N(double time_in_trajectory, int poly_order, int coeff_segment_base)
{
  Eigen::Vector3d Theta_N;
  Eigen::Matrix3d R_N_W = getR_N_W(time_in_trajectory, poly_order, coeff_segment_base);
  Theta_N = R_N_W.eulerAngles(0, 1, 2);
  return Theta_N;
}

Eigen::Vector3d getOmega_N(double time_in_trajectory, int poly_order, int coeff_segment_base)
{
  // calculate euler angle of frame N
  Eigen::Vector3d Theta_N, Theta_N_plus, Theta_N_minus, Theta_N_dot, omega_N;
  double time_infinitesimal = 1e-05;
  Theta_N = getTheta_N(time_in_trajectory, poly_order, coeff_segment_base);
  Theta_N_plus  = getTheta_N(time_in_trajectory+time_infinitesimal, poly_order, coeff_segment_base);
  Theta_N_minus = getTheta_N(time_in_trajectory-time_infinitesimal, poly_order, coeff_segment_base);
  // calculate angular rate and acceleration in N
  Theta_N_dot = (Theta_N_plus - Theta_N_minus) / 2.0 / time_infinitesimal;
  Eigen::Matrix3d V_Theta_inv;
  V_Theta_inv << 1,   0,                  -sin(Theta_N(1)),
                 0,   cos(Theta_N(2)),    sin(Theta_N(2))*cos(Theta_N(1)),
                 0,   -sin(Theta_N(2)),   cos(Theta_N(2))*cos(Theta_N(1));
  omega_N = V_Theta_inv * Theta_N_dot;
  return omega_N;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Trajectory_generate");

  ros::NodeHandle n("~");
  ros::NodeHandle nh;
  ros::Publisher Trajectory_pub          = nh.advertise<geometry_msgs::PoseStamped>("goal", 1);
  ros::Publisher Trajectory_velocity_pub = nh.advertise<geometry_msgs::Vector3>("goal_velocity", 1);
  ros::Publisher Trajectory_euler_pub    = nh.advertise<geometry_msgs::Vector3>("goal_angle", 1);
  ros::Publisher Test_data_pub           = nh.advertise<geometry_msgs::Vector3>("trajectory_generate_test_data", 1);
  ros::Subscriber CurrPose_Sub           = nh.subscribe("pose", 5, Subscribe_CurrPose);
  ros::Subscriber PolyCoeff_Sub          = nh.subscribe("/trajectory_generator_node/poly_coeff", 1, Subscribe_PolyCoeff);
  ros::Subscriber TimeAlloc_Sub          = nh.subscribe("/trajectory_generator_node/time_alloc", 1, Subscribe_TimeAlloc);

  n.getParam("x", param_x);
  n.getParam("y", param_y);
  n.getParam("z", param_z);
  n.getParam("yaw", param_yaw);
  n.getParam("rate", param_rate);
  n.getParam("radius", param_radius);
  n.getParam("enable_tracking", param_enable_tracking);
  n.getParam("round_period", param_round_period);
  n.getParam("trajectory_style", param_trajectory_style);

  if(param_trajectory_style == TRAJEC_MINSNAP)
    {
      n.getParam("vehicle_weight", param_vehicle_weight);
      n.getParam("vehicle_Jyy", param_vehicle_Jyy);
      n.getParam("vehicle_arm_length", param_vehicle_arm_length);
      ROS_INFO("Vehicle weight: %f", param_vehicle_weight);
      ROS_INFO("Vehicle arm length: %f", param_vehicle_arm_length);
      ROS_INFO("Vehicle Jyy: %f", param_vehicle_Jyy);
    }

  string CHIRP_PATH = "/home/USERNAME/catkin_ws/src/trajectory_generate/data/chirp.txt";
  string FF_PATH = "/home/USERNAME/catkin_ws/src/trajectory_generate/data/FF_Trajectory.txt";

  char* username;
  username = (char *)malloc(4*sizeof(char));
  cuserid(username);
  string USERNAME(username);

  CHIRP_PATH.replace(6,8,USERNAME);
  FF_PATH.replace(6,8,USERNAME);
  
  ifstream in(CHIRP_PATH);
  ifstream in_FF(FF_PATH);

  tf::TransformListener listener;

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

  // for min snap only
  current_seg_num = 0;

  
  double devide_variable = param_round_period * param_rate / 2.0 / pi;

  cout<<devide_variable<<endl;

  int count = 0;
  double start_time = ros::Time::now().toSec();
  double current_time = start_time;

  tf::Quaternion quaternion;
  geometry_msgs::PoseStamped msg;
  geometry_msgs::Vector3     msg_velocity;
  geometry_msgs::Vector3     msg_euler;
  geometry_msgs::Vector3     msg_test_data;
  
  while (ros::ok())
    {
      msg.header.seq      += 1;
      msg.header.stamp    = ros::Time::now();
      current_time = ros::Time::now().toSec() - start_time;

      tf::StampedTransform transform;
      try{
	listener.lookupTransform("origin", "quad", ros::Time(0), transform);
      }catch(tf::TransformException &ex){
	// Warn in AUTO mode without feedback
	ROS_WARN_THROTTLE(5,"Trajectory: Origin or Quad not ready! Check vrpn...");
      }
      
      if(ready_to_track == false)
	{
#ifdef DEBUG_TRAJECTORY	  
	  // FIXME for debugging
	  if(count > 5 * param_rate){
	    ROS_INFO("Start trajectory");
	    ready_to_track = true;
	    count = 0;
	    start_time = ros::Time::now().toSec();
	    current_time = ros::Time::now().toSec() - start_time;
	  }
#endif
	  if(transform.getOrigin().z() > 0.5 * param_z && count > 15 * param_rate){
	    ready_to_track = true;
	    ROS_INFO("\nReach height. Start tracking trajectory\n");
	    count = 0;
	    start_time = ros::Time::now().toSec();
	    current_time = ros::Time::now().toSec() - start_time;
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
	  msg_euler.x = 0;
	  msg_euler.y = 0;
	  msg_euler.z = param_yaw;
	  Trajectory_pub.publish(msg);
	  Trajectory_euler_pub.publish(msg_euler);
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
      }else if(ready_to_track == true && param_trajectory_style == TRAJEC_MINSNAP){
	Eigen::Vector3d goalAngle;
	goalAngle<<0, 0, 0;
	double minsnap_yaw=0, minsnap_yaw_rate=0;
	msg_euler.x = 0;
	msg_euler.y = 0;
	msg_euler.z = param_yaw;
        if(poly_coeff_ready && poly_time_ready){
	  int segment_num = poly_coeff.layout.dim[0].size;
	  int poly_order = poly_coeff.layout.dim[1].size / 4;
	  Eigen::Vector3d minsnap_pose, minsnap_acc;
	  // use to record the sum of time in the past segment
	  double past_seg_time = 0;
	  for(int i = 0; i < current_seg_num; i++){
	    past_seg_time += time_alloc.data[i];
	  }
	  double time_in_trajectory = current_time - past_seg_time;
	  if(current_seg_num < segment_num){
	    //cerr<<"time: "<<time_alloc.data[current_seg_num]<<endl;
	    if(time_in_trajectory >= time_alloc.data[current_seg_num]){
	      current_seg_num += 1;
	      time_in_trajectory -= time_alloc.data[current_seg_num-1];
	    }
	    int coeff_segment_base = current_seg_num*poly_order*4;
	    // calculate pose
	    if(current_seg_num < segment_num){
	      minsnap_pose = getMinSnapPose(time_in_trajectory, poly_order, coeff_segment_base);
	      msg.pose.position.x = minsnap_pose(0);
	      msg.pose.position.y = minsnap_pose(1);
	      msg.pose.position.z = minsnap_pose(2);
	      // calculate force
	      Eigen::Vector3d minsnap_acc;
	      double Tnorm;
	      minsnap_acc = getMinSnapAcc(time_in_trajectory, poly_order, coeff_segment_base);
	      minsnap_acc(2) -= 9.8;
	      Tnorm = minsnap_acc.norm() * param_vehicle_weight;
	      // calculate angular acceleration
	      double time_infinitesimal = 1e-05;
	      Eigen::Vector3d omega_N_dot, omega_N_plus, omega_N_minus;
	      omega_N_plus = getOmega_N(time_in_trajectory+time_infinitesimal, poly_order, coeff_segment_base);
	      omega_N_minus = getOmega_N(time_in_trajectory-time_infinitesimal, poly_order, coeff_segment_base);
	      omega_N_dot = (omega_N_plus - omega_N_minus) / 2.0 / time_infinitesimal;
	      // calculate angle alpha
	      double alpha = asin(saturate(param_vehicle_Jyy * omega_N_dot(1) / param_vehicle_arm_length / Tnorm, -1, 1));
	      // calculate R_B_W and its euler angle
	      Eigen::Matrix3d R_B_N(Eigen::AngleAxisd(alpha, Eigen::Vector3d::UnitY()));
	      Eigen::Matrix3d R_N_W = getR_N_W(time_in_trajectory, poly_order, coeff_segment_base);
	      Eigen::Matrix3d R_B_W = R_N_W * R_B_N;

	      // calculate yaw separately
	      minsnap_yaw = getMinSnapYaw(time_in_trajectory, poly_order, coeff_segment_base);
	      minsnap_yaw_rate = getMinSnapYawRate(time_in_trajectory, poly_order, coeff_segment_base);

	      // use eigen method to calculate euler
	      //Eigen::Quaterniond q_B_W(R_B_W);
	      //Eigen::Vector3d eulerAngle = R_B_W.eulerAngles(2, 1, 0);
	      // use tf method to calculate temp
	      
	      tfScalar euler_yaw, euler_pitch, euler_roll;
	      tf::Matrix3x3 R_B_W_TF;
	      tf::matrixEigenToTF(R_B_W, R_B_W_TF);
	      R_B_W_TF.getEulerYPR(euler_yaw, euler_pitch, euler_roll);
	      Eigen::Vector3d eulerAngle;
	      eulerAngle << minsnap_yaw_rate, euler_pitch, euler_roll;
	      

	      quaternion.setRPY(eulerAngle(2), eulerAngle(1), eulerAngle(0));

	      // use eigen method to calculate temp
	      //Eigen::Vector3d tempAngle = R_N_W.eulerAngles(2, 1, 0);
	      // use tf method to calculate temp
	      
	      tfScalar temp_yaw, temp_pitch, temp_roll;
	      tf::Matrix3x3 R_N_W_TF;
	      tf::matrixEigenToTF(R_N_W, R_N_W_TF);
	      R_N_W_TF.getEulerYPR(temp_yaw, temp_pitch, temp_roll);
	      Eigen::Vector3d tempAngle;
	      tempAngle << minsnap_yaw_rate, temp_pitch, temp_roll;

	      msg_euler.x = tempAngle(2);
	      msg_euler.y = eulerAngle(1);
	      msg_euler.z = tempAngle(0);
	      goalAngle = tempAngle;
	      msg_test_data.x = alpha;
	      Test_data_pub.publish(msg_test_data);
	      //cerr<<"RNW:"<<endl<<R_N_W<<endl;
	      //cerr<<"RBN:"<<endl<<R_B_N<<endl;
	      //cerr<<"Euler angle: "<<endl<<eulerAngle<<endl;
	      //cerr<<"Temp angle: "<<endl<<tempAngle<<endl;
	      //cerr<<alpha<<",  "<<omega_N_dot(1)<<", "<<Tnorm<<endl<<endl<<endl;
	    }
	  }
	  // set 0 RPY here to get the transpose correctly
	  quaternion.setRPY(goalAngle(2), goalAngle(1), minsnap_yaw);
	  msg.pose.orientation.x = quaternion.x();
	  msg.pose.orientation.y = quaternion.y();
	  msg.pose.orientation.z = quaternion.z();
	  msg.pose.orientation.w = quaternion.w();
	  Trajectory_pub.publish(msg);
	  Trajectory_euler_pub.publish(msg_euler);
	}
      }
      ros::spinOnce();

      loop_rate.sleep();

      ++count;
    }

  return 0;
}
