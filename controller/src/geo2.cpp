#include <ros/ros.h>
#include <geometric_controller.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/LinkStates.h>
#include <tf/transform_datatypes.h>
#include <qptrajectory.h>
#include <cstdio>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Point.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <mav_msgs/Actuators.h>
#include <geometric_controller.h>
#include <queue>
#include <nav_msgs/Odometry.h>

#define length 0.18
#define PI 3.1415926
float mp = 0.5, g = 9.8, kd = mp*g/(2*length), bd = 2.0, mF = 1.0, Md = 1.5, kf1 = 2.0;
float dt = 0.02;

geometry_msgs::PoseStamped desired_pose;
Eigen::Vector3d pose, vel;

bool flag = false;
bool force_control = false;

Eigen::Vector3d p_c2;
Eigen::Matrix3d payload_Rotation;
Eigen::Matrix3d uav_rotation;

geometry_msgs::PoseStamped force;
geometry_msgs::Point est_force;

Eigen::Matrix3d payload_rotation_truth;
bool triggered;
float payload_yaw;

Eigen::VectorXd poly_t(float t){
  Eigen::VectorXd data;
  data.resize(6);
  data << 1, t, t*t, t*t*t, t*t*t*t, t*t*t*t*t;
  return data;
}

Eigen::Vector3d p_F_c2;
Eigen::Vector3d FF_I, FF_B, vb;
int count_ = 0;
std::queue<float> pos_x_buffer;
std::queue<float> pos_y_buffer;
Eigen::VectorXd coeff_x, coeff_y;

float last_time = 0;
float tmpx = 0, tmpy = 0;
float last_tmp_x = 0, last_tmp_y = 0;
float r, Fn, an;
float command;
float last_vec_x = 0, last_vec_y = 0, last_vx = 0, last_vy = 0;

void est_force_cb(const geometry_msgs::Point::ConstPtr& msg){
  est_force = *msg;
  FF_I << est_force.x, est_force.y, est_force.z;

  p_F_c2 = pose - uav_rotation * Eigen::Vector3d(0, 0, 0.05);  // offset x from uav to connector
  p_c2 = p_F_c2 + (FF_I/FF_I.norm()) * length;

  payload_yaw = atan2( tmpy - last_tmp_y, tmpx - last_tmp_x);

  if(payload_yaw < 0){
    payload_yaw += 2*PI;
  }

  payload_Rotation << cos(payload_yaw),  -sin(payload_yaw),   0,
                      sin(payload_yaw),   cos(payload_yaw),   0,
                                     0,                  0,   1;

  //change the force from inertial frame to body.
  if((count_ %1 == 0) && (pos_x_buffer.size()>7)){
    pos_x_buffer.pop();
    pos_y_buffer.pop();
    pos_x_buffer.push(p_c2(0));
    pos_y_buffer.push(p_c2(1));
    Eigen::VectorXd tx, ty;
    Eigen::MatrixXd w;
    tx.resize(7);
    ty.resize(7);
    w.resize(7,6);
    w << poly_t(0).transpose(),
         poly_t(0.02).transpose(),
         poly_t(0.04).transpose(),
         poly_t(0.06).transpose(),
         poly_t(0.08).transpose(),
         poly_t(0.10).transpose(),
         poly_t(0.12).transpose();

    float t1,t2,t3,t4,t5,t6,t7;

    t1 = pos_x_buffer.front();
    pos_x_buffer.pop();
    t2 = pos_x_buffer.front();
    pos_x_buffer.pop();
    t3 = pos_x_buffer.front();
    pos_x_buffer.pop();
    t4 = pos_x_buffer.front();
    pos_x_buffer.pop();
    t5 = pos_x_buffer.front();
    pos_x_buffer.pop();
    t6 = pos_x_buffer.front();
    pos_x_buffer.pop();
    t7 = pos_x_buffer.front();
    pos_x_buffer.pop();

    tx << t1, t2, t3, t4, t5, t6, t7;
    coeff_x = (w.transpose() * w).inverse() * w.transpose()*tx;

    t1 = pos_y_buffer.front();
    pos_y_buffer.pop();
    t2 = pos_y_buffer.front();
    pos_y_buffer.pop();
    t3 = pos_y_buffer.front();
    pos_y_buffer.pop();
    t4 = pos_y_buffer.front();
    pos_y_buffer.pop();
    t5 = pos_y_buffer.front();
    pos_y_buffer.pop();
    t6 = pos_y_buffer.front();
    pos_y_buffer.pop();
    t7 = pos_y_buffer.front();
    pos_y_buffer.pop();

    ty << t1, t2, t3, t4, t5, t6, t7;
    coeff_y = (w.transpose() * w).inverse() * w.transpose()* ty;

    float t = 0.14;
    tmpx = coeff_x(0)*1 + coeff_x(1)*t + coeff_x(2)*t*t + coeff_x(3)*t*t*t + coeff_x(4)*t*t*t*t + coeff_x(5)*t*t*t*t*t;
    tmpy = coeff_y(0)*1 + coeff_y(1)*t + coeff_y(2)*t*t + coeff_y(3)*t*t*t + coeff_y(4)*t*t*t*t + coeff_y(5)*t*t*t*t*t;
    tmpx = 0.7*last_tmp_x + 0.3*tmpx;    // filter
    tmpy = 0.7*last_tmp_y + 0.3*tmpy;

    float dt = ros::Time::now().toSec() - last_time;
    last_time = ros::Time::now().toSec();

    float vec_x = tmpx - last_tmp_x;
    float vec_y = tmpy - last_tmp_y;
    float vx = vec_x /dt;
    float vy = vec_y /dt;
    float ax = (vx - last_vx)/dt;
    float ay = (vy - last_vy)/dt;
    float v = sqrt(vx*vx + vy*vy);

    Eigen::Vector3d offset_b = payload_Rotation*(FF_I/FF_I.norm()) * length;

    r = (v*v*v)/fabs(vx*ay - vy*ax);
    an = (v*v)/r;
    Fn = mp/2 * an;

    //float fy = FF_B(1);
    float dy = -offset_b(1);

    if((atan2(vy,vx) - atan2(last_vy,last_vx))<0){
      //determine wheather the motion is CCW or CW, if CW Fn and dy is negative.
      Fn = Fn * -1.0;
    }

    command = mF*(bd*-vb(1) - kd*dy)/Md + Fn;

    last_tmp_x = tmpx;
    last_tmp_y = tmpy;
    last_vec_x = vec_x;
    last_vec_y = vec_y;
    last_vx = vx;
    last_vy = vy;
  }
  else if (count_ %1 == 0){
    pos_x_buffer.push(p_c2(0));
    pos_y_buffer.push(p_c2(1));
  }
  count_++;
}

geometry_msgs::PoseStamped optitrack_data, last_pose;
void optitrack_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
  optitrack_data = *msg;

  pose << optitrack_data.pose.position.x, optitrack_data.pose.position.y, optitrack_data.pose.position.z;

  vel(0) = (optitrack_data.pose.position.x - last_pose.pose.position.x)/dt;
  vel(1) = (optitrack_data.pose.position.y - last_pose.pose.position.y)/dt;
  vel(2) = (optitrack_data.pose.position.z - last_pose.pose.position.z)/dt;

  float w,x,y,z;
  x = optitrack_data.pose.orientation.x;
  y = optitrack_data.pose.orientation.y;
  z = optitrack_data.pose.orientation.z;
  w = optitrack_data.pose.orientation.w;

  uav_rotation << w*w+x*x-y*y-z*z,      2*x*y-2*w*z,     2*x*z+2*w*y,
                     2*x*y +2*w*z,  w*w-x*x+y*y-z*z,     2*y*z-2*w*x,
                     2*x*z -2*w*y,      2*y*z+2*w*x, w*w-x*x-y*y+z*z;

  last_pose.pose = optitrack_data.pose;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "geo2");
  ros::NodeHandle nh;

  ros::Subscriber odom_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/RigidBody7/pose",3, optitrack_cb);
  ros::Subscriber est_force_sub = nh.subscribe<geometry_msgs::Point>("/follower_ukf/force_estimate",3, est_force_cb);

  ros::Publisher traj_pub = nh.advertise<geometry_msgs::PoseStamped>("/firefly2/command/pose",2);

  nh.setParam("/start2",false);
  nh.setParam("/force_control",false);

  triggered = false;
  ros::Rate loop_rate(20.0);

  desired_pose.pose.position.x = -0.3;
  desired_pose.pose.position.y = 0.0;
  desired_pose.pose.position.z = 1.3;

  while(ros::ok()){
    nh.getParam("/start2",flag);
    nh.getParam("/force_control",force_control);

    float ft = sqrt(FF_I(0)*FF_I(0) + FF_I(1)*FF_I(1));
    if((ft>0.3)){
      triggered = true;
    }
    else if((triggered)&&((ft<0.3)&&(ft>0.2))){
      triggered = true;
    }
    else{
      triggered = false;
    }

    if((triggered) && (force_control)){
      Eigen::Vector3d a, ab;

      FF_B = payload_Rotation * FF_I;
      vb = payload_Rotation * vel;

      ab <<                              kf1*-vb(0) + FF_B(0),
                                               command,
            5.0*(desired_pose.pose.position.z - pose(2)) + 2.0*(0 - vel(2)) + 0.5*mp*g;
      a = payload_Rotation.transpose()*ab;

      desired_pose.pose.position.x = pose(0);
      desired_pose.pose.position.y = pose(1);
      force.pose.position.x = a(0);
      force.pose.position.y = a(1);
      force.pose.position.z = a(2);
    }
    else{
      force.pose.position.x = 3*(desired_pose.pose.position.x - pose(0)) + 1*(0 - vel(0));
      force.pose.position.y = 3*(desired_pose.pose.position.y - pose(1)) + 1*(0 - vel(1));
      force.pose.position.z = 3*(desired_pose.pose.position.z - pose(2)) + 1*(0 - vel(2)) + 0.5*mp*g;
    }

    traj_pub.publish(force);

    ros::spinOnce();
    loop_rate.sleep();
  }
}
