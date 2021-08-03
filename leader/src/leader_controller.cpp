#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_datatypes.h>
#include <qptrajectory.h>
#include <cstdio>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <tf2/transform_datatypes.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Path.h>

#define PAYLOAD_LENGTH 1.59
#define normal
#define PI 3.1415926

double k1 = 3.0, k2 = 0.1, k3 = 1.0, k4 = 0.3, kv = 3.0, kw = 30.0;
double mp = 0.5,  g = 9.8, Izz = mp * PAYLOAD_LENGTH * PAYLOAD_LENGTH / 12;
double x_upper = 1.95;
double y_upper = 6.0;
double controller_body_x, controller_body_y;
Eigen::Vector3d pose, vel;
Eigen::Vector3d v_p;
Eigen::Vector3d r_p_c2(-0.5 * PAYLOAD_LENGTH, 0, 0);
double vir_x, vir_y, theta_r, vx, vy, ax, ay, jx, jy;
double eta_1, eta_2;
double last_w = 0.0;

float payload_yaw;
// double payload_roll, payload_yaw, payload_pitch;

Eigen::Vector3d v_w_eta;
Eigen::Vector3d pc2_est;

double x_e, y_e, theta_e;
Eigen::Vector3d err_state;
unsigned int tick = 0;
bool flag = false;

Eigen::Matrix3d R_pl_B;
Eigen::Vector3d w_;

geometry_msgs::PoseStamped desired_pose;
geometry_msgs::Point controller_force;

float nonlinear, vd_dot_debug;
geometry_msgs::Point debug_msg;
geometry_msgs::Point path_plot;

sensor_msgs::Imu imu_data;
void payload_imu_callback(const sensor_msgs::Imu::ConstPtr& msg){
  imu_data = *msg;

  // double w,x,y,z;
  // x = imu_data.orientation.x;
  // y = imu_data.orientation.y;
  // z = imu_data.orientation.z;
  // w = imu_data.orientation.w;
  // tf::Quaternion Q(x, y, z, w);
  // tf::Matrix3x3(Q).getRPY(payload_roll, payload_pitch, payload_yaw);

  R_pl_B << cos(payload_yaw), sin(payload_yaw),   0,
           -sin(payload_yaw), cos(payload_yaw),   0,
                           0,                0,   1;

  Eigen::Vector3d tmp;
  tmp << imu_data.angular_velocity.x,
         imu_data.angular_velocity.y,
         imu_data.angular_velocity.z;

  w_ << 0, 0, tmp(2);
}

void est_vel_cb(const geometry_msgs::Point::ConstPtr& msg){
  Eigen::Vector3d vc1 = Eigen::Vector3d(msg->x, msg->y, msg->z);
  v_p = R_pl_B*vc1;
}

void pc2_cb(const geometry_msgs::Point::ConstPtr& msg){
  pc2_est << msg->x, msg->y, msg->z;
}

void eta_cb(const geometry_msgs::Point::ConstPtr& msg){
  v_w_eta << msg->x, msg->y, msg->z;
}

void optitrack_payload_yaw_callback(const geometry_msgs::Pose2D::ConstPtr& msg){
  geometry_msgs::Pose2D optitrack_payload_data;
  optitrack_payload_data = *msg;
  payload_yaw = optitrack_payload_data.theta;
}

Eigen::Vector3d nonholonomic_output(double x_r, double y_r, double theta_r, double v_r, double w_r){

  Eigen::Vector3d output;
  Eigen::Vector3d err_state_B;
  err_state << x_r - pc2_est(0), y_r - pc2_est(1), theta_r - payload_yaw; // +(PI/2);(5)(6)
  err_state_B = R_pl_B * err_state;

  x_e = err_state_B(0);
  y_e = err_state_B(1);
  theta_e = err_state(2);

  double vd = v_r*cos(theta_e) + k1*x_e;   //(43) k1
  double w_d = w_r + v_r*k2*y_e + k3*sin(theta_e);    //k2 k3

  output << vd, w_d, 0;
  return output;
}

int main(int argc, char **argv){

  ros::init(argc, argv, "leader_controller");
  ros::NodeHandle nh;

  ros::Subscriber imu1_sub = nh.subscribe("/mavros/imu/data",2,payload_imu_callback);
  // ros::Subscriber imu1_sub = nh.subscribe("/payload/IMU1",2,payload_imu_callback);
  ros::Subscriber est_vel_sub = nh.subscribe<geometry_msgs::Point>("est_vel",3,est_vel_cb);
  ros::Subscriber pc2_sub = nh.subscribe("pointpc2",2,pc2_cb);
  ros::Subscriber eta_sub = nh.subscribe("pointvc2",2,eta_cb);
  ros::Subscriber yaw_sub = nh.subscribe("optitrack_payload_yaw",2,optitrack_payload_yaw_callback);
  // ros::Subscriber yaw_sub = nh.subscribe("/fucking_yaw",2,optitrack_payload_yaw_callback);

  ros::Publisher controller_force_pub = nh.advertise<geometry_msgs::Point>("/controller_force",2);
  ros::Publisher debug_pub = nh.advertise<geometry_msgs::Point>("/debug_msg",2);
  ros::Publisher my_path_pub = nh.advertise<geometry_msgs::Point>("/path_plot",2);
  ros::Publisher ycc_path_pub = nh.advertise<nav_msgs::Path>("trajectory", 1, true);

  ros::Rate loop_rate(20.0);
  nh.setParam("/start",false);
  // geometry_msgs::PoseStamped force;

  // YCC QP path plot
  ros::Time current_time;//, last_time;
  current_time = ros::Time::now();
  // last_time = ros::Time::now();
  nav_msgs::Path ycc_path;
  ycc_path.header.stamp = current_time;
  ycc_path.header.frame_id = "map";

  //planning
  qptrajectory plan;
  path_def path;
  trajectory_profile p1,p2,p3,p4,p5,p6,p7,p8;
  std::vector<trajectory_profile> data;
#if 0
    p1.pos << -1.2,0.9,0;
    p1.vel << 0,0,0;
    p1.acc << 0,0,0;
    p1.yaw = 0;

    p2.pos << -0.3,0.9,0;
    p2.vel << 0,0,0;
    p2.acc << 0,0,0;
    p2.yaw = 0;

    p3.pos << -0.4,0.45,0;
    p3.vel << 0,0,0;
    p3.acc << 0,0,0;
    p3.yaw = 0;

    p4.pos << -0.6,0.45,0;
    p4.vel << 0,0,0;
    p4.acc << 0,0,0;
    p4.yaw = 0;

    p5.pos << -0.3,0.55,0;
    p5.vel << 0,0,0;
    p5.acc << 0,0,0;
    p5.yaw = 0;
#endif
	p1.pos << -1.2,  0.9, 0.0;
	p1.vel << 0.0,  0.0, 0.0;
	p1.acc << 0.0, -0.0, 0.0;

	p2.pos << -0.4,  0.9, 0.0;
	p2.vel << 0.0,  0.0, 0.0;
	p2.acc << 0.0, -0.0, 0.0;

	p3.pos << -0.15, 0.45, 0.0;
	p3.vel << 0.0, 0.0, 0.0;
	p3.acc << 0.0, 0.0, 0.0;

	p4.pos << -0.35, 0.45, 0.0;
	p4.vel <<  0.0, 0.0, 0.0;
	p4.acc <<  0.0, 0.0, 0.0;
#if 0
	p5.pos << -0.35, 0.4, 0.0;
	p5.vel << 0.0, 0.0, 0.0;
	p5.acc << 0.0, 0.0, 0.0;

	p6.pos << -0.25, 0.4, 0.0;
	p6.vel << 0.0,  0.0, 0.0;
	p6.acc << 0.0,  0.0, 0.0;
#endif
  path.push_back(segments(p1,p2,7));
  path.push_back(segments(p2,p3,5));
  path.push_back(segments(p3,p4,3));
  //path.push_back(segments(p4,p5,2));
  //path.push_back(segments(p5,p6,3));
  // path.push_back(segments(p6,p7,6.0));
  // path.push_back(segments(p7,p8,6.0));
  data = plan.get_profile(path,path.size(),0.05);

  desired_pose.pose.position.x = -0.6;
  desired_pose.pose.position.y = 0.6;
  desired_pose.pose.position.z = 0.6;


  while(ros::ok()){

    nh.getParam("/start",flag);

    if(flag == false || ( tick > data.size() )){
      //do position control
      nh.setParam("/start",false);
      tick = 0;

      // force.pose.position.x = 3*(desired_pose.pose.position.x - pose(0)) + 1*(0 - vel(0));
      // force.pose.position.y = 3*(desired_pose.pose.position.y - pose(1)) + 1*(0 - vel(1));
      // force.pose.position.z = 3*(desired_pose.pose.position.z - pose(2)) + 1*(0 - vel(2)) + mp*g/2.0;
      controller_force.x = 0;
      controller_force.y = 0;
    }
    else
    {
      vir_x = data[tick].pos(0);
      vir_y = data[tick].pos(1);
      vx = data[tick].vel(0);
      vy = data[tick].vel(1);
      ax = data[tick].acc(0);
      ay = data[tick].acc(1);
      jx = data[tick].jerk(0);
      jy = data[tick].jerk(1);

      path_plot.x = vir_x;
      path_plot.y = vir_y;

      // YCC QP path plot
      geometry_msgs::PoseStamped this_pose_stamped;
      this_pose_stamped.pose.position.x = vir_x;
      this_pose_stamped.pose.position.y = vir_y;
      this_pose_stamped.pose.position.z = 2.5;

      this_pose_stamped.header.stamp = current_time;
      this_pose_stamped.header.frame_id = "map";
      ycc_path.poses.push_back(this_pose_stamped);

      theta_r = atan2(data[tick].vel(1),data[tick].vel(0));   //(4)

      if(theta_r <0){
        theta_r += 2*PI;
      }

      Eigen::Vector3d alpha;
      alpha << 0, 0, (w_(2) - last_w)/0.02;
      last_w = w_(2); //payload imu

      double w_r = (ay*vx - ax*vy)/(vx*vx + vy*vy); //(theta_r - last_theta_r) /(0.02) ;
      double vr = sqrt(vx*vx + vy*vy);

      Eigen::Vector3d nonholoutput = nonholonomic_output(vir_x, vir_y, theta_r, vr, w_r);   //vd, w_d, 0
      double vr_dot = sqrt(ax*ax + ay*ay);
      double theta_e_dot = w_r - w_(2);  //the error of the angular velocity
      double x_e_dot = w_(2) * y_e + vr*cos(theta_e) - v_w_eta(0);  //(58)
                                                      //UKF 2nd
      double y_e_dot = - w_(2) * x_e + vr*sin(theta_e);  //(58)
      double w_r_dot = (jy*vx - jx*vy)/(vr*vr) - (2*vr_dot*w_r)/vr;     //vr^(-3) ??
      double w_d_dot = w_r_dot + vr_dot*k2*y_e + vr*k2*y_e_dot + k3*theta_e_dot*cos(theta_e);   //take (43) time derivative
      double vd_dot = vr_dot*cos(theta_e) - vr*theta_e_dot*sin(theta_e) + k1*x_e_dot;

      Eigen::Vector3d nonlinearterm;

      nonlinearterm = w_.cross(v_p) - alpha.cross(r_p_c2) - w_.cross(w_.cross(r_p_c2)); //the last term of (41)

      if( nonholoutput(0) > 10 ){   //vd
        nonholoutput(0) = 10;
      }

      Eigen::Vector3d tmp;
      Eigen::Vector3d cmd_;

      eta_1 = (nonholoutput(0) - v_w_eta(0));
      eta_2 = (nonholoutput(1) - v_w_eta(2));


      //(41)(42) separately
      tmp << kv * eta_1 + x_e + nonlinearterm(0) + vd_dot,
             kw * eta_2 + sin(theta_e)/k2 + k4 * w_d_dot,   //ffy is close to zero.
             0;

      if(tmp(0) > x_upper){
        tmp(0) = x_upper;
      }
      if(tmp(1) > y_upper){
	tmp(1) = y_upper;
      }else if(tmp(1) < -y_upper){
	tmp(1) = -y_upper;
      }
      controller_body_x = tmp(0);
      controller_body_y = tmp(1);

      Eigen::Matrix3d M;
      M <<   mp,                     0,    0,
              0,  2*Izz/PAYLOAD_LENGTH,    0,
              0,                     0,    1;

      cmd_ = R_pl_B.transpose() * M * tmp;

      tick++;


      controller_force.x = cmd_(0);   // + nonlinearterm(0);// + vd_dot ;
      controller_force.y = cmd_(1);   // bias	// + w_d_dot;

      debug_msg.x = eta_1;
      debug_msg.y = nonholoutput(1) - v_w_eta(2);
      debug_msg.z = w_d_dot;
      nonlinear = nonlinearterm(0);
      vd_dot_debug = vd_dot;

    }

    controller_force_pub.publish(controller_force);
    debug_pub.publish(debug_msg);
    my_path_pub.publish(path_plot);

    ycc_path_pub.publish(ycc_path);

    // std::cout << "payload_yaw " << payload_yaw << std::endl;
    printf("controller force x: %f, y: %f\t", controller_body_x, controller_body_y);
    printf("vd_dot: %f, x_e: %f, eta x: %f, nonlinear: %f\t", vd_dot_debug, x_e, debug_msg.x, nonlinear);
    //printf("eta y: %f, w_d_dot: %f, sin(theta e): %f", debug_msg.y, debug_msg.z, sin(theta_e));
    printf("\n");


    ros::spinOnce();
    loop_rate.sleep();
  }
}
