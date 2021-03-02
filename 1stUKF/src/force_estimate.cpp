#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "force_est.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_datatypes.h>
#include "geometry_msgs/Point.h"
#include "math.h"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "geometry_msgs/TwistStamped.h"
#include <string>
#include <gazebo_msgs/ModelStates.h>
#include "geometry_msgs/WrenchStamped.h"
#include <random>

#define l 0.25
#define k 0.02
std::string model_name;
int drone_flag;
forceest forceest1(statesize,measurementsize);
geometry_msgs::Point euler, euler_ref, force, torque, bias, angular_v, pose;
sensor_msgs::Imu drone_imu;
geometry_msgs::PoseStamped optitrack_data, drone_pose, last_pose;
geometry_msgs::TwistStamped drone_vel;
float dt = 0.02;

void imu_cb(const sensor_msgs::Imu::ConstPtr &msg){
  drone_imu = *msg;
}

void optitrack_cb(const geometry_msgs::PoseStamped::ConstPtr &msg){
  optitrack_data = *msg;

  drone_pose.pose = optitrack_data.pose;

  drone_vel.twist.linear.x = (drone_pose.pose.position.x - last_pose.pose.position.x)/dt;
  drone_vel.twist.linear.y = (drone_pose.pose.position.y - last_pose.pose.position.y)/dt;
  drone_vel.twist.linear.z = (drone_pose.pose.position.z - last_pose.pose.position.z)/dt;
  last_pose.pose = optitrack_data.pose;
}

Eigen::Vector3d f1, f2, f3, f4, f5, f6;
void f1_cb(const geometry_msgs::WrenchStamped::ConstPtr& msg){
  f1 << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z;
}

void f2_cb(const geometry_msgs::WrenchStamped::ConstPtr& msg){
  f2 << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z;
}

void f3_cb(const geometry_msgs::WrenchStamped::ConstPtr& msg){
  f3 << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z;
}

void f4_cb(const geometry_msgs::WrenchStamped::ConstPtr& msg){
  f4 << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z;
}

void f5_cb(const geometry_msgs::WrenchStamped::ConstPtr& msg){
  f5 << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z;
}

void f6_cb(const geometry_msgs::WrenchStamped::ConstPtr& msg){
  f6 << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "force_estimate");
  ros::NodeHandle nh;

  std::string topic_imu, topic_mocap;
  ros::param::get("~topic_imu", topic_imu);
  ros::param::get("~topic_mocap", topic_mocap);
  ros::param::get("~topic_drone", drone_flag);

  if(drone_flag == 2){
    model_name = "/firefly2/firefly2";
    std::cout << "drone 2" << std::endl;
  }

  if(drone_flag == 1){
    model_name = "/firefly1/firefly1";
    std::cout << "drone 1" << std::endl;
  }

  ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data",4,imu_cb);
  ros::Subscriber pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/RigidBody7/pose",4,optitrack_cb);

  ros::Subscriber f1_sub = nh.subscribe(model_name+std::string("/rotor_0_ft"),2,f1_cb);
  ros::Subscriber f2_sub = nh.subscribe(model_name+std::string("/rotor_1_ft"),2,f2_cb);
  ros::Subscriber f3_sub = nh.subscribe(model_name+std::string("/rotor_2_ft"),2,f3_cb);
  ros::Subscriber f4_sub = nh.subscribe(model_name+std::string("/rotor_3_ft"),2,f4_cb);
  ros::Subscriber f5_sub = nh.subscribe(model_name+std::string("/rotor_4_ft"),2,f5_cb);
  ros::Subscriber f6_sub = nh.subscribe(model_name+std::string("/rotor_5_ft"),2,f6_cb);

  ros::Publisher euler_pub = nh.advertise<geometry_msgs::Point>("euler",2);
  ros::Publisher euler_ref_pub = nh.advertise<geometry_msgs::Point>("euler_ref",2);
  ros::Publisher force_pub = nh.advertise<geometry_msgs::Point>("force_estimate",2);

  ros::Rate loop_rate(50);

  float measure_ex, measure_ey, measure_ez;

  Eigen::MatrixXd mnoise;
  mnoise.setZero(measurementsize,measurementsize);
  mnoise = 3e-3*Eigen::MatrixXd::Identity(measurementsize,measurementsize);

  mnoise(mp_x,mp_x) = 1e-4;
  mnoise(mp_y,mp_y) = 1e-4;
  mnoise(mp_z,mp_z) = 1e-4;

  mnoise(mv_x,mv_x) = 1e-2;
  mnoise(mv_y,mv_y) = 1e-2;
  mnoise(mv_z,mv_z) = 1e-2;

  mnoise(momega_x,momega_x) = 1e-2;
  mnoise(momega_y,momega_y) = 1e-2;
  mnoise(momega_z,momega_z) = 1e-2;

  mnoise(me_x,me_x) = 1;
  mnoise(me_y,me_y) = 1;
  mnoise(me_z,me_z) = 1;

  forceest1.set_measurement_noise(mnoise);

  Eigen::MatrixXd pnoise;
  pnoise.setZero(statesize,statesize);
  pnoise(p_x,p_x) = 1e-2;
  pnoise(p_y,p_y) = 1e-2;
  pnoise(p_z,p_z) = 1e-2;

  pnoise(v_x,v_x) = 1e-2;
  pnoise(v_y,v_y) = 1e-2;
  pnoise(v_z,v_z) = 1e-2;

  pnoise(e_x,e_x) = 0.005;//0.5,調小beta收斂較快
  pnoise(e_y,e_y) = 0.005;
  pnoise(e_z,e_z) = 0.005;

  pnoise(omega_x,omega_x) = 1e-2;
  pnoise(omega_y,omega_y) = 1e-2;
  pnoise(omega_z,omega_z) = 1e-2;

  pnoise(F_x,F_x) = 1.5;
  pnoise(F_y,F_y) = 1.5;
  pnoise(F_z,F_z) = 1.5;
  pnoise(tau_z,tau_z) = 0.05;

  pnoise(beta_x,beta_x) = 0.05;//調大beta會無法收斂
  pnoise(beta_y,beta_y) = 0.05;
  pnoise(beta_z,beta_z) = 0.05;

  forceest1.set_process_noise(pnoise);

  Eigen::MatrixXd measurement_matrix;
  measurement_matrix.setZero(measurementsize,statesize);

  measurement_matrix(mp_x,p_x) = 1;
  measurement_matrix(mp_y,p_y) = 1;
  measurement_matrix(mp_z,p_z) = 1;

  measurement_matrix(mv_x,v_x) = 1;
  measurement_matrix(mv_y,v_y) = 1;
  measurement_matrix(mv_z,v_z) = 1;

  measurement_matrix(momega_x,omega_x) = 1;
  measurement_matrix(momega_y,omega_y) = 1;
  measurement_matrix(momega_z,omega_z) = 1;

  measurement_matrix(me_x,e_x) = 1;//1,調小，beta會劇烈震盪
  measurement_matrix(me_y,e_y) = 1;
  measurement_matrix(me_z,e_z) = 1;

  forceest1.set_measurement_matrix(measurement_matrix);

  while(ros::ok()){

    float F1, F2, F3, F4;
    float U_x, U_y, U_z;

    const float mean = 0.0;
    const float stddev = 0.1;
    std::default_random_engine generatorx, generatory, generatorz;
    std::normal_distribution<float> distx(mean,stddev);
    std::normal_distribution<float> disty(mean,stddev);
    std::normal_distribution<float> distz(mean,stddev);
    forceest1.gausian_noise << distx(generatorx), disty(generatory), distz(generatorz);

    pose.x = drone_pose.pose.position.x;

    if(drone_imu.angular_velocity.x != 0 && drone_pose.pose.position.x != 0 && drone_vel.twist.linear.x != 0){
      if((drone_flag == 2)||(drone_flag == 1)){

        F1 = f3(2);//(6.13176e-06*(pwm3*pwm3) -0.0181164*pwm3 + 15.9815); //drone
        F2 = f1(2);//(6.13176e-06*(pwm1*pwm1) -0.0181164*pwm1 + 15.9815); //left_right:265.7775
        F3 = f4(2);//(6.13176e-06*(pwm4*pwm4) -0.0181164*pwm4 + 15.9815); //up_down:265.7775
        F4 = f2(2);//(6.13176e-06*(pwm2*pwm2) -0.0181164*pwm2 + 15.9815);
      }

      forceest1.thrust = f1(2) + f2(2) + f3(2) + f4(2) + f5(2) + f6(2);

      U_x = (sqrt(2)/2)*l*(F1 - F2 - F3 + F4);
      U_y = (sqrt(2)/2)*l*(-F1 - F2 + F3 + F4);
      U_z = k*F1 - k*F2 + k*F3 - k*F4;

      forceest1.U << U_x, U_y, U_z;
      float x = drone_pose.pose.orientation.x;
      float y = drone_pose.pose.orientation.y;
      float z = drone_pose.pose.orientation.z;
      float w = drone_pose.pose.orientation.w;

      forceest1.R_IB.setZero();
      forceest1.R_IB << w*w+x*x-y*y-z*z,     2*x*y-2*w*z,     2*x*z+2*w*y,
                            2*x*y+2*w*z, w*w-x*x+y*y-z*z,     2*y*z-2*w*x,
                            2*x*z-2*w*y,     2*y*z+2*w*x, w*w-x*x-y*y+z*z;

      forceest1.angular_v_measure << drone_imu.angular_velocity.x,
                                     drone_imu.angular_velocity.y,
                                     drone_imu.angular_velocity.z;

      forceest1.predict();
      Eigen::VectorXd measure;
      measure.setZero(measurementsize);

      measure << drone_pose.pose.position.x, drone_pose.pose.position.y, drone_pose.pose.position.z,
                 drone_vel.twist.linear.x, drone_vel.twist.linear.y, drone_vel.twist.linear.z,
                 measure_ex, measure_ey, measure_ez,
                 drone_imu.angular_velocity.x, drone_imu.angular_velocity.y, drone_imu.angular_velocity.z;

      forceest1.qk11 = forceest1.qk1;

      forceest1.correct(measure);
      forceest1.x[e_x] = 0;
      forceest1.x[e_y] = 0;
      forceest1.x[e_z] = 0;

      bias.x = forceest1.x[beta_x];
      bias.y = forceest1.x[beta_y];
      bias.z = forceest1.x[beta_z];

      euler.x = forceest1.euler_angle(0);//roll:forceest1.euler_angle(0)
      euler.y = forceest1.euler_angle(1);//pitch:forceest1.euler_angle(1)
      euler.z = forceest1.euler_angle(2);//yaw:forceest1.euler_angle(2)

      angular_v.x = drone_imu.angular_velocity.x;
      angular_v.y = drone_imu.angular_velocity.y;
      angular_v.z = drone_imu.angular_velocity.z;
      tf::Quaternion quat_transform_ref(drone_pose.pose.orientation.x, drone_pose.pose.orientation.y, drone_pose.pose.orientation.z, drone_pose.pose.orientation.w);
      double roll_ref, pitch_ref, yaw_ref;

      tf::Matrix3x3(quat_transform_ref).getRPY(roll_ref, pitch_ref, yaw_ref);

      euler_ref.x = roll_ref*180/3.1415926;        //roll_ref*180/3.1415926
      euler_ref.y = pitch_ref*180/3.1415926;       //pitch_ref*180/3.1415926
      euler_ref.z = yaw_ref*180/3.1415926;         //yaw_ref*180/3.1415926

      euler_pub.publish(euler);
      euler_ref_pub.publish(euler_ref);

      force.x = forceest1.x[F_x];
      force.y = forceest1.x[F_y];
      force.z = forceest1.x[F_z];
      torque.z = forceest1.x[tau_z];

      force_pub.publish(force);
    }

    loop_rate.sleep();
    ros::spinOnce();
  }
}
