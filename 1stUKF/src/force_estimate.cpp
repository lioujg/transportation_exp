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
#include "proj_conf.h"
#include <float.h>
#include <cmath>
#include <queue>
#include <iostream>

#define l 0.25
#define k 0.02
std::string model_name;
forceest forceest1(statesize,measurementsize);
geometry_msgs::Point euler, euler_ref, force, torque, bias, angular_v, pose;
sensor_msgs::Imu drone_imu;
geometry_msgs::PoseStamped optitrack_data, drone_pose, last_pose;
geometry_msgs::TwistStamped drone_vel;
float dt = 0.02;
int nan_count = 0;
bool flag = false;
float ukf_lpf_gain;
float ukf_estimated_force_enu[3];
float ukf_estimated_force_enu_lpf[3];
std::queue<float> x_bias_queue;
std::queue<float> y_bias_queue;
float bias_sum[2] = {0.0f};
float bias_mean[2] = {0.0f};

void lpf_first_order_init(float *ret_gain, float sampling_time, float cutoff_freq)
{
  //reference: low pass filter (wikipedia)

  //return the alpha value of the first order low pass filter
  *ret_gain = sampling_time / (sampling_time + 1/ (2 * M_PI * cutoff_freq));
}

void lpf_first_order(float origin, float *filtered, float alpha)
{
  *filtered = (origin * alpha) + (*filtered * (1.0f - alpha));
}

void ukf_low_pass_filter(float * ukf_estimated_force_enu, float * ukf_estimated_force_enu_lpf)
{
  //parameter.ukf_lpf_gain = 0.13f;
  lpf_first_order(ukf_estimated_force_enu[0], &(ukf_estimated_force_enu_lpf[0]), ukf_lpf_gain);
  lpf_first_order(ukf_estimated_force_enu[1], &(ukf_estimated_force_enu_lpf[1]), ukf_lpf_gain);
  lpf_first_order(ukf_estimated_force_enu[2], &(ukf_estimated_force_enu_lpf[2]), ukf_lpf_gain);
}

void imu_cb(const sensor_msgs::Imu::ConstPtr &msg){
  drone_imu = *msg;
}

Eigen::Vector3d thrust, last_thrust;
void thrust_cb(const geometry_msgs::WrenchStamped::ConstPtr &msg){
  // if(isnan(msg->wrench.force.z) == 0){
  //   thrust << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z;}
  // else if(isnan(msg->wrench.force.z) != 0){
  //   thrust(2) = last_thrust(2);
  //   std::cout << "I meet something cool like nan!!" << std::endl;
  //   nan_count++;
  // }
  // std::cout << thrust(2) << std::endl;
  // last_thrust = thrust;

  thrust << 0, 0, 23;
}

float ground_truth_yaw;

void optitrack_cb(const geometry_msgs::PoseStamped::ConstPtr &msg){
  optitrack_data = *msg;

  drone_pose.pose = optitrack_data.pose;

  drone_vel.twist.linear.x = (drone_pose.pose.position.x - last_pose.pose.position.x)/dt;
  drone_vel.twist.linear.y = (drone_pose.pose.position.y - last_pose.pose.position.y)/dt;
  drone_vel.twist.linear.z = (drone_pose.pose.position.z - last_pose.pose.position.z)/dt;
  last_pose.pose = optitrack_data.pose;

  //yaw
  double quaternion_w, quaternion_x, quaternion_y, quaternion_z;
  double payload_roll, payload_yaw, payload_pitch;
  quaternion_x = optitrack_data.pose.orientation.x;
  quaternion_y = optitrack_data.pose.orientation.y;
  quaternion_z = optitrack_data.pose.orientation.z;
  quaternion_w = optitrack_data.pose.orientation.w;
  tf::Quaternion quaternion(quaternion_x, quaternion_y, quaternion_z, quaternion_w);
  tf::Matrix3x3(quaternion).getRPY(payload_roll, payload_pitch, payload_yaw);
  ground_truth_yaw = payload_yaw * 180 / M_PI;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "force_estimate");
  ros::NodeHandle nh;

  ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/imu/data_raw",4,imu_cb);
#if (MAV_SELECT == LEADER)
  ros::Subscriber pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/MAV1/pose",4,optitrack_cb);
#pragma message("I'm leader!")
#elif (MAV_SELECT == FOLLOWER)
  ros::Subscriber pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/MAV2/pose",4,optitrack_cb);
#pragma message("I'm follower!")
#endif
  ros::Subscriber thrust_sub = nh.subscribe<geometry_msgs::WrenchStamped>("/rotor_all_ft",4,thrust_cb);

  ros::Publisher force_pub = nh.advertise<geometry_msgs::Point>("force_estimate",2);

  ros::Rate loop_rate(20);

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

  // sampling time = 0.05s (20Hz), cutoff frequency = 5Hz
  lpf_first_order_init(&ukf_lpf_gain, 0.05, 5);

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

      //F1 = f3(2);//(6.13176e-06*(pwm3*pwm3) -0.0181164*pwm3 + 15.9815); //drone
      //F2 = f1(2);//(6.13176e-06*(pwm1*pwm1) -0.0181164*pwm1 + 15.9815); //left_right:265.7775
      //F3 = f4(2);//(6.13176e-06*(pwm4*pwm4) -0.0181164*pwm4 + 15.9815); //up_down:265.7775
      //F4 = f2(2);//(6.13176e-06*(pwm2*pwm2) -0.0181164*pwm2 + 15.9815);

      forceest1.thrust = thrust(2);

      U_x = 0;//(sqrt(2)/2)*l*(F1 - F2 - F3 + F4);
      U_y = 0;//(sqrt(2)/2)*l*(-F1 - F2 + F3 + F4);
      U_z = 0;//k*F1 - k*F2 + k*F3 - k*F4;

      forceest1.U << U_x, U_y, U_z;
      float x = drone_pose.pose.orientation.x;
      float y = drone_pose.pose.orientation.y;
      float z = drone_pose.pose.orientation.z;
      float w = drone_pose.pose.orientation.w;

      forceest1.R_IB.setZero();
      forceest1.R_IB << w*w+x*x-y*y-z*z,     2*x*y-2*w*z,     2*x*z+2*w*y,
                            2*x*y+2*w*z, w*w-x*x+y*y-z*z,     2*y*z-2*w*x,
                            2*x*z-2*w*y,     2*y*z+2*w*x, w*w-x*x-y*y+z*z;

      // forceest1.R_IB <<   cos(ground_truth_yaw), sin(ground_truth_yaw),   0,
      //                    -sin(ground_truth_yaw), cos(ground_truth_yaw),   0,
      //                                         0,                     0,   1;

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

      ukf_estimated_force_enu[0] = forceest1.x[F_x]; // bias
      ukf_estimated_force_enu[1] = forceest1.x[F_y];
      ukf_estimated_force_enu[2] = forceest1.x[F_z];
      torque.z = forceest1.x[tau_z];

      ukf_low_pass_filter(ukf_estimated_force_enu, ukf_estimated_force_enu_lpf);

      force.x = ukf_estimated_force_enu_lpf[0];
      force.y = ukf_estimated_force_enu_lpf[1];
      force.z = ukf_estimated_force_enu_lpf[2];

      nh.getParam("/start",flag);

      printf("%d", nan_count);
      if(flag == true){
        force.x -= bias_mean[0];
        force.y -= bias_mean[1];
        printf("UKF estimated force  x: %f  y: %f  z: %f  bias x: %f y: %f\n", force.x, force.y, force.z, bias_mean[0], bias_mean[1]);
      }
      else{
        printf("!UKF estimated force  x: %f  y: %f  z: %f  bias x: %f y: %f\n", force.x, force.y, force.z, bias_mean[0], bias_mean[1]);
        x_bias_queue.push(force.x);
        y_bias_queue.push(force.y);
        bias_sum[0] += x_bias_queue.back();
        bias_sum[1] += y_bias_queue.back();

        if(x_bias_queue.size() > 100 || y_bias_queue.size() > 100){
          bias_sum[0] -= x_bias_queue.front();
          bias_sum[1] -= y_bias_queue.front();
          x_bias_queue.pop();
          y_bias_queue.pop();
        }

        bias_mean[0] = bias_sum[0] / x_bias_queue.size();
        bias_mean[1] = bias_sum[1] / y_bias_queue.size();
        force.x = 0.0;
        force.y = 0.0;
      }
      force_pub.publish(force);
    }


    loop_rate.sleep();
    ros::spinOnce();
  }
}
