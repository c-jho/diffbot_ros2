#include "diffbot_msgs/msg/motorinfo.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2/LinearMath/Quaternion.h>


#define robot_width     0.3 // mm  
#define wheel_diameter  0.08 // mm
#define PI              3.14159265359
#define X		        0
#define W		        1
#define LEFT            0
#define RIGHT           1
#define WHEEL_NUM       2

typedef struct{
    bool torque_flag, TQ_off;
    float *rpm_data, *base_vel;
    int rpm_array[WHEEL_NUM][2], id, InitError[WHEEL_NUM], nPPR;
    float wheel_l, wheel_r, delta_s, delta_theta, theta, last_theta, last_velocity[WHEEL_NUM];
    float cmd_xw[3], Tick2RAD, current_tick[WHEEL_NUM], last_tick[WHEEL_NUM], last_diff_tick[WHEEL_NUM], last_rad[WHEEL_NUM], last_vel[WHEEL_NUM], odom_pose[3], last_base_vel[2], last_odom_pose[3];
    float joint_states_pos[WHEEL_NUM], joint_states_vel[WHEEL_NUM], wheel_rad_s[WHEEL_NUM], joint_RPM[WHEEL_NUM];
}BaseControl;
extern BaseControl BC;

extern int initMB(BaseControl &data);
extern int initOdom(nav_msgs::msg::Odometry &odom_);
extern int initJointStates(sensor_msgs::msg::JointState &joint_states_);
extern int calcOdometry(double current_time);
extern int updateOdometry(nav_msgs::msg::Odometry &odom);
extern int updateTF(geometry_msgs::msg::TransformStamped &odom_tf_, nav_msgs::msg::Odometry odom_);
extern int updateJointStates(sensor_msgs::msg::JointState &joint_states_);
float *CmdVel2RPM(float cmd_vel[]);
extern float transRPM(float m_per_s);
extern float transVel(float rev_per_m);
