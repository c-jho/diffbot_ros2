#include "diffbot_core/com.hpp"
#include "diffbot_core/diffbot_base.hpp"

tf2::Quaternion qt;
geometry_msgs::msg::Quaternion odom_qt;
double last_time=0;

int initMB(BaseControl &data)
{
    //하드웨어 데이터 셋업
    BC.Tick2RAD     = 0.0104719755;         // 360 / 600(1회전당 펄스 수) * π / 180  =  0.0104719755
    BC.nPPR         = 450;
    BC.Tick2RAD     = (360.0/BC.nPPR)*PI / 180;

    BC.id = 1;

    BC.rpm_array[0][0] = 0;
    BC.rpm_array[0][1] = 1;
    BC.rpm_array[1][0] = 2;
    BC.rpm_array[1][1] = 3;
    

    for(int i = 0; i < WHEEL_NUM; i++){
        BC.cmd_xw[i] = 0;
        BC.InitError[i] = 0;
        BC.current_tick[i] = 0;
        BC.last_tick[i] = 0;
        BC.last_theta = 0;
        BC.delta_s = 0;
        BC.delta_theta = 0;
        BC.wheel_l = 0;
        BC.wheel_r = 0;
    }
    return 1;
}

int initOdom(nav_msgs::msg::Odometry &odom_)
{
    for(int i  = 0; i < 2; i++){
        BC.last_base_vel[i] = 0;
    }
    odom_.pose.pose.position.x = 0;
    odom_.pose.pose.position.y = 0;
    odom_.pose.pose.position.z = 0;
    odom_.pose.pose.orientation.x = 0;
    odom_.pose.pose.orientation.y = 0;
    odom_.pose.pose.orientation.z = 0;
    odom_.pose.pose.orientation.w = 0;

    odom_.twist.twist.linear.x = 0;
    odom_.twist.twist.linear.y = 0;
    odom_.twist.twist.angular.z = 0;
}

int initJointStates(sensor_msgs::msg::JointState &joint_states_)
{
    static char *joint_states_name[] = {(char*)"wheel_joint_left", (char*)"wheel_joint_right"};

    joint_states_.header.frame_id = "base_link";
    joint_states_.name.push_back(joint_states_name[0]);
    joint_states_.name.push_back(joint_states_name[1]);
    joint_states_.name.push_back(joint_states_name[2]);

    joint_states_.name.resize(WHEEL_NUM);
    joint_states_.position.resize(WHEEL_NUM);
    joint_states_.velocity.resize(WHEEL_NUM);
    joint_states_.effort.resize(WHEEL_NUM);
}

int calcOdometry(double current_time)
{
    double dt = current_time - last_time;

    BC.wheel_l = -BC.Tick2RAD * (double)BC.last_diff_tick[LEFT];
    BC.wheel_r = BC.Tick2RAD * (double)BC.last_diff_tick[RIGHT];

    if (isnan(BC.wheel_l))
        BC.wheel_l = 0.0;

    if (isnan(BC.wheel_r))
        BC.wheel_r = 0.0;
    
    BC.delta_s = (wheel_diameter/2) * (BC.wheel_l + BC.wheel_r) / 2.0;
    BC.delta_theta = (wheel_diameter/2) * (BC.wheel_r - BC.wheel_l) / robot_width;

    BC.last_odom_pose[0] += BC.delta_s * cos(BC.last_odom_pose[2] + (BC.delta_theta / 2.0));
    BC.last_odom_pose[1] += BC.delta_s * sin(BC.last_odom_pose[2] + (BC.delta_theta / 2.0));
    BC.last_odom_pose[2] += BC.delta_theta;

    BC.last_vel[0] = BC.wheel_l / dt;
    BC.last_vel[1] = BC.wheel_r / dt;

    last_time = current_time;
    return 1;
}

int updateOdometry(nav_msgs::msg::Odometry &odom_)
{
    qt.setRPY(0.0, 0.0, BC.last_odom_pose[2]);
    odom_qt.x = (float)qt.x();
    odom_qt.y = (float)qt.y();
    odom_qt.z = (float)qt.z();
    odom_qt.w = (float)qt.w();

    odom_.header.frame_id = "/odom";
    odom_.child_frame_id  = "/base_footprint";

    odom_.pose.pose.position.x = BC.last_odom_pose[0];
    odom_.pose.pose.position.y = BC.last_odom_pose[1];
    odom_.pose.pose.position.z = 0;
    odom_.pose.pose.orientation = odom_qt;

    // odom_.twist.twist.linear.x = BC.last_base_vel[0];
    // odom_.twist.twist.linear.y = BC.last_base_vel[1];
    // odom_.twist.twist.angular.z = BC.last_base_vel[2];

    return 1;
}

int updateTF(geometry_msgs::msg::TransformStamped &odom_tf_, nav_msgs::msg::Odometry odom_)
{
    odom_tf_.header = odom_.header;
    odom_tf_.child_frame_id = odom_.child_frame_id;

    odom_tf_.transform.translation.x = odom_.pose.pose.position.x;
    odom_tf_.transform.translation.y = odom_.pose.pose.position.y;
    odom_tf_.transform.translation.z = odom_.pose.pose.position.z;
    odom_tf_.transform.rotation      = odom_.pose.pose.orientation;

    return 1;
}

int updateJointStates(sensor_msgs::msg::JointState &joint_states_)
{
    for(int i = 0; i < WHEEL_NUM; i++){
        BC.joint_states_vel[i] = BC.last_vel[i];
        BC.joint_states_pos[i] = -BC.last_rad[i];

        joint_states_.position[i] = BC.joint_states_pos[i];
        joint_states_.velocity[i] = BC.joint_states_vel[i];
    }
}

float *CmdVel2RPM(float cmd_vel[])
{
    float RPM[WHEEL_NUM] = {0.0, 0.0};
    float *return_rpm = RPM;
    
    RPM[LW] = -transRPM((cmd_vel[X]-cmd_vel[W]*robot_width/2)/(wheel_diameter/2));
    RPM[RW] = transRPM((cmd_vel[X]+cmd_vel[W]*robot_width/2)/(wheel_diameter/2));

    BC.joint_RPM[LW] = RPM[LW];
    BC.joint_RPM[RW] = RPM[RW];
    
    // cout << BC.joint_RPM[LW] << "  :  " << BC.joint_RPM[RW] << endl;
    
    return return_rpm;
}


float transRPM(float m_per_s)
{
    float rpm = m_per_s * 60 / (2 * PI);
    return rpm;
}

float transVel(float rev_per_m)
{
    float vel = rev_per_m * (PI * wheel_diameter) / 60;
    return vel;
}