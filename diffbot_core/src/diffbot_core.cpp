#include "diffbot_core/com.hpp"
#include "diffbot_core/diffbot_base.hpp" 

Communication Com;  
BaseControl BC;

geometry_msgs::msg::TransformStamped odom_tf;
nav_msgs::msg::Odometry odom;
sensor_msgs::msg::JointState joint_states;

BYTE SendCmdVel = OFF;
int id_num, speed[4] = {0,};
float rpm_[2] = {0,};

void CmdVelCallBack(const geometry_msgs::msg::Twist::SharedPtr msg) {
    BC.cmd_xw[X] = msg->linear.x;
    BC.cmd_xw[W] = msg->angular.z;

    BC.rpm_data = CmdVel2RPM(BC.cmd_xw);

    //BC.rpm_data에 저장된 데이터가 main에서 유실되는 현상 방지를 위함
    rpm_[0] = BC.rpm_data[0];
    rpm_[1] = BC.rpm_data[1];

    SendCmdVel = ON;
}
void MotorTorqueCallBack(const std_msgs::msg::Bool::SharedPtr msg)
{
    BC.torque_flag = msg->data;
}

//=======================================================================================================================//
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("diffbot_core");

    rclcpp::Time stamp_now;
    tf2_ros::TransformBroadcaster tf_broadcaster_(node);

    //publisher
    auto motor_info_pub = node->create_publisher<diffbot_msgs::msg::Motorinfo>("/motor_information", 1000);
    auto odom_pub = node->create_publisher<nav_msgs::msg::Odometry>("odom", 1000);
    auto joint_states_pub = node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1000);

    //subscriber
    auto vel_sub = node->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel",1000, CmdVelCallBack);
    auto motor_torque_flag_sub = node->create_subscription<std_msgs::msg::Bool>("/motor_torque_flag",1000, MotorTorqueCallBack);
    auto info_msg = diffbot_msgs::msg::Motorinfo();


    Com.nIDMDUI = 184;
    Com.nIDMDT = 183;
    Com.nBaudrate = 57600;
    Com.nGearRatio = 25;

    IByte iData;
    int nArray[2], rpmArray[4];
    static BYTE InitMotor, fgInitsetting, byCntInitStep, byCntComStep, byCnt2500us, byCntStartDelay,byCntCase[5];
    
    byCntInitStep     = 1;
    InitMotor         = ON;
    fgInitsetting     = OFF;

    initMB(BC);
    initOdom(odom);
    initJointStates(joint_states);
    InitSerial();   //communication initialization in com.cpp

    while (rclcpp::ok()) {
        ReceiveDataFromController(InitMotor);
        if(++byCnt2500us == 50)
        {
            byCnt2500us = 0;

            if(fgInitsetting == ON)
            {
                switch(++byCntComStep)
                {
                case 1:{ //Odometry publish
                    stamp_now = node->get_clock()->now();

                    calcOdometry(node->get_clock()->now().seconds());
                    updateOdometry(odom);
                    odom.header.stamp = stamp_now;
                    odom_pub->publish(odom);

                    updateTF(odom_tf, odom);
                    odom_tf.header.stamp = stamp_now;
                    tf_broadcaster_.sendTransform(odom_tf);
                    // printf("in1\n");
                    break;
                }
                case 2: //Control wheel
                    if(++byCntCase[byCntComStep] == TIME_50MS)
                    {
                        byCntCase[byCntComStep] = 0;

                        if(SendCmdVel && BC.torque_flag == true)
                        {
                            for(id_num = 0; id_num < WHEEL_NUM; id_num++){
                                iData = Short2Byte(rpm_[id_num] * Com.nGearRatio); // #1,2 Wheel RPM
                                rpmArray[BC.rpm_array[id_num][0]] = iData.byLow;
                                rpmArray[BC.rpm_array[id_num][1]] = iData.byHigh;
                            }
                            PutMdData(PID_VEL_CMD, Com.nIDMDT, rpmArray);

                            //n대의 모터드라이버에게 동시에 main data를 요청할 경우 data를 받을 때 데이터가 섞임을 방지.
                            nArray[0] = PID_MAIN_DATA;
                            PutMdData(PID_REQ_PID_DATA, Com.nIDMDT, BC.id, nArray);  // Main data request
                        
                            SendCmdVel = IDupdate(BC.id, SendCmdVel);
                        }
                        else
                        {
                            //n대의 모터드라이버에게 동시에 main data를 요청할 경우 data를 받을 때 데이터가 섞임을 방지.
                            nArray[0] = PID_MAIN_DATA;
                            PutMdData(PID_REQ_PID_DATA, Com.nIDMDT, BC.id, nArray);  // Main data request

                            SendCmdVel = IDupdate(BC.id, SendCmdVel);
                            //------------------------------------------------------------------
                        }
                    }
                    break;  

                case 3: //Motor RPM & POS update
                    for(id_num = 0; id_num < WHEEL_NUM; id_num++){
                        info_msg.motor[id_num].id   = id_num+1;
                        info_msg.motor[id_num].rpm  = Com.sMotorRPM[id_num]/Com.nGearRatio;
                        info_msg.motor[id_num].pos  = Com.lMotorPosi[id_num];

                        //motor pos update//
                        BC.current_tick[id_num]     = Com.lMotorPosi[id_num];

                        BC.last_diff_tick[id_num]   = BC.current_tick[id_num] - BC.last_tick[id_num];
                        BC.last_tick[id_num]        = BC.current_tick[id_num];
                        BC.last_rad[id_num]        += BC.Tick2RAD * (double)BC.last_diff_tick[id_num];
                        // printf("%d rad : %f\n",id_num,BC.last_rad[id_num]);
                    }

                    motor_info_pub->publish(info_msg);
                    break;

                case 4: //Joint state update
                    // printf("in4\n");
                    stamp_now = node->get_clock()->now();

                    updateJointStates(joint_states);
                    joint_states.header.stamp = stamp_now;
                    joint_states_pub->publish(joint_states);

                    break;
                
                case 5: //Motor torque onoff
                    if(BC.torque_flag == false){
                        nArray[0] = ON;
                        PutMdData(PID_TQ_OFF, Com.nIDMDT, nArray);
                        BC.TQ_off = true;
                    }
                    else if(BC.torque_flag == true && BC.TQ_off == true){
                        for(int i = 0; i < 4; i++) rpmArray[i] = 0;
                        PutMdData(PID_VEL_CMD, Com.nIDMDT, rpmArray);
                        BC.TQ_off = false;
                    }
                    break;

                case 6:
                    // printf("in6\n");

                    byCntComStep = 0;
                    break;
                }
            }
            else
            {
                if(byCntStartDelay <= 200) byCntStartDelay++;
                else
                {
                    switch (byCntInitStep)
                    {
                    case 1: //Motor connect check
                        nArray[0] = PID_MAIN_DATA;
                        PutMdData(PID_REQ_PID_DATA, Com.nIDMDT, BC.id, nArray);

                        for(int i = 1; i <= WHEEL_NUM; i++){
                            if(BC.id == i)
                                BC.InitError[i-1]++;

                            if(BC.InitError[i-1] > 10){
                                if(i == 1){ RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"),"ID 1 MOTOR INIT ERROR!!"); return 0;}
                                if(i == 2){ RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"),"ID 2 MOTOR INIT ERROR!!"); return 0;}
                            }
                        }
                        if(BC.id > WHEEL_NUM){
                            byCntInitStep++;
                            BC.id = 1;
                            InitMotor = OFF;
                        }
                        break;

                    case 2:
                        byCntInitStep++;
                        break;

                    case 3:  //Motor torque ON
                        for(int i = 0; i < 6; i++) rpmArray[i] = 0;
                        PutMdData(PID_VEL_CMD, Com.nIDMDT, rpmArray);
                        BC.torque_flag = true;
                        BC.TQ_off = false;
                        byCntInitStep++;
                        break;

                    case 4: //Motor POS reset
                        nArray[0] = 0;
                        PutMdData(PID_POSI_RESET, Com.nIDMDT, nArray);
                        byCntInitStep++;
                        break;

                    case 5:
                        printf("========================================================\n\n");
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ROBOT INIT END\n");
                        fgInitsetting = ON;

                        break;

                    }
                }
            }
        }

        rclcpp::spin_some(node);
    }

    rclcpp::shutdown();
    return 0;
}
