#!/usr/bin/env python3
import rclpy
import sys, signal, os
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from getkey import getkey

MAX_LIN_VEL = 0.36
MAX_ANG_VEL = 1.4
LIN_VEL_STEP_SIZE = 0.020
ANG_VEL_STEP_SIZE = 0.10

msg = """
-------------------------------------------------
Moving around:
        w    e
   a    s    d
        x

w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity

e : torque on/off
s : force stop

CTRL-C to quit
-------------------------------------------------
"""

class TeleopKey(Node):
    def __init__(self):
        super().__init__("teleop_key_node")

        qos_profile = rclpy.qos.QoSProfile(depth=10, reliability=rclpy.qos.ReliabilityPolicy.RELIABLE)

        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", qos_profile)
        self.motor_torque_flag_pub = self.create_publisher(Bool, "/motor_torque_flag", 10)
        signal.signal(signal.SIGINT, self.signal_handler) #callback if ctrl+C signal is input.

        target_linear_vel = 0.0
        target_angular_vel = 0.0
        
        onoff_state = Bool()
        onoff_state.data = True

        os.system('clear')
        print(msg)

        while(1):
            key = getkey()
            if key == "w":
                target_linear_vel = self.CheckLinearVelLimit(target_linear_vel + LIN_VEL_STEP_SIZE)
                os.system('clear')
                print(msg)
                print("linear_vel = ", target_linear_vel, "| angular_vel = ", target_angular_vel)
            elif key == "x":
                target_linear_vel = self.CheckLinearVelLimit(target_linear_vel - LIN_VEL_STEP_SIZE)
                os.system('clear')
                print(msg)
                print("linear_vel = ", target_linear_vel, "| angular_vel = ", target_angular_vel)
            elif key == "a":
                target_angular_vel = self.CheckAngularVelLimit(target_angular_vel + ANG_VEL_STEP_SIZE)
                os.system('clear')
                print(msg)
                print("linear_vel = ", target_linear_vel, "| angular_vel = ", target_angular_vel)
            elif key == "d":
                target_angular_vel = self.CheckAngularVelLimit(target_angular_vel - ANG_VEL_STEP_SIZE)
                os.system('clear')
                print(msg)
                print("linear_vel = ", target_linear_vel, "| angular_vel = ", target_angular_vel)
            elif key == "s":
                target_linear_vel = 0.0
                target_angular_vel = 0.0
                os.system('clear')
                print(msg)
                print("linear_vel = ", target_linear_vel, "| angular_vel = ", target_angular_vel)
            elif key == 'e':
                if onoff_state.data == False:
                    onoff_state.data = True
                    os.system('clear')
                    print(msg)
                    print ("Motor ON/OFF Flag : ON")
                else:
                    onoff_state.data = False
                    os.system('clear')
                    print(msg)
                    print ("Motor ON/OFF Flag : OFF")
                self.motor_torque_flag_pub.publish(onoff_state)
            twist = Twist()
            twist.linear.x = target_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = target_angular_vel
            self.cmd_vel_pub.publish(twist)
    

    def signal_handler(self, sig, frame):
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
            
        print("linear_vel = ", twist.linear.x, "| angular_vel = ", twist.angular.z)
        sys.exit(0)

    def CheckLinearVelLimit(self, vel):
        if vel <= -MAX_LIN_VEL:
            vel = -MAX_LIN_VEL
        elif vel >= MAX_LIN_VEL:
            vel = MAX_LIN_VEL
        
        return round(vel,2)

    def CheckAngularVelLimit(self, vel):
        if vel <= -MAX_ANG_VEL:
            vel = -MAX_ANG_VEL
        elif vel >= MAX_ANG_VEL:
            vel = MAX_ANG_VEL
        
        return round(vel,2)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKey()
    rclpy.spin(node)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()