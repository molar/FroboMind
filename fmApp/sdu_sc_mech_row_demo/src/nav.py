#!/usr/bin/env python 

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from msgs.msg import serial
from std_msgs.msg import Float64
from std_msgs.msg import Bool

from dynamic_reconfigure.server import Server
from sdu_sc_mech_row_demo.cfg import sdu_sc_mech_row_demo_paramsConfig

class NavNode():
    def __init__(self):
        self.input_topic = rospy.get_param("~line_subscriber_topic","~line_input")
        self.output_topic = rospy.get_param("~vel_publisher_topic","~line_output")
        self.automode_topic = rospy.get_param("~automode_sub",'/fmDecisionMakers/automode')
        
        self.automode = False
        self.automode_prev = False
        self.sum = 0.0
        
        self.offset = float(rospy.get_param("~input_offset","0.0"))
        self.x = float(rospy.get_param("~forward_velocity","0.1"))
        self.vel_pub = rospy.Publisher(self.output_topic,TwistStamped)
        self.vel_msg = TwistStamped()
        self.prev_error = 0.0
        self.error_msg = Float64()
        self.error_pub = rospy.Publisher("~error",Float64)
        self.last_input = rospy.Time(0)
        
        self.feedback_sub = rospy.Subscriber(self.input_topic,serial,self.on_serial)
        rospy.Subscriber(self.automode_topic, Bool, self.on_automode_message)

        self.looptimer = rospy.Timer(rospy.Duration(0.1),self.loop)
        self.dynserver = Server(sdu_sc_mech_row_demo_paramsConfig,self.dyn_cb_pid_params)
        
        
        
    def on_serial(self,msg):
        
        self.last_input = msg.header.stamp
        try:
            self.val = float(msg.data)
        except ValueError:
            pass
        
    def calculate_correction(self):
        error = self.offset - self.val
        
        self.error_msg.data = error
        self.error_pub.publish(self.error_msg)
        diff = error - self.prev_error
        self.sum += error
        
        if self.sum > self.pid_params.Ilim:
            self.sum = self.pid_params.Ilim
        
        if self.sum < - self.pid_params.Ilim:
            self.sum = -self.pid_params.Ilim
        
        out = error * self.pid_params.P + self.sum * self.pid_params.I +  diff * self.pid_params.D 
        
        self.prev_error = error
        
        return out
    
    def on_automode_message(self, msg):
        self.automode = msg.data
        if self.automode != self.automode_prev:
            self.automode_prev = self.automode
            if self.automode:
                rospy.loginfo("Switching to auto")
            else:
                rospy.loginfo("Switching to manual")
        
        
    def loop(self,timerevent):
        """
            
        """
        if (rospy.Time.now() - self.last_input).to_sec() < 1:
            if self.automode:
                # calculate error and correction
                
                self.vel_msg.twist.linear.x = self.x
                self.vel_msg.twist.angular.z = self.calculate_correction()
                if self.vel_msg.twist.angular.z > 0.5:
                    self.vel_msg.twist.angular.z = 0.5
                if self.vel_msg.twist.angular.z < -0.5:
                    self.vel_msg.twist.angular.z = -0.5
            else:
                
                self.vel_msg.twist.linear.x = 0.0
                self.vel_msg.twist.angular.z = 0.0
        else:
            rospy.loginfo("Time too old for sensor message")
            self.vel_msg.twist.linear.x = 0.0
            self.vel_msg.twist.angular.z = 0.0
            
        if self.automode:
            self.vel_msg.header.stamp = rospy.Time.now()
            self.vel_pub.publish(self.vel_msg)

    def dyn_cb_pid_params(self,config,level):
        self.pid_params = config
        return config
    
if __name__ == "__main__":
    rospy.init_node("driver")
    
    node = NavNode()
    
    rospy.spin()
    