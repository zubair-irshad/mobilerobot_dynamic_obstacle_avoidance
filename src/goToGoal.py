#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry

import math

class process(object):
    def __init__(self):
        self.subscriber_odom = rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.subscriber_lidar = rospy.Subscriber("/lin_cord", Point, self.lidar_callback)
        self.publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=5)

        self.Init = True
        self.Init_ang = None

        self.Init_pos = Point()
        self.Init_pos.x = 0
        self.Init_pos.y = 0
        self.Init_pos.z = 0

        self.globalAng = 0
        self.globalPos = Point()
        self.globalPos.x = 0
        self.globalPos.y = 0
        self.Init_pos.z = 0

        self.x_start = 0
        self.y_start = 0

        self.x_1 = 1.4
        self.y_1 = 0

        self.x_2 = 1.4
        self.y_2 = 1.5

        self.x_3 = 0
        self.y_3 = 1.5

        self.cmd_twist = Twist()

        self.cmd_twist.linear.x = 0
        self.cmd_twist.linear.y = 0
        self.cmd_twist.linear.z = 0
        self.cmd_twist.angular.x = 0
        self.cmd_twist.angular.y = 0
        self.cmd_twist.angular.z = 0 

        self.phase = "1"
    
    def lidar_callback(self, lindata):

        self.varsaw=lindata.x
        self.localangle = lindata.y
        # print ("Range:", self.varsaw)
        # print ("Angle:", self.localangle)
        
        # extract minDist and angle here

    def odom_callback(self, odom):
        self.update_Odometry(odom)

        

        if self.phase == "1":

            desired_angle0 = np.arctan2((self.y_1 - self.globalPos.y),(self.x_1 - self.globalPos.x))
            ang_error0 =  desired_angle0 - self.globalAng
            #print(ang_error0)

            # if desired_angle0 - self.globalAng > math.pi:
            #     ang_error20 = ang_error0 - (2*math.pi)
            # if desired_angle0 - self.globalAng <-math.pi:
            #     ang_error0 = ang_error0 + (2*math.pi)

            distance1 = np.sqrt(np.power((self.globalPos.x - self.x_start), 2) + np.power((self.globalPos.y - self.y_start), 2))
            lin_error =  self.x_1 - distance1

            if lin_error > 0.05 or ang_error0 >0.05:
                lin_vel = 0.5 * lin_error
                ang_vel0 = 4 * ang_error0
                self.cmd_twist.linear.x = lin_vel
                self.cmd_twist.angular.z = ang_vel0
            else:
                self.cmd_twist.linear.x = 0
                self.cmd_twist.angular.z = 0

                #print("I am here and ready to turn")

                desired_angle = np.arctan2((self.y_2 - self.globalPos.y),(self.x_2 - self.globalPos.x))

                ang_error =  desired_angle - self.globalAng

                if desired_angle - self.globalAng > math.pi:
                    ang_error = ang_error - (2*math.pi)
                if desired_angle - self.globalAng <-math.pi:
                    ang_error = ang_error + (2*math.pi)

                ang_vel = 0.5 * ang_error

                if ang_error >0.05:
                    self.cmd_twist.angular.z = ang_vel
                else:
                    self.cmd_twist.angular.z = 0
                    self.phase = "2"

        if self.phase == "2":

            distance_goal = np.sqrt(np.power((self.x_2 - self.x_start), 2) + np.power((self.y_2 - self.y_start), 2))

            distance2 = np.sqrt(np.power((self.globalPos.x - self.x_start), 2) + np.power((self.globalPos.y - self.y_start), 2))

            lin_error2 =  distance_goal - distance2

            desired_angle2 = np.arctan2((self.y_2 - self.globalPos.y),(self.x_2 - self.globalPos.x))
            ang_error2 =  desired_angle2 - self.globalAng

            if desired_angle2 - self.globalAng > math.pi:
                ang_error2 = ang_error2 - (2*math.pi)
            if desired_angle2 - self.globalAng <-math.pi:
                ang_error2 = ang_error2 + (2*math.pi)

            if self.globalPos.y <0.85:
                ang_vel2 = 8 * ang_error2
            else:
                ang_vel2 = 9 * ang_error2

            if self.varsaw > 0.4:

                print(self.varsaw)

                if lin_error2 > 0.2 or ang_error2 > 0.05:
                    self.cmd_twist.linear.x = 0.5
                    self.cmd_twist.angular.z = ang_vel2
                else:

                    self.cmd_twist.linear.x = 0
                    self.cmd_twist.angular.z = 0

                    desired_angle4 = np.arctan2((self.y_3 - self.globalPos.y),(self.x_3 - self.globalPos.x))

                    ang_error4 =  desired_angle4 - self.globalAng

                    if ang_error4 > math.pi:
                        ang_error4 = ang_error4 - (2*math.pi)
                    if ang_error4 <-math.pi:
                        ang_error4 = ang_error4 + (2*math.pi)

                    ang_vel4 = 8 * ang_error4

                    if ang_error4 >0.05:
                        self.cmd_twist.angular.z = ang_vel4
                    else:
                        self.cmd_twist.angular.z = 0
                        self.phase = "3"
                    


            else:

                desired_angle3 = self.localangle + self.globalAng + (math.pi/2)

                if desired_angle3 > math.pi:
                    desired_angle3 = desired_angle3 - (2*math.pi)
                elif desired_angle3 <-math.pi:
                    desired_angle3 = desired_angle3 + (2*math.pi)

                ang_error3 = desired_angle3 - self.globalAng

                if ang_error3 > math.pi:
                    ang_error3 = ang_error3 - (2*math.pi)
                elif ang_error3 <-math.pi:
                    ang_error3 = ang_error3 + (2*math.pi)

                ang_vel3 = 0.5 * ang_error3

                #print(math.degrees(self.localangle), math.degrees(self.globalAng), math.degrees(desired_angle3), math.degrees(ang_error3))

                if ang_error3 >0.05:
                    self.cmd_twist.linear.x = 0.08
                    self.cmd_twist.angular.z = ang_vel3
                else:
                    self.cmd_twist.linear.x = 0.08
                    self.cmd_twist.angular.z = 0
                # self.cmd_twist.angular.z = 0.05

        if self.phase == "3":

            print("I am in phase 3")

            distance_goal3 = np.sqrt(np.power((self.x_3 - self.x_2), 2) + np.power((self.y_3 - self.y_2), 2))

            distance4 = np.sqrt(np.power((self.globalPos.x - self.x_2), 2) + np.power((self.globalPos.y - self.y_2), 2))

            lin_error4 =  distance_goal3 - distance4

            desired_angle6 = np.arctan2((self.y_3 - self.globalPos.y),(self.x_3 - self.globalPos.x))
            ang_error6 =  desired_angle6 - self.globalAng

            if ang_error6 > math.pi:
                ang_error6 = ang_error6 - (2*math.pi)
            if ang_error6 <-math.pi:
                ang_error6 = ang_error6 + (2*math.pi)

            ang_vel6 = 8 * ang_error6

            if self.varsaw > 0.4:

                if lin_error4 > 0.05 or ang_error6 >0.05:
                    self.cmd_twist.linear.x = 0.5
                    self.cmd_twist.angular.z = ang_vel6
                else:
                    self.cmd_twist.linear.x = 0
                    self.cmd_twist.angular.z = 0
            else:

                desired_angle5 = self.localangle + self.globalAng + (math.pi/2)

                if desired_angle5 > math.pi:
                    desired_angle5 = desired_angle5 - (2*math.pi)
                elif desired_angle5 <-math.pi:
                    desired_angle5 = desired_angle5 + (2*math.pi)

                ang_error5 = desired_angle5 - self.globalAng

                if ang_error5 > math.pi:
                    ang_error5 = ang_error5 - (2*math.pi)
                elif ang_error5 <-math.pi:
                    ang_error5 = ang_error5 + (2*math.pi)

                ang_vel5 = 1 * ang_error5

                if ang_error5 >0.05:
                    self.cmd_twist.linear.x = 0.08
                    self.cmd_twist.angular.z = ang_vel5
                else:
                    self.cmd_twist.linear.x = 0.08
                    self.cmd_twist.angular.z = 0
            
            # Get the current position
            #(position, rotation) = self.get_odom()
            # Compute the Euclidean distance from the start

        self.publisher.publish(self.cmd_twist)

        # elif self.phase == "2":

        # elif self.phase == "3":

        # else:
            # set all vels to 0 and publish

    def update_Odometry(self,Odom):
        
        position = Odom.pose.pose.position

        q = Odom.pose.pose.orientation
        orientation = np.arctan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

        if self.Init:
            #The initial data is stored to by subtracted to all the other values as we want to start at position (0,0) and orientation 0
            self.Init = False
            self.Init_ang = orientation
            self.globalAng = self.Init_ang
            Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],[-np.sin(self.Init_ang), np.cos(self.Init_ang)]])        
            self.Init_pos.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y
            self.Init_pos.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y
            self.Init_pos.z = position.z

        Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],[-np.sin(self.Init_ang), np.cos(self.Init_ang)]])        

        #We subtract the initial values
        self.globalPos.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y - self.Init_pos.x
        self.globalPos.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y - self.Init_pos.y
        self.globalAng = orientation - self.Init_ang

        self.globalAng = np.arctan2(np.sin(self.globalAng),np.cos(self.globalAng))

def main():
    rospy.init_node('go_To_Goal', anonymous=True)
    process()
    try:
        rospy.spin()
    except KeyboardInterrupt:
       print("Shutting down ROS")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
