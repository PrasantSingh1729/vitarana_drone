#!/usr/bin/env python

# Importing the required libraries

from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
import rospy
import time
import tf
import math



class Edrone_pos():
    """docstring for Edrone"""
    def __init__(self,LAT,LONG,ALT):
        rospy.init_node('position_controller')  # initializing ros node with name drone_control





        # The setpoint of longitude, latitude and altitude 
        self.set_longitude = LONG
        self.set_latitude = LAT
        self.set_altitude = ALT

        # Orientation of drone to attitude controller by edrone_cmd.msg
        self.orientation_cmd = edrone_cmd()
        self.orientation_cmd.rcRoll = 1500.0
        self.orientation_cmd.rcPitch = 1500.0
        self.orientation_cmd.rcYaw = 1500.0
        self.orientation_cmd.rcThrottle = 8.0


        # initial setting of Kp, Kd and ki 
        self.Kp = [5000,6027]
        self.Ki = [0,0]
        self.Kd = [5000,6027]

        #maximum and minimum value for Roll, Pitch and Yaw
        self.min_values = [1000, 1000, 1000]
        self.max_values = [2000, 2000, 2000]  

        # -----------------------Add other required variables for pid here ----------------------------------------------

        self.integral = [0.0, 0.0]    
        self.prev_values_error = [0.0, 0.0]
        self.out_long = 0
        self.out_lat = 0   
        self.latitude = 0
        self.longitude = 0
        self.altitude = 0.0
        self.lat_error = 0.0
        self.long_error = 0.0        
        self.zero_error = 0.0
        self.sample_time = 0.01  # in seconds
        self.time = 0.0 # use to calculate the total runtime of script

        # ------------------------Publishers-----------------------------------------------------
        self.lat_error_pub = rospy.Publisher('/lat_error',Float32,queue_size=1)
        self.long_error_pub = rospy.Publisher('/long_error',Float32,queue_size=1)
        self.orientation_pub = rospy.Publisher('/drone_command',edrone_cmd,queue_size=1)
        self.zero_error_pub = rospy.Publisher('/zero_error',Float32,queue_size= 1)


        # -----------------------------------------------------------------------------------------------------------

        # -------------------------Subscribers----------------------------------------------------
        rospy.Subscriber('/edrone/gps',NavSatFix,self.get_location)
        # we removed PID TUNING nodes subcription after tuning  
        
        # ------------------------------------------------------------------------------------------------------------


    def get_location(self, msg):
        self.altitude = msg.altitude
        self.latitude = msg.latitude
        self.longitude = msg.longitude

# Function to take input 
    def set_position(self,lat,lon,alt):
        self.set_altitude = alt
        self.set_latitude = lat
        self.set_longitude = lon



    def pos_pid(self):
        self.time = self.time + self.sample_time
        # -----------------------------Write the PID algorithm here--------------------------------------------------------------

        # defining local vairialble used in equation
        derivative=[0.0,0.0] 
        output=[0.0,0.0]
        error=[0.0,0.0]
        error[0] = 10000*(self.set_latitude - self.latitude)    # amplified latitude error
        error[1] = 10000*(self.set_longitude - self.longitude)  # amplified longitude error
        
        # PID equations
        self.integral[0] +=  error[0] * self.sample_time
        derivative[0] = (error[0] - self.prev_values_error[0]) / self.sample_time
        output[0] = (self.Kp[0] * error[0] +  self.Ki[0] * self.integral[0] + self.Kd[0] * derivative[0])
        
        self.integral[1] += error[1] * self.sample_time
        derivative[1] = (error[1] - self.prev_values_error[1]) / self.sample_time
        output[1] = (self.Kp[1] * error[1] + self.Ki[1] * self.integral[1] + self.Kd[1] * derivative[1])

        # Storing Previous error in Class variable
        self.prev_values_error[0]=error[0]
        self.prev_values_error[1]=error[1]


        self.lat_error = self.prev_values_error[0]
        self.long_error = self.prev_values_error[1]

        # to plotjuggler
        self.out_lat =  output[0]
        self.out_long =  output[1]


        # Setting oriention from equation
        self.orientation_cmd.rcPitch = 1500 + (self.out_long)
        self.orientation_cmd.rcRoll = 1500 + (self.out_lat)  #+ self.out_long         #self.out_lat
        self.orientation_cmd.rcYaw = 1500
        self.orientation_cmd.rcThrottle = self.set_altitude
        
        # condition input to motor
        if self.orientation_cmd.rcPitch > self.max_values[0]:
            self.orientation_cmd.rcPitch = self.max_values[0]

        if self.orientation_cmd.rcRoll > self.max_values[1]:
            self.orientation_cmd.rcRoll = self.max_values[1]

        if self.orientation_cmd.rcYaw > self.max_values[2]:
            self.orientation_cmd.rcYaw = self.max_values[2]



        if self.orientation_cmd.rcPitch < self.min_values[0]:
            self.orientation_cmd.rcPitch = self.min_values[0]

        if self.orientation_cmd.rcRoll < self.min_values[1]:
            self.orientation_cmd.rcRoll = self.min_values[1]
        if self.orientation_cmd.rcYaw < self.min_values[2]:
            self.orientation_cmd.rcYaw = self.min_values[2]


        self.orientation_pub.publish(self.orientation_cmd)
        self.lat_error_pub.publish(self.lat_error)
        self.long_error_pub.publish(self.long_error)
        self.zero_error_pub.publish(self.zero_error)



        # Debagging--------------------------------------------------------------------------------------------------
        
        # info = "\n"+str(self.orientation_cmd.rcPitch)+" "+str(self.orientation_cmd.rcRoll)+" "+str(self.orientation_cmd.rcYaw)+" "+str(self.orientation_cmd.rcThrottle)
        # errorinfo = "\n"+str(self.lat_error)+" "+str(self.long_error)
        # pidinfo = "\nPid Lat: "+str(self.Kp[0])+" "+str(self.Kd[0])+"\nPid Long: "+str(self.Kp[1])+" "+str(self.Kd[1])
        # rospy.loginfo(info+errorinfo+pidinfo+"\nTime: "+str(self.time))
        # rospy.loginfo("\n"+str(derivative[0]))
        # if (self.latitude < 19.0000486874 and self.latitude > 19.0000406534) and (self.altitude>0.29 and self.altitude<0.32) :
        #     self.orientation_cmd.rcPitch = 1500
        #     self.orientation_cmd.rcRoll =1500
        #     self.orientation_cmd.rcYaw=1500


        


        #debagging
        # #rospy.loginfo()
        # info = "\n"+str(self.orientation_cmd.rcPitch)+" "+str(self.orientation_cmd.rcRoll)+" "+str(self.orientation_cmd.rcYaw)+" "+str(self.orientation_cmd.rcThrottle)
        # errorinfo = "\n"+str(self.lat_error)+" "+str(self.long_error)
        # pidinfo = "\nPid Lat: "+str(self.Kp[0])+" "+str(self.Kd[0])+"\nPid Long: "+str(self.Kp[1])+" "+str(self.Kd[1])
        # rospy.loginfo(info+errorinfo+pidinfo)
        # rospy.loginfo(self.longitude)
        # rospy.loginfo(self.latitude)
        # errorinfo = "\nError: "+str(error[0]/10000)+" "+str(error[1]/1000)
        # rospy.loginfo(errorinfo)
        # # rospy.loginfo(self.set_altitude)


        #
        #
        #
        #
        #
        #
        #
        # ------------------------------------------------------------------------------------------------------------------------
        #publising orientation to attitude controller and plotjuggler




if __name__ == '__main__':

    e_drone_pos = Edrone_pos(19.000, 72.000, 3.0)
    r = rospy.Rate(int(math.floor(1/e_drone_pos.sample_time)))  # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
    try:
        while not rospy.is_shutdown():
            if(e_drone_pos.time>3 and e_drone_pos.time<15):
                e_drone_pos.set_position(19.0000451704,72.0,3) # changing latitude after drone attain stablity 
            if(e_drone_pos.time>15):
                e_drone_pos.set_position(19.0000451704,72.0,0.30) # changing altitude after drone attain stablity on correct position 
            e_drone_pos.pos_pid()
            r.sleep()
    except rospy.ROSInterruptException:
        pass

