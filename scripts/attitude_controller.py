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



class Edrone():
    """docstring for Edrone"""
    def __init__(self):
        rospy.init_node('attitude_controller', anonymous= True)  # initializing ros node with name drone_control

        # This corresponds to  current orientation of eDrone in quaternion format. This value must be updated each time in your imu callback
        # [x,y,z,w]
        self.drone_orientation_quaternion = [0.0, 0.0, 0.0, 0.0]

        # This corresponds to  current orientation of eDrone converted in euler angles form.
        # [r,p,y]
        self.drone_orientation_euler = [0.0, 0.0, 0.0]

        # This is the setpoint that will be received from the drone_command in the range from 1000 to 2000
        # [r_setpoint, p_setpoint, y_setpoint]
        self.setpoint_cmd = [1500.0, 1500.0, 1500.0]

        # The setpoint of orientation in euler angles at which you want to stabilize the drone
        # [r_setpoint, p_psetpoint, y_setpoint]
        self.setpoint_euler = [0.0, 0.0, 0.0]
        self.set_altitude = 4.0 # in meter hover distance from land
        self.altitude = 0.0
        self.set_throttle = 509

        # Declaring pwm_cmd of message type prop_speed and initializing values
        
        self.pwm_cmd = prop_speed()
        self.pwm_cmd.prop1 = 0.0
        self.pwm_cmd.prop2 = 0.0
        self.pwm_cmd.prop3 = 0.0
        self.pwm_cmd.prop4 = 0.0

        # initial setting of Kp, Kd and ki for [roll, pitch, yaw]. eg: self.Kp[2] corresponds to Kp value in yaw axis
        # after tuning and computing corresponding PID parameters, change the parameters
        self.Kp = [212.22, 336, 5000.0, 51.2]
        self.Ki = [0.0, 0.0, 0.0, 0.0]
        self.Kd = [39.3, 96, 502.0, 91.5]
        
        self.integral = [0.0, 0.0, 0.0, 0.0]

    
        self.prev_values_error = [0.0, 0.0, 0.0, 0.0]
        self.max_values = [1024, 1024, 1024, 1024]
        self.min_values = [0, 0, 0, 0] 

        self.out_pitch = 0
        self.out_roll = 0
        self.out_yaw = 0    

        self.roll_error = 0.0
        self.pitch_error = 0.0        
        self.yaw_error = 0.0
        self.zero_error = 0.0

        self.PI= 3.14159265359

        
        # # This is the sample time in which we need to run pid. Stimulation step time is 50 ms
        self.sample_time = 0.06  

        # Publishing /edrone/pwm, /roll_error, /pitch_error, /yaw_error, /zero_error, /z_error
        self.pwm_pub = rospy.Publisher('/edrone/pwm', prop_speed, queue_size=1)
        self.roll_error_pub = rospy.Publisher('/roll_error',Float32,queue_size=1)
        self.pitch_error_pub = rospy.Publisher('/pitch_error',Float32,queue_size=1)
        self.yaw_error_pub = rospy.Publisher('/yaw_error',Float32,queue_size=1)
        self.zero_error_pub = rospy.Publisher('/zero_error',Float32,queue_size= 1)
        self.altitude_error_pub = rospy.Publisher('/z_error',Float32,queue_size=1)

        # Subscribing to /drone_command, imu/data, /pid_tuning_roll, /pid_tuning_pitch, /pid_tuning_yaw, pid_tuning altitue.
        rospy.Subscriber('/drone_command', edrone_cmd, self.drone_command_callback)
        rospy.Subscriber('/edrone/imu/data', Imu, self.imu_callback)
        rospy.Subscriber('/edrone/gps',NavSatFix,self.get_altitude)
        rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)   
        rospy.Subscriber('/pid_tuning_pitch',PidTune,self.pitch_set_pid)
        rospy.Subscriber('/pid_tuning_yaw',PidTune,self.yaw_set_pid)
        rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
       
    # Imu callback function
    # The function gets executed each time when imu publishes /edrone/imu/data

    def imu_callback(self, msg):

        self.drone_orientation_quaternion[0] = msg.orientation.x
        self.drone_orientation_quaternion[1] = msg.orientation.y
        self.drone_orientation_quaternion[2] = msg.orientation.z
        self.drone_orientation_quaternion[3] = msg.orientation.w 

        
    def drone_command_callback(self, msg):
        self.setpoint_cmd[0] = msg.rcPitch
        self.setpoint_cmd[1] = msg.rcRoll
        self.setpoint_cmd[2] = msg.rcYaw
        self.set_altitude = msg.rcThrottle

        # ---------------------------------------------------------------------------------------------------------------

    # Callback function for /pid_tuning_roll
    # This function gets executed each time when /tune_pid publishes /pid_tuning_roll
    def roll_set_pid(self, roll):
        self.Kp[1] = roll.Kp * 0.2  # This is just for an example. You can change the ratio/fraction value accordingly
        self.Ki[1] = roll.Ki * 0.0028
        self.Kd[1] = roll.Kd * 0.2
    def pitch_set_pid(self, pitch):
        self.Kp[0] = pitch.Kp * 0.06  
        self.Ki[0] = pitch.Ki * 0.008
        self.Kd[0] = pitch.Kd * 0.3
    def yaw_set_pid(self, yaw):
        self.Kp[2] = yaw.Kp * 1  
        self.Ki[2] = yaw.Ki * 0.008
        self.Kd[2] = yaw.Kd * 1
    
    def altitude_set_pid(self, altitude):
        self.Kp[3] = altitude.Kp * 0.8  
        self.Ki[3] = altitude.Ki * 0.008
        self.Kd[3] = altitude.Kd * 0.5

    def get_altitude(self, msg):
        self.altitude = msg.altitude


    
    def pid(self):
        

        # Converting quaternion to euler angles
        (self.drone_orientation_euler[0], self.drone_orientation_euler[1], self.drone_orientation_euler[2]) = tf.transformations.euler_from_quaternion([self.drone_orientation_quaternion[0], self.drone_orientation_quaternion[1], self.drone_orientation_quaternion[2], self.drone_orientation_quaternion[3]])

        # Convertng the range from 1000 to 2000 in the range of -10 degree to 10 degree for roll axis
        self.setpoint_euler[0] = (self.PI/18)*(self.setpoint_cmd[0] * 0.002 - 3)
        self.setpoint_euler[1] = (self.PI/18)*(self.setpoint_cmd[1] * 0.002 - 3)
        self.setpoint_euler[2] = (self.PI)*(self.setpoint_cmd[2] * 0.002 - 3)

        #Calculating error.
        error=[0.0,0.0,0.0,0.0]
        error[0] = self.setpoint_euler[0] - self.drone_orientation_euler[0]
        error[1] = self.setpoint_euler[1] - self.drone_orientation_euler[1]
        error[2] = self.setpoint_euler[2] - self.drone_orientation_euler[2]
        error[3] = self.set_altitude - self.altitude
        
      
    
     
        derivative=[0.0,0.0,0.0,0.0]
        output=[0.0,0.0,0.0,0.0]
        
        # Calculation output for roll, pitch, yaw and altitude. Using PID for the same.

        self.integral[0] +=  error[0] * self.sample_time
        derivative[0] = (error[0] - self.prev_values_error[0]) / self.sample_time
        output[0] = self.Kp[0] * error[0] +  self.Ki[0] * self.integral[0] + self.Kd[0] * derivative[0]
        
        self.integral[1] += error[1] * self.sample_time
        derivative[1] = (error[1] - self.prev_values_error[1]) / self.sample_time
        output[1] = self.Kp[1] * error[1] + self.Ki[1] * self.integral[1] + self.Kd[1] * derivative[1]

        self.integral[2] += error[2] * self.sample_time
        derivative[2] = (error[2] - self.prev_values_error[2]) / self.sample_time
        output[2] = self.Kp[2] * error[2] + self.Ki[2] * self.integral[2] + self.Kd[2] * derivative[2]
        
        self.integral[3] += error[3] * self.sample_time
        derivative[3] = (error[3] - self.prev_values_error[3]) / self.sample_time
        output[3] = self.Kp[3] * error[3] + self.Ki[3] * self.integral[3] + self.Kd[3] * derivative[3]
         
        # Updateing previous errors
       
        self.prev_values_error[0]=error[0]
        self.prev_values_error[1]=error[1]
        self.prev_values_error[2]=error[2]
        self.prev_values_error[3]=error[3]

        # Sending errors to plotjuggler so that we can use it for PID tunning.

        self.roll_error = self.prev_values_error[1]
        self.pitch_error = self.prev_values_error[0]
        self.yaw_error = self.prev_values_error[2]  
        self.z_error = self.prev_values_error[3]

        self.out_roll =  output[1]
        self.out_pitch =  output[0]
        self.out_yaw = output[2]
        self.out_altitude = output[3]

        # Giving required values to each propeller.

        self.pwm_cmd.prop1 = 509 + self.out_altitude - self.out_roll + self.out_pitch - self.out_yaw
        self.pwm_cmd.prop2 = 509 + self.out_altitude - self.out_roll - self.out_pitch + self.out_yaw
        self.pwm_cmd.prop3 = 509 + self.out_altitude + self.out_roll - self.out_pitch - self.out_yaw
        self.pwm_cmd.prop4 = 509 + self.out_altitude + self.out_roll + self.out_pitch + self.out_yaw

        
        #sometime it may happen propellers get less or more value than their capacity.
        #  So limiting them to min and max values.

        if self.pwm_cmd.prop1 > self.max_values[0]:
            self.pwm_cmd.prop1 = self.max_values[0]

        if self.pwm_cmd.prop2 > self.max_values[1]:
            self.pwm_cmd.prop2 = self.max_values[1]

        if self.pwm_cmd.prop3 > self.max_values[2]:
            self.pwm_cmd.prop3 = self.max_values[2]

        if self.pwm_cmd.prop4 > self.max_values[3]:
            self.pwm_cmd.prop4 = self.max_values[3]



        if self.pwm_cmd.prop1 < self.min_values[0]:
            self.pwm_cmd.prop1 = self.min_values[0]

        if self.pwm_cmd.prop2 < self.min_values[1]:
            self.pwm_cmd.prop2 = self.min_values[1]

        if self.pwm_cmd.prop3 < self.min_values[2]:
            self.pwm_cmd.prop3 = self.min_values[2]

        if self.pwm_cmd.prop4 < self.min_values[3]:
            self.pwm_cmd.prop4 = self.min_values[3]

        """ The below prameters were used for debugging and knowing performance of drone in detail."""

        # #debagging
        # droneorientation = "\nDrone Orientation [ "+str(self.drone_orientation_euler[0])+", "+str(self.drone_orientation_euler[1])+", "+str(self.drone_orientation_euler[2])+" ]"
        # errorinfo = "\nerror [ "+str(error[0])+", "+str(error[1])+", "+str(error[2])+", "+str(error[3])+" ]"
        # outinfo = "\nout [ "+str(output[0])+", "+str(output[1])+", "+str(output[2])+", "+str(output[3])+" ]"
        # propinfo = "\nMotor Speed [ "+str(self.pwm_cmd.prop1)+", "+str(self.pwm_cmd.prop2)+", "+str(self.pwm_cmd.prop3)+", "+str(self.pwm_cmd.prop4)+" ]"
        # rollinfo = "\nKpid Pitch [ "+str(self.Kp[0])+", "+str(self.Ki[0])+", "+str(self.Kd[0])+" ]"
        # pitchinfo = "\nKpid Roll [ "+str(self.Kp[1])+", "+str(self.Ki[1])+", "+str(self.Kd[1])+" ]"
        # yawinfo = "\nKpid Yaw [ "+str(self.Kp[2])+", "+str(self.Ki[2])+", "+str(self.Kd[2])+" ]"
        # altitudeinfo = "\nKpid Altitude [ "+str(self.Kp[3])+", "+str(self.Ki[3])+", "+str(self.Kd[3])+" ]"
        # integralinfo = "\nIntegral [ "+str(self.integral[0])+", "+str(self.integral[1])+", "+str(self.integral[2])+", "+str(self.integral[3])+" ] "
        # setinfo = "\nSet ["+str(self.setpoint_euler[0])+", "+str(self.setpoint_euler[1])+", "+str(self.setpoint_euler[2])+", "+str(self.set_altitude)+" ]"
        # rospy.loginfo(droneorientation+outinfo+propinfo+errorinfo+rollinfo+pitchinfo+yawinfo+altitudeinfo+integralinfo+setinfo)


        # Publishing     

        self.pwm_pub.publish(self.pwm_cmd)
        self.pitch_error_pub.publish(self.pitch_error)
        self.roll_error_pub.publish(self.roll_error)
        self.yaw_error_pub.publish(self.yaw_error)
        self.zero_error_pub.publish(self.zero_error)
        self.altitude_error_pub.publish(self.z_error)


if __name__ == '__main__':

    e_drone = Edrone()
    r = rospy.Rate(int(math.floor(1/e_drone.sample_time)))  
    try:
        while not rospy.is_shutdown():
            e_drone.pid()
            r.sleep()
    except rospy.ROSInterruptException:
        pass
