# !/usr/bin/env python 

import rospy 
import serial 
import sys
from imu_driver.msg import imu_msg #import ROS custom message "imu_msg" present in package "imu_driver" 
import numpy as np
from math import pi

# Converts Euler angles to quaternion.
def get_quaternion_from_euler(yaw, pitch, roll): #degre to rad n check formula
  
# Input:
#      roll: rotation around x-axis
#      pitch: rotation around y-axis
#      yaw: yaw (rotation around z-axis)

# Output:return qx, qy, qz, qw: orientation in quaternion [x,y,z,w] format

    qx = np.sin(float(roll)/2) * np.cos(float(pitch)/2) * np.cos(float(yaw)/2) - np.cos(float(roll)/2) * np.sin(float(pitch)/2) * np.sin(float(yaw)/2)
    qy = np.cos(float(roll)/2) * np.sin(float(pitch)/2) * np.cos(float(yaw)/2) + np.sin(float(roll)/2) * np.cos(float(pitch)/2) * np.sin(float(yaw)/2)
    qz = np.cos(float(roll)/2) * np.cos(float(pitch)/2) * np.sin(float(yaw)/2) - np.sin(float(roll)/2) * np.sin(float(pitch)/2) * np.cos(float(yaw)/2)
    qw = np.cos(float(roll)/2) * np.cos(float(pitch)/2) * np.cos(float(yaw)/2) + np.sin(float(roll)/2) * np.sin(float(pitch)/2) * np.sin(float(yaw)/2)

    return [qx, qy, qz, qw]

# Sample data for parsing
# n = "$VNYMR,+164.618,+022.062,-003.757,-00.3611,-00.0797,+00.2916,+03.553,+00.595,-08.826,+00.004000,-00.000843,+00.000141*64"

if __name__ == '__main__':
    rospy.init_node('imuData', anonymous=True) #imuData = ROS node
    serial_port = sys.argv[1]
    serial_baud_rate = 115200
    port = serial.Serial(serial_port,serial_baud_rate,timeout = 1)
    port.write(b"$VNWRG,07,40*XX")
    port.write(b"$VNWRG,06,14*XX")
    pub = rospy.Publisher('imu',imu_msg, queue_size = 10) #imu = ROS topic
    IMU_msg = imu_msg() #ROS custom message calling
    IMU_msg.Header.frame_id = "IMU1_Frame"
    data_count = 0

    try:
        while not rospy.is_shutdown(): 
            init_line = port.readline()
                        
           # print('init_line',init_line) #Testing 
            data = init_line.decode('utf-8')
            splitline = data.split(',')
           
            if '$VNYMR' in splitline[0]:
                
                yaw_in_rad = (float(splitline[1]))*(pi/180)
                pitch_in_rad = (float(splitline[2]))*(pi/180)
                roll_in_rad = (float(splitline[3]))*(pi/180)
                quaternion_values = get_quaternion_from_euler(yaw_in_rad, pitch_in_rad , roll_in_rad)
                # print(quaternion_values) #Testing to print quaternion_values
                time = rospy.get_rostime();
                IMU_msg.Header.stamp.secs = time.secs
                IMU_msg.Header.stamp.nsecs = time.nsecs
                IMU_msg.IMU.orientation.x = quaternion_values[0]
                IMU_msg.IMU.orientation.y = quaternion_values[1]
                IMU_msg.IMU.orientation.z = quaternion_values[2]
                IMU_msg.IMU.orientation.w = quaternion_values[3]
                
                IMU_msg.MagField.magnetic_field.x = float(splitline[4])
                IMU_msg.MagField.magnetic_field.y = float(splitline[5])
                IMU_msg.MagField.magnetic_field.z = float(splitline[6])
                IMU_msg.IMU.linear_acceleration.x = float(splitline[7])
                IMU_msg.IMU.linear_acceleration.y = float(splitline[8])
                IMU_msg.IMU.linear_acceleration.z = float(splitline[9])
                IMU_msg.IMU.angular_velocity.x = float(splitline[10])
                IMU_msg.IMU.angular_velocity.y = float(splitline[11])
                IMU_msg.IMU.angular_velocity.z = float(splitline[12][:-5])
                #print(IMU_msg)
                pub.publish(IMU_msg)

    except rospy.ROSInterruptException:
        port.close()
        
else:
    rospy.ROSInterruptException;
    port.close() 
