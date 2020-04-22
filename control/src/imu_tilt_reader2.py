#!/usr/bin/env python
from __future__ import division, print_function

import rospy
import tf.transformations as transformations
import tf2_ros

from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from geometry_msgs.msg import TransformStamped

import serial

from cobs import cobs
from struct import unpack, error
from math import degrees

device_location = "/dev/imu_sensor_bed"

def create_transform_for_angle(parent_name, child_name, angle):
    transform_stamped = TransformStamped()
    transform_stamped.header.stamp = rospy.Time.now()
    transform_stamped.header.frame_id = parent_name
    transform_stamped.child_frame_id = child_name
    zero_quaternion = transformations.quaternion_from_euler(0, angle, 0)
    transform_stamped.transform.rotation.x = zero_quaternion[0]
    transform_stamped.transform.rotation.y = zero_quaternion[1]
    transform_stamped.transform.rotation.z = zero_quaternion[2]
    transform_stamped.transform.rotation.w = zero_quaternion[3]
    return transform_stamped

def create_transform_for_height(parent_name, child_name, height):
    transform_stamped = TransformStamped()
    transform_stamped.header.stamp = rospy.Time.now()
    transform_stamped.header.frame_id = parent_name
    transform_stamped.child_frame_id = child_name
    zero_quaternion = transformations.quaternion_from_euler(0, 0, 0)
    transform_stamped.transform.translation.z = height
    transform_stamped.transform.rotation.x = zero_quaternion[0]
    transform_stamped.transform.rotation.y = zero_quaternion[1]
    transform_stamped.transform.rotation.z = zero_quaternion[2]
    transform_stamped.transform.rotation.w = zero_quaternion[3]
    return transform_stamped



class ArduinoIMU(object):
    def __init__(self):
        super(ArduinoIMU, self).__init__()
        # rospy.init_node("sensor_bed_imu")
        rospy.init_node("imu_reader2")
        # variables
        self.footprint_link = rospy.get_param("~footprint_link_name", default="base_footprint")
        self.parent_link = rospy.get_param("~parent_link_name", default="base_stabilized")
        self.child_link = rospy.get_param("~child_link_name", default="base_link")
        self.tilt = 0.0
        self.height_from_ground = rospy.get_param("ziva_height_from_ground", default=0.3)
        # publishers
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tilt_publisher = rospy.Publisher("temp/ziva/tilt", Float64, queue_size=1)
        self.heading_publisher = rospy.Publisher("temp/heading/current", Float64, queue_size=1)
        # subscribers


        while not False:
            try:
                self.run()
            except KeyboardInterrupt:
                # shutdown
                break
            except Exception as e:
                # serial or runtime error
                print(str(e))
                # rospy.logerr(str(e))


    def run(self):
        with serial.Serial(device_location, 115200) as port:
            print("device found at " + device_location)
            while not False:
                try:
                    data = port.read_until(bytes(bytearray([0])), 4*4+3)
                    # remove trailing 0
                    data = data[:-1]
                    # print( "read in data " + data)
                    decoded = cobs.decode(data)
                    quaternion = unpack("<ffff", decoded)
  		    print(quarternion)
 		    #euler = transformations.euler_from_quaternion(quarternion)
                    euler = [1,2,3]
		    #print(map(degrees, transformations.euler_from_quaternion(quaternion)))
		    #self.tilt = euler[1]
		    #euler = [1,2,3]
		    print(euler)
        	    #self.tilt_publisher.publish(self.tilt)
        	    #self.heading_publisher.publish(euler[2])

                except (cobs.DecodeError, error) as e:
                    # parsing error
                    print("parsing error: " + str(e))
                    # rospy.logerr("parsing error " + str(e))

	        except KeyboardInterrupt:
        	    # shutdown 
                    break
    	        except Exception as e:
        	    # serial or runtime error
        	    print(str(e))
        	    # rospy.logerr(str(e))



class PhidgetIMU(object):
    def __init__(self):
        super(PhidgetIMU, self).__init__()
        rospy.init_node("imu_reader")
        # variables
        self.footprint_link = rospy.get_param("~footprint_link_name", default="base_footprint")
        self.parent_link = rospy.get_param("~parent_link_name", default="base_stabilized")
        self.child_link = rospy.get_param("~child_link_name", default="base_link")
        self.tilt = 0.0
        self.height_from_ground = rospy.get_param("ziva_height_from_ground", default=0.3)
        # publishers
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tilt_publisher = rospy.Publisher("ziva/tilt", Float64, queue_size=1)
        self.heading_publisher = rospy.Publisher("heading/current", Float64, queue_size=1)
        # subscribers
        rospy.Subscriber("imu/data", Imu, self.on_imu_msg, queue_size=10)
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            # self.tf_broadcaster.sendTransform(create_transform_for_angle(self.parent_link, self.chil$
            # self.tf_broadcaster.sendTransform(create_transform_for_height(self.footprint_link, self.$
            rate.sleep()

    def on_imu_msg(self, imu_msg):
        quaternion = imu_msg.orientation
        # euler = transformations.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, qua$
        self.tilt = euler[1]
        self.tilt_publisher.publish(self.tilt)
        self.heading_publisher.publish(euler[2])


if __name__ == "__main__":
    # PhidgetIMU()
    ArduinoIMU()






