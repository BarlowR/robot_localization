import rospy
from std_msgs.msg import Header, String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Pose, PoseArray, Point, Quaternion
from tf.transformations import euler_from_quaternion

import tf
import tf2_ros

import numpy as np 
import matplotlib.pyplot as plt 
import matplotlib.patches as patches



class Neato(object):

    def __init__(self):
        self.delta_pose = None
        self.laser_scan = None
        self.laser_scan_update = None
        

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.previous_update_time = None
        self.current_update_time = None

        rospy.Subscriber('/scan', LaserScan, self.process_scan)

    def update(self):

        try: 
            if self.current_update_time is None: 
                self.current_update_time = self.tf_buffer.get_latest_common_time('odom', 'base_link')

            if (self.previous_update_time is None or (self.tf_buffer.get_latest_common_time('odom', 'base_link')-self.previous_update_time) > rospy.Duration(.1)):
                self.previous_update_time = self.current_update_time
                self.current_update_time = self.tf_buffer.get_latest_common_time('odom', 'base_link')
                
                self.laser_scan_update = self.laser_scan

                full_transform = self.tf_buffer.lookup_transform_full('base_link',
                                                                              self.previous_update_time,
                                                                              'base_link',
                                                                              self.current_update_time,
                                                                              'odom').transform
                yaw = euler_from_quaternion((full_transform.rotation.x,
                                      full_transform.rotation.y,
                                      full_transform.rotation.z,
                                      full_transform.rotation.w))[2]
                self.delta_pose = (full_transform.translation.x, full_transform.translation.y, yaw)

        except Exception as ex:
            print(ex)
            pass

    def process_scan(self, msg):
        self.laser_scan = msg.ranges

if __name__ == '__main__':
    rospy.init_node('neato')
    neato = Neato()
    r = rospy.Rate(5)

    while not rospy.is_shutdown():
        neato.update()
        print(neato.delta_pose)
        r.sleep()