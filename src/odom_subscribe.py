#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class OdomSubscriber():

    def callback(self, topic_data: Odometry):
        # We're only interested in the pose part of the Odometry message,
        # so we'll extract this bit first:
        pose = topic_data.pose.pose
        # This contains information about both the "position" and "orientation"
        # of the robot, so let's extract those two parts out next:
        position = pose.position
        orientation = pose.orientation 
        # "position" data is provided in meters, so we don't need to do any
        # conversion on this, and can extract the relevant parts of this directly:
        pos_x = position.x
        pos_y = position.y
        pos_z = position.z
        # "orientation" data is in quaternions, so we need to convert this 
        # using the "euler_from_quaternion" function 
        # See here for further details:
        # https://tom-howard.github.io/ros/com2009/assignment1/part2/#euler_angs

        # Add your code here!
        orientation_x = topic_data.pose.pose.orientation.x
        orientation_y = topic_data.pose.pose.orientation.y
        orientation_z = topic_data.pose.pose.orientation.z
        orientation_w = topic_data.pose.pose.orientation.w

        (roll, pitch, yaw) = euler_from_quaternion([orientation_x, 
                                orientation_y, orientation_z, orientation_w],
                                'sxyz')

        # Here we print out the values that we're interested in:
        if self.counter > 10:
            self.counter = 0
            print(f"x = {pos_x:.3f} (m), y = {pos_y:.3f} (m), theta_z = {yaw:.3f} (radians)")
        else:
            self.counter += 1

    def __init__(self):
        node_name = "odom_subscriber" # a name for our node (we can call it anything we like)
        rospy.init_node(node_name, anonymous=True)
        # When setting up the subscriber, the "odom" topic needs to be specified
        # and the message type (Odometry) needs to be provided
        self.sub = rospy.Subscriber("odom", Odometry, self.callback)
        # an optional status message:
        rospy.loginfo(f"The '{node_name}' node is active...")
        
        self.counter = 0       # What's this bit for, do you think?

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    node = OdomSubscriber()
    node.main()