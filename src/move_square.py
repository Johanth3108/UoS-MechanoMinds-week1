import rospy

# import the Twist message for publishing velocity commands:
from geometry_msgs.msg import Twist

# import the Odometry message for subscribing to the odom topic:
from nav_msgs.msg import Odometry

# import the function to convert orientation from quaternions to angles:
from tf.transformations import euler_from_quaternion

# import some useful mathematical operations (and pi), which you may find useful:
from math import sqrt, pow, pi

class Square():
    def callback_function(self, topic_data: Odometry):
        # obtain relevant topic data: pose (position and orientation):
        pose = topic_data.pose.pose
        position = pose.position
        orientation = pose.orientation

        # obtain the robot's position co-ords:
        pos_x = position.x
        pos_y = position.y

        # convert orientation co-ords to roll, pitch & yaw 
        # (theta_x, theta_y, theta_z):
        (roll, pitch, yaw) = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w], "sxyz"
        )

        # We're only interested in x, y and theta_z
        # so assign these to class variables (so that we can
        # access them elsewhere within our Square() class):
        self.x = pos_x
        self.y = pos_y
        self.theta_z = yaw

        # If this is the first time that the callback_function has run
        # (e.g. the first time a message has been received), then
        # obtain a "reference position" (used to determine how far
        # the robot has moved during its current operation)
        if self.startup:
            # don't initialise again:
            self.startup = False
            # set the reference position:
            self.x0 = self.x
            self.y0 = self.y
            self.theta_z0 = self.theta_z

    def __init__(self):
        node_name = "move_square"
        # a flag if this node has just been launched
        self.startup = True

        # This might be useful in the main_loop() (to switch between 
        # turning and moving forwards)
        self.turn = False

        # setup a '/cmd_vel' publisher and an '/odom' subscriber:
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.sub = rospy.Subscriber("odom", Odometry, self.callback_function)

        rospy.init_node(node_name, anonymous=True)
        self.rate = rospy.Rate(10)  # hz

        # define the robot pose variables and initialise them to zero:
        # variables for the robot's "current position":
        self.x = 0.0
        self.y = 0.0
        self.theta_z = 0.0
        # variables for a "reference position":
        self.x0 = 0.0
        self.y0 = 0.0
        self.theta_z0 = 0.0

        # define a Twist message instance, to set robot velocities
        self.vel = Twist()

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

        rospy.loginfo(f"the {node_name} node has been initialised...")

    def shutdownhook(self):
        # publish an empty twist message to stop the robot
        # (by default all velocities will be zero):
        self.pub.publish(Twist())
        self.ctrl_c = True

    def main_loop(self):
    # Define the side length of the square (1 meter):
        side_length = 1.0

        
        # Set the linear velocity (forward speed) to 0.1 m/s:
        self.vel.linear.x = 0.1
        self.vel.linear.y = 0.0
        self.vel.linear.z = 0.0

        # Set the angular velocity (rotation speed) to 0.0 rad/s:
        self.vel.angular.x = 0.0
        self.vel.angular.y = 0.0
        self.vel.angular.z = 0.0
        # Move forward for the side length:
        distance_moved = 0.0
        self.x0 = self.x  # Reset reference position for each side
        self.y0 = self.y
        while distance_moved < side_length:
            self.pub.publish(self.vel)
            self.rate.sleep()
            # Calculate the distance moved using the difference
            # between current position and reference position:
            distance_moved = sqrt(pow(self.x - self.x0, 2) + pow(self.y - self.y0, 2))
            print(distance_moved)


        # Stop the robot:
        self.vel.linear.x = 0.0
        self.vel.linear.y = 0.0
        self.vel.linear.z = 0.0
        self.vel.angular.z = 0.0
        self.pub.publish(self.vel)
        rospy.sleep(2.)

        # Rotate the robot by 90 degrees for the next side:
        initial_yaw = self.theta_z
        target_yaw = initial_yaw + (pi / 2)  # 90 degrees in radians

        # Keep rotating until the desired angle is reached:
        while abs(self.theta_z - initial_yaw) < abs(target_yaw - initial_yaw):
            self.vel.angular.z = 0.3  # Angular velocity for rotation
            self.pub.publish(self.vel)
            # print("turn")
            self.rate.sleep()

        # Stop the robot after reaching the desired angle:
        self.vel.angular.z = 0.0
        self.pub.publish(self.vel)
        self.rate.sleep()


if __name__ == "__main__":
    node = Square()
    try:
        for _ in range(4):
            node.main_loop()
    except rospy.ROSInterruptException:
        pass