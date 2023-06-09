"""
SlamFilter class to filter odometry messages. Stores the last num_avg_readings odometry messages
"""
from typing import List

import tf
import rospy
import numpy as np
from nav_msgs.msg import Odometry


class SlamFilter:
    """
    Class to filter odometry messages. Stores the last num_avg_readings odometry messages
    and averages them to get a more accurate odometry message. Also checks that the distance
    between the current odometry message and the previous odometry message is not too large.
    It publishes the filtered odometry message to filtered_odom_topic

    Parameters
    ----------
    odom_topic : str
        Topic to subscribe to for odometry messages
    filtered_odom_topic : str
        Topic to publish filtered odometry messages to
    max_move_dist : float
        Maximum distance between current odometry message and previous odometry message
    num_avg_readings : int
        Number of odometry messages to average
    """

    def __init__(
        self,
        odom_topic: str,
        filtered_odom_topic: str,
        max_move_dist: float,
        num_avg_readings: int,
    ):
        self.max_move_dist = max_move_dist
        self.num_avg_readings = num_avg_readings
        rospy.Subscriber(odom_topic, Odometry, self.odom_callback)
        self.filtered_pub = rospy.Publisher(
            filtered_odom_topic, Odometry, queue_size=10
        )

        self.odometry = Odometry()
        self.odometry.header.stamp = rospy.Time.now()
        self.odometry.header.frame_id = "map"

        self.prev_readings: List[Odometry] = []

    def odom_callback(self, odom: Odometry) -> None:
        """
        Callback for odometry messages. Stores the last num_avg_readings odometry messages
        Publishes the average odometry message to filtered_odom_topic

        Parameters
        ----------
        odom : Odometry
            Odometry message to store
        """
        distance = self.dist_to_odom(odom)
        if distance > self.max_move_dist:
            rospy.logwarn(
                "Odometry distance is too large, ignoring message. Distance: %f",
                distance,
            )
            return
        self.prev_readings.append(odom)
        if len(self.prev_readings) > self.num_avg_readings:
            self.prev_readings.pop(0)

        self.odometry = self.average_odom()
        self.filtered_pub.publish(self.odometry)

    def average_odom(self) -> Odometry:
        """
        Averages the last num_avg_readings odometry messages

        Returns
        -------
        Odometry
            Averaged odometry message
        """
        avg_odom = Odometry()
        avg_odom.header.stamp = rospy.Time.now()
        avg_odom.header.frame_id = "map"
        avg_odom.pose.pose.position.x = np.mean(
            [odom.pose.pose.position.x for odom in self.prev_readings]
        )
        avg_odom.pose.pose.position.y = np.mean(
            [odom.pose.pose.position.y for odom in self.prev_readings]
        )
        avg_odom.pose.pose.position.z = np.mean(
            [odom.pose.pose.position.z for odom in self.prev_readings]
        )
        avg_odom.pose.pose.orientation.x = np.mean(
            [odom.pose.pose.orientation.x for odom in self.prev_readings]
        )
        avg_odom.pose.pose.orientation.y = np.mean(
            [odom.pose.pose.orientation.y for odom in self.prev_readings]
        )
        avg_odom.pose.pose.orientation.z = np.mean(
            [odom.pose.pose.orientation.z for odom in self.prev_readings]
        )
        avg_odom.pose.pose.orientation.w = np.mean(
            [odom.pose.pose.orientation.w for odom in self.prev_readings]
        )
        return avg_odom

    def dist_to_odom(self, odom: Odometry) -> float:
        """
        Computes the distance between current self.odometry and odom
        Distance is a combination of l2 distance for position and quaternion

        Parameters
        ----------
        odom : Odometry
            Odometry message to compute distance to

        Returns
        -------
        float
            Distance between current self.odometry and odom
        """
        # Compute l2 distance
        curr_pos = [
            self.odometry.pose.pose.position.x,
            self.odometry.pose.pose.position.y,
            self.odometry.pose.pose.position.z,
        ]
        new_pos = [
            odom.pose.pose.position.x,
            odom.pose.pose.position.y,
            odom.pose.pose.position.z,
        ]
        l2_dist = np.linalg.norm(np.array(curr_pos) - np.array(new_pos))

        # curr_euler = tf.transformations.euler_from_quaternion(
        #     self.odometry.pose.pose.orientation
        # )
        # new_euler = tf.transformations.euler_from_quaternion(odom.pose.pose.orientation)
        euler_dist = 0

        return l2_dist.item() + euler_dist
