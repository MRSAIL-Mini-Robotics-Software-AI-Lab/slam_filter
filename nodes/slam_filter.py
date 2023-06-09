import rospy
from SlamFilter import SlamFilter


def main():
    """
    This is the main function that is executed upon running the node.
    """
    rospy.init_node("slam_filter")

    odom_topic = rospy.get_param("/odom_topic")
    filtered_odom_topic = rospy.get_param("/filtered_odom_topic")
    max_move_dist = rospy.get_param("/max_move_dist")
    num_avg_readings = rospy.get_param("/num_avg_readings")

    slam_filter = SlamFilter(
        odom_topic, filtered_odom_topic, max_move_dist, num_avg_readings
    )

    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
