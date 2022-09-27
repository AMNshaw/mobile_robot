import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool

class TopicTool:
    def __init__(self):
        self.host_mocap = PoseStamped()
        self.mocap_pos_pub = rospy.Publisher("/mavros/vision_pose/pose", PoseStamped, queue_size=2)
        self.host_sub = rospy.Subscriber("/vrpn_client_node/rigidbody7", PoseStamped, self.pos_cb, queue_size=10)
        self.rate = rospy.Rate(30)

    def pos_cb(self, data):
        self.host_mocap = data

    def sensor_fusion(self):
        """
        publish pose data from OptiTrack to flight control board which will
        automatically do sensor fusion
        """
        rospy.loginfo("odom: {:.3f} {:.3f} {:.3f}".format(self.host_mocap.pose.position.x,
                                                          self.host_mocap.pose.position.y,
                                                          self.host_mocap.pose.position.z))
        self.mocap_pos_pub.publish(self.host_mocap)
        self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("topic_tool")
    topic_tool = TopicTool()

    while not rospy.is_shutdown():
        topic_tool.sensor_fusion()
