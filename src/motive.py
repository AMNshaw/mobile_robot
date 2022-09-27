import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Point
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, SetModeRequest
from mavros_msgs.srv import CommandBool, CommandBoolRequest

def quaternion_to_euler(q):
    """
    Transform quaternion to euler angle.
    The ordering of quaternion elements is [x, y, z, w]
    Inputs:
        q: List
    Returns:
        roll : Float
        pitch: Float
        yaw  : Float
    """
    sinr_cosp = 2 * (q[3] * q[0] + q[1] * q[2])
    cosr_cosp = 1 - 2 * (q[0] * q[0] + q[1] * q[1])
    roll = math.atan2(sinr_cosp, cosr_cosp)

    pitch = 0
    sinp = 2 * (q[3] * q[1] - q[2] * q[0])
    if math.fabs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2 * (q[3] * q[2] + q[0] * q[1])
    cosy_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2])
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

class Motive:
    def __init__(self):
        self.init = False

        self.KPx = 1
        self.KPy = 1
        self.KPz = 1.2
        self.KPyaw = 1

        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        self.vir = np.zeros(4, dtype=np.double)

        self.current_state = State()
        self.host_mocap = PoseStamped()
        self.initial_pose = PoseStamped()
        self.vs = TwistStamped()

        self.offb_set_mode = SetModeRequest()
        self.arm_cmd = CommandBoolRequest()

        self.state_sub = rospy.Subscriber("/mavros/state", State, self.state_cb, queue_size=10)
        self.pos_sub = rospy.Subscriber("/vrpn_client_node/rigidbody7/pose", PoseStamped, self.pos_cb, queue_size=10)
        self.local_vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=2)

        self.arming_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)

        self.rate = rospy.Rate(50)

    def initialize(self):
        while ((not rospy.is_shutdown()) and self.current_state.connected):
            self.rate.sleep()

        self.vir = np.array([0, 0, 0.5, 0])
        self.vs = TwistStamped()

        for i in range(100):
            self.local_vel_pub.publish(self.vs)
            self.vir = np.array([0, 0, 0.5, self.yaw])
            rospy.loginfo("initial_pose: {:.3f}, {:.3f}, {:.3f}".format(self.initial_pose.pose.position.x,
                                                                        self.initial_pose.pose.position.y,
                                                                        self.initial_pose.pose.position.z))
            self.rate.sleep()

        self.offb_set_mode.custom_mode = "OFFBOARD"
        self.arm_cmd.value = True

    def state_cb(self, data):
        self.current_state = data

    def pos_cb(self, data):
        self.host_mocap = data

        if (self.init == False):
            self.initial_pose = self.host_mocap
            self.init = True

        self.host_mocap.pose.position.x -= self.initial_pose.pose.position.x
        self.host_mocap.pose.position.y -= self.initial_pose.pose.position.y
        self.host_mocap.pose.position.z -= self.initial_pose.pose.position.z

        quaternion = [self.host_mocap.pose.orientation.x,
                      self.host_mocap.pose.orientation.y,
                      self.host_mocap.pose.orientation.z,
                      self.host_mocap.pose.orientation.w]
        euler = quaternion_to_euler(quaternion)

        self.roll = euler[0]
        self.pitch = euler[1]
        self.yaw = euler[2]

    def follow(self):
        errx = self.vir[0] - self.host_mocap.pose.position.x
        erry = self.vir[1] - self.host_mocap.pose.position.y
        errz = self.vir[2] - self.host_mocap.pose.position.z
        err_yaw = self.vir[3] - self.yaw

        if err_yaw > np.pi:
            err_yaw = err_yaw - 2 * np.pi
        elif err_yaw < -np.pi:
            err_yaw = err_yaw + 2 * np.pi

        rospy.loginfo("err: {:.3f}, {:.3f}, {:.3f}, {:.3f}".format(errx,
                                                                   erry,
                                                                   errz,
                                                                   err_yaw/np.pi*180))

        ux = self.KPx * errx
        uy = self.KPy * erry
        uz = self.KPz * errz
        uyaw = self.KPyaw * err_yaw

        if (ux <= -1.5 or ux >= 1.5):
            ux = 1.5 * ux / np.abs(ux)
        if (uy <= -1.5 or uy >= 1.5):
            uy = 1.5 * uy / np.abs(uy)
        if (uz <= -0.4 or uz >= 0.4):
            uz = 0.4 * uz / np.abs(uz)

        self.vs.twist.linear.x = ux
        self.vs.twist.linear.y = uy
        self.vs.twist.linear.z = uz
        self.vs.twist.angular.z = uyaw

    def takeoff(self):
        last_request = rospy.Time.now()

        while not rospy.is_shutdown():
            if (self.current_state.mode != "OFFBOARD" and rospy.Time.now() - last_request > rospy.Duration(5.0)):
                rospy.wait_for_service("/mavros/set_mode")
                ans = self.set_mode_client(self.offb_set_mode)
                if ans.mode_sent:
                    rospy.loginfo("Offboard enable")
                last_request = rospy.Time.now()
            else:
                if (not self.current_state.armed) and (rospy.Time.now() - last_request > rospy.Duration(5.0)):
                    rospy.wait_for_service("/mavros/cmd/arming")
                    ans = self.arming_client(self.arm_cmd)
                    if ans.success:
                        rospy.loginfo("Vehicle armed")
                    last_request = rospy.Time.now()

            if self.vir[3] > np.pi:
                self.vir[3] -= 2 * np.pi
            elif (self.vir[3] < -np.pi):
                self.vir[3] += 2 * np.pi

            rospy.loginfo("setpoint: {:.3f}, {:.3f}, {:.3f}, {:.3f}".format(self.vir[0],
                                                                            self.vir[1],
                                                                            self.vir[2],
                                                                            self.vir[3]/np.pi*180))
            self.follow()
            self.local_vel_pub.publish(self.vs)
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("motive")
    motive = Motive()

    motive.initialize()
    motive.takeoff()
