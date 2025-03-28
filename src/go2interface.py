# mujoco imports
import pathlib
import mujoco
import mujoco.viewer
from mujoco_mpc import agent as agent_lib

# unitree imports
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
from unitree_sdk2py.utils.thread import RecurrentThread
import go2_constants as go2
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.go2.robot_state.robot_state_client import RobotStateClient
# ros imports
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped

# other imports
import threading
import time
from collections import deque
import numpy as np
from scipy.spatial.transform import Rotation as R

class Go2Interface:

    def __init__(self, standup=True):

        self.standup = standup
        # initialize mujoco model
        mujoco_mpc_path = pathlib.Path(agent_lib.__file__).resolve().parent
        model_path = (
            mujoco_mpc_path / "mjpc/tasks/quadruped/task_flat.xml"
        )
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)
        
        mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        mujoco.mj_forward(self.model, self.data)

        # initalize mocap 
        rospy.init_node('mocap_odom_listener_loop', anonymous=True)
        
        self.odom_sub = rospy.Subscriber('/mocap_node/Robot_2/Odom', Odometry, self.odom_callback)
        self.vel_sub = rospy.Subscriber('/mocap_node/Robot_2/velocity', TwistStamped, self.vel_callback)

        self.pos_offset = np.array([-2.8, 0.78, 0])
        
        self.lock = threading.Lock()

        self.Kp = 60.0
        self.Kd = 5.0

        # initialize sensor data
        self._joint_angles = np.zeros(12)
        self._joint_angle_vels = np.zeros(12)

        self._pos = np.array([0.0, 0.0, 0.3])
        self._quat = np.array([1.0, 0.0, 0.0, 0.0])
        self._lin_vel = np.zeros(3)
        self._ang_vel = np.zeros(3)
        # TODO: update this with the actual values from the mocap system
        self.qpos = np.concatenate([self._pos, self._quat, self._joint_angles])
        self.qvel = np.concatenate([self._lin_vel, self._ang_vel, self._joint_angle_vels])

        # self.update_mocap()
        self.low_cmd = unitree_go_msg_dds__LowCmd_()  
        self.low_state = None  

        self._targetPos_1 = [0.0, 1.36, -2.65, 0.0, 1.36, -2.65,
                             -0.2, 1.36, -2.65, 0.2, 1.36, -2.65]
        self._targetPos_2 = [0.0, 0.67, -1.3, 0.0, 0.67, -1.3,
                             0.0, 0.67, -1.3, 0.0, 0.67, -1.3]
        self._targetPos_3 = [-0.35, 1.36, -2.65, 0.35, 1.36, -2.65,
                             -0.5, 1.36, -2.65, 0.5, 1.36, -2.65]

        self.startPos = [0.0] * 12
        self.duration_1 = 500
        self.duration_2 = 500
        self.duration_3 = 1000
        self.duration_4 = 900
        self.percent_1 = 0
        self.percent_2 = 0
        self.percent_3 = 0
        self.percent_4 = 0

        self.firstRun = True
        self.done = False
        # thread handling
        self.lowCmdWriteThreadPtr = None

        self.crc = CRC()
        pass
    # Public methods
    def Init(self):
        self.InitLowCmd()

        # create publisher #
        self.lowcmd_publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowcmd_publisher.Init()

        # create subscriber # 
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateMessageHandler, 10)


        rsc = RobotStateClient()
        rsc.SetTimeout(3.0)
        rsc.Init()
        code = rsc.ServiceSwitch("sport_mode", False)    
        if code != 0:
            print("service stop sport_mode error. code:", code)
        else:
            print("service stop sport_mode success. code:", code)
    
    def Start(self):
        self.lowCmdWriteThreadPtr = RecurrentThread(
            interval=0.002, target=self.LowCmdWrite, name="writebasiccmd"
        )
        self.lowCmdWriteThreadPtr.Start()

    # Private methods
    def InitLowCmd(self):
        self.low_cmd.head[0]=0xFE
        self.low_cmd.head[1]=0xEF
        self.low_cmd.level_flag = 0xFF
        self.low_cmd.gpio = 0
        for i in range(20):
            self.low_cmd.motor_cmd[i].mode = 0x01  # (PMSM) mode
            self.low_cmd.motor_cmd[i].q= go2.PosStopF
            self.low_cmd.motor_cmd[i].kp = 0
            self.low_cmd.motor_cmd[i].dq = go2.VelStopF
            self.low_cmd.motor_cmd[i].kd = 0
            self.low_cmd.motor_cmd[i].tau = 0

    def LowStateMessageHandler(self, msg: LowState_):
        self.low_state = msg

    def send_action(self, action):
        for i in range(12):
            self.low_cmd.motor_cmd[i].q = action[i]
            self.low_cmd.motor_cmd[i].dq = 0
            self.low_cmd.motor_cmd[i].kp = self.Kp
            self.low_cmd.motor_cmd[i].kd = self.Kd
            self.low_cmd.motor_cmd[i].tau = 0
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.lowcmd_publisher.Write(self.low_cmd)
        pass

    def LowCmdWrite(self):

        if self.firstRun:
            for i in range(12):
                self.startPos[i] = self.low_state.motor_state[i].q
            self.firstRun = False
        
        # update state estimates
        self._update_joint_angles()
        self.update_state()

        if self.standup:
            action = np.zeros(12)
            if self.percent_1 < 1:
                self.percent_1 += 1.0 / self.duration_1
                self.percent_1 = min(self.percent_1, 1)
                for i in range(12):
                    action[i] = (1 - self.percent_1) * self.startPos[i] + self.percent_1 * self._targetPos_1[i]

            elif (self.percent_1 == 1) and (self.percent_2 < 1):
                self.percent_2 += 1.0 / self.duration_2
                self.percent_2 = min(self.percent_2, 1)
                for i in range(12):
                    action[i] = (1 - self.percent_2) * self._targetPos_1[i] + self.percent_2 * self._targetPos_2[i]

            elif (self.percent_1 == 1) and (self.percent_2 == 1) and (self.percent_3 < 1):
            # else:   # for calibration only
                self.percent_3 += 1.0 / self.duration_3
                self.percent_3 = min(self.percent_3, 1)
                for i in range(12):
                    action[i] = self._targetPos_2[i]
            else:
                self.standup = False

        else:
            action = self.action  # Assuming self.action is updated elsewhere

        # # Use send_action to send the joint commands
        self.send_action(action)

    def _update_joint_angles(self):
        for i in range(12):
            self._joint_angles[i] = self.low_state.motor_state[i].q
            self._joint_angle_vels[i] = self.low_state.motor_state[i].dq
    
    def odom_callback(self, odom_msg):
        pos_msg = odom_msg.pose.pose.position
        quat_msg = odom_msg.pose.pose.orientation
        pos = np.array([pos_msg.x, pos_msg.y, pos_msg.z]) - self.pos_offset
        quat = np.array([quat_msg.w, quat_msg.x, quat_msg.y, quat_msg.z])
    
        with self.lock:
            self._pos = pos
            self._quat = quat

    def vel_callback(self, vel_msg):
        lin_vel_msg = vel_msg.twist.linear
        ang_vel_msg = vel_msg.twist.angular
        lin_vel = np.array([lin_vel_msg.x, lin_vel_msg.y, lin_vel_msg.z])
        ang_vel = np.array([ang_vel_msg.x, ang_vel_msg.y, ang_vel_msg.z])
        
        with self.lock:
            self._lin_vel = lin_vel
            self._ang_vel = ang_vel

    def update_state(self):
        with self.lock:
            self.qpos = np.concatenate([self._pos, self._quat, self._joint_angles])
            self.qvel = np.concatenate([self._lin_vel, self._ang_vel, self._joint_angle_vels])
        return None

    def _mocap_vel_loop(self):
        while True:
            self.update_mocap_vel()

    def _mocap_loop(self):
        while True:
            self.update_mocap()

    def launch_mjpc_gui(self):
        
        with agent_lib.Agent(
            server_binary_path=pathlib.Path(agent_lib.__file__).parent
            / "mjpc"
            / "ui_agent_server",
            task_id="Quadruped Flat",
            model=self.model,
        ) as self.agent:
            while True:
                t1 = time.perf_counter()
                self.agent.set_state(qpos=self.qpos, qvel=self.qvel)
                self.action = self.agent.get_action()
                print(time.perf_counter() - t1)
        pass

if __name__ == "__main__":

    ChannelFactoryInitialize(0)
    go2robot = Go2Interface(standup=False)
    go2robot.Init()
    go2robot.Start()

    go2robot.launch_mjpc_gui()
    pass