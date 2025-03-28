import pathlib
import mujoco
import mujoco.viewer
from mujoco_mpc import agent as agent_lib
import sys
sys.path.append(sys.path[0] + '/../unitree_legged_sdk/lib/python/amd64')
print(sys.path[0])
import robot_interface as sdk
import numpy as np
import rospy
from nav_msgs.msg import Odometry
import threading
import time
from collections import deque
from scipy.spatial.transform import Rotation as R

class Go1Interface:

    def __init__(self):
        # initialize unitree sdk
        LOWLEVEL  = 0xff
        self.udp = sdk.UDP(LOWLEVEL, 8080, "192.168.123.10", 8007)
        self.safe = sdk.Safety(sdk.LeggedType.Go1)
        
        self.cmd = sdk.LowCmd()
        self.state = sdk.LowState()
        self.udp.InitCmdData(self.cmd)

        self.udp.SetSend(self.cmd)
        self.udp.Send()

        self.Kp = 60 
        self.Kd = 3 
        self.pos_offset = np.array([-2.8, 0.78, 0])

        # initalize mocap 
        rospy.init_node('mocap_odom_listener_loop', anonymous=True)
       
        # initialize mujoco model
        mujoco_mpc_path = pathlib.Path(agent_lib.__file__).resolve().parent
        model_path = (
            mujoco_mpc_path / "mjpc/tasks/quadruped/task_flat.xml"
        )
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)
        
        mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        mujoco.mj_forward(self.model, self.data)

        # threading 
        # Lock for thread-safe data access
        self.lock = threading.Lock()
        # Initialize threads for asynchronous updates
        self.joint_angles_thread = threading.Thread(target=self._joint_angles_loop, daemon=True)
        self.mocap_thread = threading.Thread(target=self._mocap_loop, daemon=True)
        # Initialize history for filtering
        self.pos_history = deque(maxlen=5)
        self.quat_history = deque(maxlen=5)
        self.time_history = deque(maxlen=5)

        # Initialize filtered velocity variables
        self.pos_vel_filtered = np.zeros(3)
        self.ang_vel_filtered = np.zeros(3)
        # Smoothing factor for low-pass filter
        self.alpha = 0.3  # Tune this parameter

        self.update_joint_angles()
        self.update_mocap()
        self.qpos = np.concatenate([self.pos, self.quat, self.joint_angles])
        self.qvel = np.concatenate([self.pos_vel_filtered, self.ang_vel_filtered, self.joint_vels])
        
        # Start the threads
        self.mocap_thread.start()

        print("Initialized Go1Interface!")
        
    def _finite_diff(self, history, time_history):
        """
        Compute finite differences for velocity estimation using central differences.
        Assumes at least 3 entries in history.
        """
        x1, x2 = history[0], history[-1]
        t1, t2 = time_history[0], time_history[-1]
        
        delta_x = x2 - x1
        delta_t = t2 - t1 if t2 != t1 else 1e-6  # Avoid divide by zero
        
        return delta_x / delta_t

    def _low_pass_filter(self, new_value, prev_filtered_value):
        """
        Apply a simple exponential moving average (low-pass filter).
        """
        return self.alpha * new_value + (1 - self.alpha) * prev_filtered_value
    
    def update_joint_angles(self):
        self.udp.Recv()
        self.udp.GetRecv(self.state)
        with self.lock:
            self.joint_angles = np.array([motorState.q for motorState in self.state.motorState[:12]])
            self.joint_vels = np.array([motorState.dq for motorState in self.state.motorState[:12]])

        return None
    
    def update_mocap(self):
        # Wait for a message from the /mocap_node/Go1_body/Odom topic (blocking call)
        odom_msg = rospy.wait_for_message('/mocap_node/Go1_body/Odom', Odometry)
        # Extract position and orientation data
        pos_msg = odom_msg.pose.pose.position
        quat_msg = odom_msg.pose.pose.orientation
        pos = np.array([pos_msg.x, pos_msg.y, pos_msg.z]) - self.pos_offset
        quat = np.array([quat_msg.w, quat_msg.x, quat_msg.y, quat_msg.z])
        # Add the current measurement to the history
        self.pos_history.append(pos)
        self.quat_history.append(quat)
        self.time_history.append(time.perf_counter())

        with self.lock:
            self.pos = pos
            self.quat = quat

        if len(self.pos_history) >= 3:
            # Compute finite differences for velocity estimation
            pos_vel = self._finite_diff(self.pos_history, self.time_history)
            ang_vel = self._compute_angular_velocity(self.quat_history, self.time_history)
            with self.lock:
                # Apply low-pass filter to smooth the velocity estimates
                self.pos_vel_filtered = self._low_pass_filter(pos_vel, self.pos_vel_filtered)
                self.ang_vel_filtered = self._low_pass_filter(ang_vel, self.ang_vel_filtered)
        else:
            with self.lock:
                self.pos_vel_filtered = np.zeros(3)
                self.ang_vel_filtered = np.zeros(3)
        return None

    def _compute_angular_velocity(self, quat_history, time_history):
        """
        Compute angular velocity using quaternion difference.
        """
        # Get the two most recent quaternions
        q1 = R.from_quat(quat_history[0])  # Quaternion at time t0
        q2 = R.from_quat(quat_history[-1])  # Quaternion at time t1

        # Compute the relative rotation quaternion
        q_rel = q1.inv() * q2

        # Convert the relative quaternion to an angle-axis representation
        angle_axis = q_rel.as_rotvec()

        # Time difference
        t1, t2 = time_history[0], time_history[-1]
        delta_t = t2 - t1 if t2 != t1 else 1e-6  # Avoid divide by zero

        # Angular velocity (rad/s)
        angular_velocity = angle_axis / delta_t

        return angular_velocity

    def update_state(self):
        with self.lock:
            self.qpos = np.concatenate([self.pos, self.quat, self.joint_angles])
            self.qvel = np.concatenate([self.pos_vel_filtered, self.ang_vel_filtered, self.joint_vels])
        return None

    def _joint_angles_loop(self):
        while True:
            self.update_joint_angles()

    def _mocap_loop(self):
        while True:
            self.update_mocap()

    def update_action(self):
        self.action = self.ag
        return None

    def send_action(self):
        for i in range(12):
            self.cmd.motorCmd[i].q = self.action[i]
            self.cmd.motorCmd[i].dq = 0
            self.cmd.motorCmd[i].Kp = self.Kp
            self.cmd.motorCmd[i].Kd = self.Kd
            self.cmd.motorCmd[i].tau = 0

        self.safe.PowerProtect(self.cmd, self.state, 7)
        self.udp.SetSend(self.cmd)
        self.udp.Send()
        return None
    
    def launch_passive_viewer(self):
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            viewer.sync()
            while viewer.is_running():
                start = time.perf_counter()
                self.update_state()
                self.data.qpos[:] = self.qpos
                self.data.qvel[:] = self.qvel
                self.update_action()
                self.send_action()
                update_time = time.perf_counter() - start
                # print(f"Update time: {update_time}")
                # print(self.data.qvel[:6])
                
                mujoco.mj_forward(self.model, self.data)
                # mujoco.mj_step(self.model, self.data)
                viewer.sync()
        return None
    
    def launch_mjpc_gui(self):
        delays = []

        with agent_lib.Agent(
            server_binary_path=pathlib.Path(agent_lib.__file__).parent
            / "mjpc"
            / "ui_agent_server",
            task_id="Quadruped Flat",
            model=self.model,
        ) as agent:
            while True:
                
                self.update_joint_angles()
                self.update_state()
                agent.set_state(qpos=self.qpos, qvel=self.qvel)

                self.action = agent.get_action()

                self.send_action()

        return None
    
# if __name__ == "__main__":
#     go1 = Go1Interface()
#     # go1.launch_passive_viewer()
#     go1.launch_mjpc_gui()
    