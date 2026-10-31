# 定义节点，初始化通信，保持运行，publish和subscribe
# 
# 方向  话题    类型    内容
# 仿真→控制器   /imu    sensor_msgs/msg/Imu orientation    机体姿态四元数 (x,y,z,w)；angular_velocity 机体角速度 (rad/s)
# 仿真→控制器   /joint_states   sensor_msgs/msg/JointState      name 依次是 right1,right2,left1,left2,rightwheel,leftwheel；position(rad)、velocity(rad/s)
# 控制器→仿真   /actuator_cmd   rm_msgs/msg/WheelLegCmd     6 个关节力矩 (N·m)
# 外部→控制器   /cmd_vel    geometry_msgs/msg/Twist     目标速度（阶段 2/5 用），映射到 LQR 的 v_set
# 
# ground_detection
# lqr_update
# roll_leg_control
# vmc_cal
# 
# 
# 
import numpy as np
import math
import csv
import rclpy
from .ground_detection import detect_leg_ground_status
from .lqr_update import lqr_controller
from .roll_leg_control import RollLegController
from .vmc_cal import five_links_param
from .climb_traj import VelocityRampFilter
from rclpy.node import Node
from rm_msgs.msg import WheelLegCmd
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Twist

JOINT_NAMES = ('right1', 'right2', 'left1', 'left2', 'rightwheel', 'leftwheel')


class PIDController:
    def __init__(self, kp, ki, kd, max_out, max_i, feedforward=0.0):
        self.kp = kp; self.ki = ki; self.kd = kd; self.max_out = max_out; self.max_i = max_i; self.feedforward = feedforward
        self.integral = 0.0; self.prev_error = 0.0
    def compute(self, current, target):
        error = target - current; self.integral += error
        self.integral = np.clip(self.integral, -self.max_i, self.max_i)
        derivative = (error - self.prev_error); self.prev_error = error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative + self.feedforward
        return np.clip(output, -self.max_out, self.max_out)

class WheelLegController(Node):
    def __init__(self):
        super().__init__('wheel_leg_controller')
        self.pitch_start_stamp = None
        with open('pitch.csv', 'w', newline='') as file:
            csv.writer(file).writerow(['sim_time_s', 'pitch_deg', 'stamp_s'])
        # 跨回调使用的参数、状态和控制器对象保存在 self 中。
        # topic
        self.dt = 0.002
        self.wheel_radius = 0.06
        self.target_length_left = 0.1
        self.target_length_right = 0.1
        self.roll = 0.0
        self.current_v = 0.0  # 爬坡滤波后的目标速度。
        self.actuator_pub=self.create_publisher(WheelLegCmd,'actuator_cmd',10)
        self.create_subscription(Imu,'imu',self.deal_imu,10)
        self.create_subscription(JointState,'joint_states',self.deal_joint,10)
        self.create_subscription(Twist,'cmd_vel',self.deal_twist,10)
        self.left_torque = 0.0
        self.right_torque = 0.0
        self.track = 0.183  # 左右轮间距 (m)
        self.target_w=0.0
        self.target_v=0.0
        self.common_motor_pos=0.0
        self.common_motor_speed=0.0
        self.pitch = None
        self.yaw = 0.0
        self.gyro_pitch = None
        self.gyro_yaw = None
        self.gyro_roll = 0.0
        self.robot_is_airborne = False
        self.ground_detection_enabled = False  # 默认关闭离地检测。
        # 缓存同一仿真步的两种原始消息，配对后才更新算法状态。
        self.imu_receiver=Imu()
        self.joint_receiver=JointState()
        self.five_links_right = five_links_param(dt=self.dt)
        self.five_links_left = five_links_param(dt=self.dt)
        self.lqr_right = lqr_controller(dt=self.dt)
        self.lqr_left = lqr_controller(dt=self.dt)
        self.v_filter = VelocityRampFilter(dt=self.dt, accel_rate=0.8, decel_rate=1.4)
        self.roll_balancer = RollLegController(kp=1.0, ki=0.0, kd=0.0, max_out=0.14, min_leg=0.10, max_leg=0.14)
        self.left_yaw_pid = PIDController(0.5, 0.0, 1.0, 2.0, 0.0, feedforward=0.0)
        self.right_yaw_pid = PIDController(0.5, 0.0, 1.0, 2.0, 0.0, feedforward=0.0)
        self.left_L0_pid = PIDController(700.0, 0.0, 5.0, 150.0, 0.0, feedforward=0.0)
        self.right_L0_pid = PIDController(700.0, 0.0, 5.0, 150.0, 0.0, feedforward=0.0)
    # 发布wheellegcmd
    # 逻辑:时间由仿真器控制,仿真器传来状态(同时包括imu和joint),
    # 控制器计算返回,仿真器受到返回值
    # 执行callback又传来更新后的状态
    def publish_wheellegcmd(self):
        msg=WheelLegCmd()
        # 目标由 Twist 回调保存，滤波后的目标速度保留到下一次计算。
        self.current_v = self.v_filter.update(current_v=self.current_v, target_v=self.target_v)
        if not (self.current_v == 0.0 and self.target_v == 0):
            self.lqr_left.x_b_set = (self.common_motor_pos * self.wheel_radius)
            self.lqr_right.x_b_set = (self.common_motor_pos * self.wheel_radius)
            
        self.lqr_left.v_set = self.current_v
        self.lqr_right.v_set = self.current_v
        # 左右腿控制函数计算得到轮和腿力矩再publish即可
        ctrl_left_0, ctrl_left_1, left_wheel_T, left_flight = self.control_left_leg(
            self.five_links_left, self.lqr_left, self.left_yaw_pid, self.left_L0_pid,
            self.pitch, self.gyro_pitch, self.gyro_yaw, self.target_w, self.target_length_left,
            self.common_motor_pos, self.common_motor_speed, self.wheel_radius,
            is_airborne_last_step=self.robot_is_airborne,
            ground_detection_enabled=self.ground_detection_enabled,
            leg_tp_comp=0.0
        )

        # ---------------- 调用右腿控制函数（传入检测使能锁） ----------------
        ctrl_right_0, ctrl_right_1, right_wheel_T, right_flight = self.control_right_leg(
            self.five_links_right, self.lqr_right, self.right_yaw_pid, self.right_L0_pid,
            self.pitch, self.gyro_pitch, self.gyro_yaw, self.target_w, self.target_length_right,
            self.common_motor_pos, self.common_motor_speed, self.wheel_radius,
            is_airborne_last_step=self.robot_is_airborne,
            ground_detection_enabled=self.ground_detection_enabled,
            leg_tp_comp=0.0
        )
        self.robot_is_airborne = (
            self.ground_detection_enabled and bool(left_flight or right_flight)
        )
        msg.right_phi1=ctrl_right_0
        msg.right_phi4=ctrl_right_1
        msg.left_phi1=ctrl_left_0
        msg.left_phi4=ctrl_left_1
        msg.right_wheel=right_wheel_T
        msg.left_wheel=left_wheel_T
        self.actuator_pub.publish(msg)

    def deal_imu(self, msg):
        self.imu_receiver=msg
        self.try_control()

    def deal_joint(self, msg):
        self.joint_receiver=msg
        self.try_control()

    def try_control(self):
        if self.imu_receiver.header.stamp == self.joint_receiver.header.stamp:
            self._process_imu(self.imu_receiver)
            self._process_joint(self.joint_receiver)
            self.publish_wheellegcmd()

    def _process_imu(self,msg):
        # 读取 IMU 四元数并转为欧拉角 (rad)
        w = msg.orientation.w
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        t0 = 2.0 * (w * x + y * z)
        t1 = 1.0 - 2.0 * (x * x + y * y)
        self.roll = -math.atan2(t0, t1)
        
        t2 = 2.0 * (w * y - z * x)
        t2 = max(-1.0, min(1.0, t2))  # 防止运算出界产生 NaN
        self.pitch = -math.asin(t2)

        # 记录原始 stamp 和 pitch；相对首帧的时间列供现有 plot.py 使用。
        stamp = msg.header.stamp
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if self.pitch_start_stamp is None:
            self.pitch_start_stamp = stamp_ns
        with open('pitch.csv', 'a', newline='') as file:
            csv.writer(file).writerow([
                (stamp_ns - self.pitch_start_stamp) / 1e9,
                math.degrees(self.pitch), f'{stamp.sec}.{stamp.nanosec:09d}'])
        
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        self.yaw = math.atan2(t3, t4)
        # 读取imu中剩余数据
        self.gyro_pitch = -msg.angular_velocity.y
        self.gyro_yaw = msg.angular_velocity.z
        self.gyro_roll = -msg.angular_velocity.x
        self.target_length_left, self.target_length_right = self.roll_balancer.compute_leg_lengths(
            current_roll=self.roll,target_roll=0.0,base_length=0.1)
        return
        
    def _process_joint(self,msg):
        motor_speed_left = self.joint_receiver.velocity[5]
        motor_speed_right = self.joint_receiver.velocity[4]
        motor_pos_left = self.joint_receiver.position[5]
        motor_pos_right = self.joint_receiver.position[4]
            
        self.common_motor_speed = (motor_speed_left - motor_speed_right) / 2.0
        self.common_motor_pos = (motor_pos_left - motor_pos_right) / 2.0
        return

    def deal_twist(self,msg):
        # 目标持续沿用；Twist 更新本身不触发控制步，也不要求每帧都收到。
        self.target_v = msg.linear.x
        self.target_w = msg.angular.z
        return

    def control_right_leg(self,  five_links, lqr, yaw_pid, L0_pid, pitch, gyro_pitch, gyro_yaw,
                        target_w, target_length, common_motor_pos, common_motor_speed, wheel_radius,
                        is_airborne_last_step, ground_detection_enabled, leg_tp_comp=0.0):
        """
        右腿 LQR + VMC 核心控制计算与条件离地保护机制
        """
        motor1_angle = self.joint_receiver.position[0]
        motor2_angle =self.joint_receiver.position[1]
        
        # 1. 运动学与状态更新
        five_links.forward_kinematics_cal(0, motor1_angle, motor2_angle, pitch, gyro_pitch)
        lqr.states_update(0, five_links, pitch, gyro_pitch, -common_motor_pos, -common_motor_speed, wheel_radius)
        lqr.calc_k_matrix_from_poly(five_links.L_0)

        # 2. VMC 虚拟力基本计算
        F_0_ctrl = L0_pid.compute(five_links.L_0, target_length)
        F_0 = (4 / math.cos(five_links.theta) if math.cos(five_links.theta) != 0 else 0) + F_0_ctrl
        F_0 = np.clip(F_0, -100.0, 100.0)

        # 预计算常规状况下的 Tp
        T_p = (lqr.K[1] * (lqr.theta - 0.0)
            + lqr.K[3] * (lqr.d_theta - 0.0)
            + lqr.K[5] * (lqr.x_b - lqr.x_b_set)
            + lqr.K[7] * (lqr.v_b_whole - lqr.v_set)
            + lqr.K[9] * (lqr.phi - 0.0)
            + lqr.K[11] * (lqr.d_phi - 0.0))
        
        # 3. 核心条件判定
        if ground_detection_enabled and (detect_leg_ground_status(five_links, F_0, T_p) == 1 or is_airborne_last_step):
            right_wheel_T = 0.0
            T_p = lqr.K[1] * (lqr.theta - 0.0) + lqr.K[3] * (lqr.d_theta - 0.0)
            T_p += leg_tp_comp
            flight_flag = 1
        else:
            yaw_ctrl = yaw_pid.compute(gyro_yaw, target_w)
            wheel_T = (lqr.K[0] * (lqr.theta - 0.0)
                    + lqr.K[2] * (lqr.d_theta - 0.0)
                    + lqr.K[4] * (lqr.x_b - lqr.x_b_set)
                    + lqr.K[6] * (lqr.v_b_whole - lqr.v_set)
                    + lqr.K[8] * (lqr.phi - 0.0)
                    + lqr.K[10] * (lqr.d_phi - 0.0))
                    
            right_wheel_T = wheel_T - yaw_ctrl
            right_wheel_T = np.clip(right_wheel_T, -1.5, 1.5)
            flight_flag = 0

        torque_set_0, torque_set_1 = five_links.VMC_torque_cal(F_0, T_p)
        
        return torque_set_0, torque_set_1, right_wheel_T, flight_flag


    def control_left_leg(self,  five_links, lqr, yaw_pid, L0_pid, pitch, gyro_pitch, gyro_yaw,
                        target_w, target_length, common_motor_pos, common_motor_speed, wheel_radius,
                        is_airborne_last_step, ground_detection_enabled, leg_tp_comp=0.0):
        """
        左腿 LQR + VMC 核心控制计算与条件离地保护机制
        :param ground_detection_enabled: 是否开启离地检测，由控制器成员变量传入，默认关闭。
        """
        motor1_angle = self.joint_receiver.position[2]
        motor2_angle = self.joint_receiver.position[3]

        # 1. 运动学与状态更新
        five_links.forward_kinematics_cal(1, motor1_angle, motor2_angle, pitch, gyro_pitch)
        lqr.states_update(1, five_links, pitch, gyro_pitch, common_motor_pos, common_motor_speed, wheel_radius)
        lqr.calc_k_matrix_from_poly(five_links.L_0)

        # 2. VMC 虚拟力基本计算
        F_0_ctrl = L0_pid.compute(five_links.L_0, target_length)
        F_0 = (4 / math.cos(five_links.theta) if math.cos(five_links.theta) != 0 else 0) + F_0_ctrl
        F_0 = np.clip(F_0, -100.0, 100.0)

        # 预计算常规状况下的 Tp
        T_p = (lqr.K[1] * (lqr.theta - 0.0)
            + lqr.K[3] * (lqr.d_theta - 0.0)
            + lqr.K[5] * (lqr.x_b - lqr.x_b_set)
            + lqr.K[7] * (lqr.v_b_whole - lqr.v_set)
            + lqr.K[9] * (lqr.phi - 0.0)
            + lqr.K[11] * (lqr.d_phi - 0.0))

        # 3. 核心条件判定：只有启用离地检测后，才执行离地保护状态切换
        if ground_detection_enabled and (detect_leg_ground_status(five_links, F_0, T_p) == 1 or is_airborne_last_step):
            # 离地保护状态：轮子力矩清零，髋关节切换为空中阻尼姿态维持
            left_wheel_T = 0.0
            T_p = lqr.K[1] * (lqr.theta - 0.0) + lqr.K[3] * (lqr.d_theta - 0.0)
            T_p += leg_tp_comp
            flight_flag = 1
        else:
            # 正常控制状态（未开启离地检测或者处于着地状态）：正常的自平衡控制
            yaw_ctrl = yaw_pid.compute(gyro_yaw, target_w)
            wheel_T = (lqr.K[0] * (lqr.theta - 0.0)
                    + lqr.K[2] * (lqr.d_theta - 0.0)
                    + lqr.K[4] * (lqr.x_b - lqr.x_b_set)
                    + lqr.K[6] * (lqr.v_b_whole - lqr.v_set)
                    + lqr.K[8] * (lqr.phi - 0.0)
                    + lqr.K[10] * (lqr.d_phi - 0.0))
            
            left_wheel_T = -wheel_T - yaw_ctrl
            left_wheel_T = np.clip(left_wheel_T, -1.5, 1.5)
            flight_flag = 0

        # 4. 最终 VMC 力矩映射
        torque_set_0, torque_set_1 = five_links.VMC_torque_cal(F_0, T_p)
        
        return -torque_set_0, -torque_set_1, left_wheel_T, flight_flag




def main(args=None):
    rclpy.init(args=args)
    node=WheelLegController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



if __name__=='__main__':
    main()
