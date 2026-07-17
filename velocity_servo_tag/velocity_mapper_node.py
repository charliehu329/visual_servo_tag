#!/usr/bin/env python3
"""
vel_send_node.py 节点


ros2 launch velocity_servo_tag velocity_servo_tag.launch.py

发送相机速度
ros2 topic pub -r 30 \
  /simulink/camera_velocity \
  std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"

ros2 topic pub --once \
  /simulink/camera_velocity \
  std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"



概括：
    创建 VelocityMapperNode 节点。
    接收视觉伺服节点发布的相机速度 V_c，将其转换为末端速度 V_e，
    再根据当前关节角和雅可比矩阵计算关节速度 q_dot，
    最后发送给 Franka 速度控制器。

功能：
    1. 订阅当前关节状态。
    2. 订阅视觉伺服速度 V_c。
    3. 将相机速度转换为末端执行器速度。
    4. 根据雅可比矩阵将末端速度转换为关节速度。
    5. 对目标关节速度进行有限值检查和绝对值限幅。
    6. 对连续关节速度命令进行加速度限制，避免速度突变。
    7. 发布平滑后的关节速度给 Franka。
    8. 视觉速度超时后平滑减速到零。
    9. 关节状态超时后平滑减速到零。
    10. 收到错误、无效或非有限速度时平滑减速到零。
    11. 节点退出前尝试平滑减速到零。

接口：
    joint_state_callback(msg)
    visual_velocity_callback(msg)
    timer_callback()
    publish_zero_velocity()

输入：
    JointState:
        当前七个关节角。

    /simulink/camera_velocity:
        V_c = [vx, vy, vz, wx, wy, wz]
        在相机坐标系下表达。

输出：
    command_topic:
        q_dot = [dq1, dq2, dq3, dq4, dq5, dq6, dq7]

安全机制：
    1. visual_velocity_timeout_sec：
       超过该时间未收到新的视觉速度，将目标关节速度设为零。

    2. joint_state_timeout_sec：
       超过该时间未收到新的关节状态，将目标关节速度设为零。

    3. max_joint_accelerations：
       限制相邻控制周期的关节速度变化量，避免非零速度与零速度
       或正负速度之间直接跳变。

    4. Simulink 没有有效目标时应持续发布零相机速度。

说明：
    原有运动参数仍由 YAML 或命令行提供。
    max_joint_accelerations 可分别设置 7 个关节的加速度上限，
    正式运行时建议在 YAML 中明确设置。
"""

import os

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from velocity_servo_tag.robot_kinematics import FrankaKinematics
from velocity_servo_tag.safety import (
    check_finite_vector,
    limit_joint_velocity,
)


class VelocityMapperNode(Node):
    """
    Franka 笛卡尔速度到关节速度转换节点。
    """

    def __init__(self):
        super().__init__("velocity_mapper_node")

        # =====================================================
        # 声明参数
        # =====================================================

        # 运动学模型
        self.declare_parameter("urdf_path", "")
        self.declare_parameter("end_effector_frame", "fr3_hand_tcp")
        self.declare_parameter(
            "T_end_effector_camera",
            [
                1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0,
            ]
        )

        # 控制参数
        self.declare_parameter("damping", 0.02)
        self.declare_parameter("publish_rate_hz", 120.0)
        self.declare_parameter("duration_sec", 0.0)
        self.declare_parameter("dry_run", True)

        # Topic 和速度限制
        self.declare_parameter("joint_state_topic", "/franka/joint_states")
        self.declare_parameter(
            "command_topic",
            "/velocity_mapper_node/target_joints_velocities"
        )
        self.declare_parameter(
            "visual_velocity_topic",
            "/simulink/camera_velocity"
        )
        self.declare_parameter(
            "max_joint_velocities",
            [0.05] * 7
        )

        # 相邻控制周期之间允许的最大关节速度变化率。
        # 单位 rad/s^2。保守默认值用于兼容旧 YAML。
        self.declare_parameter(
            "max_joint_accelerations",
            [0.20] * 7
        )

        # 安全超时参数
        self.declare_parameter(
            "visual_velocity_timeout_sec",
            0.2
        )
        self.declare_parameter(
            "joint_state_timeout_sec",
            0.1
        )

        # =====================================================
        # 读取参数
        # =====================================================

        self.urdf_path = str(
            self.get_parameter("urdf_path").value
        )

        if not self.urdf_path:
            self.urdf_path = os.path.join(
                get_package_share_directory("velocity_servo_tag"),
                "config",
                "urdf",
                "fr3.urdf"
            )

        if not os.path.isfile(self.urdf_path):
            raise ValueError(
                f"URDF file does not exist: {self.urdf_path}"
            )

        self.end_effector_frame = (
            self.get_required_parameter(
                "end_effector_frame"
            )
        )

        transform_data = np.asarray(
            self.get_required_parameter(
                "T_end_effector_camera"
            ),
            dtype=float
        )

        if transform_data.size != 16:
            raise ValueError(
                "T_end_effector_camera must "
                "contain 16 elements."
            )

        if not np.all(
            np.isfinite(transform_data)
        ):
            raise ValueError(
                "T_end_effector_camera contains "
                "nan or inf."
            )

        self.T_end_effector_camera = (
            transform_data.reshape(4, 4)
        )

        rotation = self.T_end_effector_camera[:3, :3]
        if (
            not np.allclose(
                self.T_end_effector_camera[3, :],
                [0.0, 0.0, 0.0, 1.0],
                atol=1e-8
            )
            or not np.allclose(
                rotation.T @ rotation,
                np.eye(3),
                atol=1e-6
            )
            or not np.isclose(
                np.linalg.det(rotation),
                1.0,
                atol=1e-6
            )
        ):
            raise ValueError(
                "T_end_effector_camera must be a rigid transform."
            )

        self.damping = float(
            self.get_required_parameter(
                "damping"
            )
        )

        self.publish_rate_hz = float(
            self.get_required_parameter(
                "publish_rate_hz"
            )
        )

        self.duration_sec = float(
            self.get_required_parameter(
                "duration_sec"
            )
        )

        self.dry_run = bool(
            self.get_required_parameter(
                "dry_run"
            )
        )

        self.joint_state_topic = str(
            self.get_required_parameter(
                "joint_state_topic"
            )
        )

        self.command_topic = str(
            self.get_required_parameter(
                "command_topic"
            )
        )

        self.visual_velocity_topic = str(
            self.get_parameter(
                "visual_velocity_topic"
            ).value
        )

        self.max_joint_velocities = np.asarray(
            self.get_parameter(
                "max_joint_velocities"
            ).value,
            dtype=float
        ).reshape(-1)

        self.max_joint_accelerations = np.asarray(
            self.get_parameter(
                "max_joint_accelerations"
            ).value,
            dtype=float
        ).reshape(-1)

        self.visual_velocity_timeout_sec = float(
            self.get_required_parameter(
                "visual_velocity_timeout_sec"
            )
        )

        self.joint_state_timeout_sec = float(
            self.get_required_parameter(
                "joint_state_timeout_sec"
            )
        )

        self.validate_parameters()

        # =====================================================
        # 当前控制状态
        # =====================================================

        # 当前关节角
        self.current_q = None

        # 当前末端速度
        self.V_e = np.zeros(
            6,
            dtype=float
        )

        # 最近一次收到消息的时间
        self.last_visual_velocity_time = None
        self.last_joint_state_time = None

        # 当前安全停止原因
        # 用来避免在每个定时器周期重复打印相同警告
        self.safe_stop_reason = None

        # 已经实际发布的关节速度命令。
        # 后续所有目标速度都从该状态平滑逼近，不能直接跳变。
        self.commanded_q_dot = np.zeros(
            7,
            dtype=float
        )

        # 上一次执行速度变化率限制的时间。
        self.last_command_time = None

        # 控制时间到达后，先平滑减速到零，再关闭节点。
        self.duration_stop_requested = False

        # =====================================================
        # 初始化运动学模型
        # =====================================================

        self.kinematics = FrankaKinematics(
            urdf_path=self.urdf_path,
            end_effector_frame=(
                self.end_effector_frame
            )
        )

        # =====================================================
        # ROS 通信
        # =====================================================

        self.joint_state_sub = (
            self.create_subscription(
                JointState,
                self.joint_state_topic,
                self.joint_state_callback,
                10
            )
        )

        self.visual_velocity_sub = (
            self.create_subscription(
                Float64MultiArray,
                self.visual_velocity_topic,
                self.visual_velocity_callback,
                10
            )
        )

        self.publisher = self.create_publisher(
            Float64MultiArray,
            self.command_topic,
            10
        )

        self.start_time = self.get_clock().now()

        timer_period = (
            1.0 /
            self.publish_rate_hz
        )

        self.timer = self.create_timer(
            timer_period,
            self.timer_callback
        )

        # =====================================================
        # 启动信息
        # =====================================================

        self.get_logger().info(
            "Cartesian servo node started."
        )

        self.get_logger().info(
            f"URDF: {self.urdf_path}"
        )

        self.get_logger().info(
            "End-effector frame: "
            f"{self.end_effector_frame}"
        )

        self.get_logger().info(
            "Joint state topic: "
            f"{self.joint_state_topic}"
        )

        self.get_logger().info(
            "Visual velocity topic: "
            f"{self.visual_velocity_topic}"
        )

        self.get_logger().info(
            "Command topic: "
            f"{self.command_topic}"
        )

        self.get_logger().info(
            f"Damping: {self.damping}"
        )

        self.get_logger().info(
            "Publish rate: "
            f"{self.publish_rate_hz} Hz"
        )

        self.get_logger().info(
            f"Duration: {self.duration_sec} s"
        )

        self.get_logger().info(
            "Max joint velocities: "
            f"{self.max_joint_velocities.tolist()} rad/s"
        )

        self.get_logger().info(
            "Max joint accelerations: "
            f"{self.max_joint_accelerations.tolist()} rad/s^2"
        )

        self.get_logger().info(
            "Visual velocity timeout: "
            f"{self.visual_velocity_timeout_sec} s"
        )

        self.get_logger().info(
            "Joint state timeout: "
            f"{self.joint_state_timeout_sec} s"
        )

        self.get_logger().info(
            f"Dry run: {self.dry_run}"
        )

        self.get_logger().info(
            "Waiting for joint state and "
            "visual servo velocity."
        )

    def validate_parameters(self):
        """
        检查参数是否合法。
        """

        if self.damping < 0.0:
            raise ValueError(
                "damping must be non-negative."
            )

        if self.publish_rate_hz <= 0.0:
            raise ValueError(
                "publish_rate_hz must be positive."
            )

        if self.duration_sec < 0.0:
            raise ValueError(
                "duration_sec must be non-negative."
            )

        if (
            self.max_joint_velocities.shape != (7,)
            or not np.all(np.isfinite(self.max_joint_velocities))
            or np.any(self.max_joint_velocities <= 0.0)
        ):
            raise ValueError(
                "max_joint_velocities must contain "
                "7 positive finite values."
            )

        if (
            self.max_joint_accelerations.shape != (7,)
            or not np.all(np.isfinite(self.max_joint_accelerations))
            or np.any(self.max_joint_accelerations <= 0.0)
        ):
            raise ValueError(
                "max_joint_accelerations must contain "
                "7 positive finite values."
            )

        if (
            self.visual_velocity_timeout_sec
            <= 0.0
        ):
            raise ValueError(
                "visual_velocity_timeout_sec "
                "must be positive."
            )

        if self.joint_state_timeout_sec <= 0.0:
            raise ValueError(
                "joint_state_timeout_sec "
                "must be positive."
            )

    def get_required_parameter(
        self,
        name
    ):
        """
        读取必须由 YAML 或命令行提供的参数。

        如果参数未设置，直接报错。
        """

        parameter = self.get_parameter(name)

        if (
            parameter.type_ ==
            rclpy.Parameter.Type.NOT_SET
        ):
            raise ValueError(
                f"Required parameter '{name}' "
                "is not set. Please provide it "
                "in YAML or command line."
            )

        return parameter.value

    def joint_state_callback(
        self,
        msg
    ):
        """
        接收 Franka 当前关节角。
        """

        joint_map = dict(
            zip(
                msg.name,
                msg.position
            )
        )

        try:
            current_q = np.asarray(
                [
                    joint_map["fr3_joint1"],
                    joint_map["fr3_joint2"],
                    joint_map["fr3_joint3"],
                    joint_map["fr3_joint4"],
                    joint_map["fr3_joint5"],
                    joint_map["fr3_joint6"],
                    joint_map["fr3_joint7"],
                ],
                dtype=float
            )

            current_q = check_finite_vector(
                current_q,
                "current_q"
            )

        except KeyError as error:
            self.current_q = None

            self.enter_safe_stop(
                "Joint state message is missing "
                f"a required joint: {error}"
            )

            return

        except ValueError as error:
            self.current_q = None

            self.enter_safe_stop(
                f"Invalid joint state: {error}"
            )

            return

        self.current_q = current_q

        self.last_joint_state_time = (
            self.get_clock().now()
        )

    def visual_velocity_callback(
        self,
        msg
    ):
        """
        接收视觉伺服节点输出的相机速度 V_c。

        输入：
            msg.data:
                V_c = [vx, vy, vz, wx, wy, wz]
                相机坐标系下的速度。

                
        输出：
            self.V_e:
                V_e = [vx, vy, vz, wx, wy, wz]
                在末端执行器坐标系下表达。

                V_e = (self.camera_velocity_to_end_effector_velocity(V_c)
                

        说明：
            视觉节点没有检测到目标时，
            应发布 [0, 0, 0, 0, 0, 0]。
        """

        V_c = np.asarray(
            msg.data,
            dtype=float
        ).reshape(-1)

        if V_c.shape != (6,):
            self.enter_safe_stop(
                "Visual velocity must contain "
                "exactly 6 elements."
            )
            return

        try:
            V_c = check_finite_vector(
                V_c,
                "V_c"
            )

            V_e = (
                self.camera_velocity_to_end_effector_velocity(
                    V_c
                )
            )

            V_e = check_finite_vector(
                V_e,
                "V_e"
            )

        except ValueError as error:
            self.enter_safe_stop(
                f"Invalid visual velocity: {error}"
            )
            return

        # 只有数据完全合法时才更新当前速度
        self.V_e = V_e

        self.last_visual_velocity_time = (
            self.get_clock().now()
        )

    @staticmethod
    def skew(
        vector
    ):
        """
        将三维向量转换为反对称矩阵。
        """

        x, y, z = vector

        return np.asarray(
            [
                [0.0, -z, y],
                [z, 0.0, -x],
                [-y, x, 0.0]
            ],
            dtype=float
        )

    def camera_velocity_to_end_effector_velocity(
        self,
        V_c
    ):
        """
        将相机坐标系速度 V_c 转换为
        末端执行器坐标系速度 V_e。

        输入：
            V_c:
                [vx, vy, vz, wx, wy, wz]

        输出：
            V_e:
                [vx, vy, vz, wx, wy, wz]
        """

        R_e_c = (
            self.T_end_effector_camera[
                :3,
                :3
            ]
        )

        t_e_c = (
            self.T_end_effector_camera[
                :3,
                3
            ]
        )

        adjoint_e_c = np.zeros(
            (6, 6),
            dtype=float
        )

        # 当前速度排列：
        # [线速度, 角速度]
        adjoint_e_c[:3, :3] = R_e_c

        adjoint_e_c[:3, 3:] = (
            self.skew(t_e_c) @
            R_e_c
        )

        adjoint_e_c[3:, 3:] = R_e_c

        V_e = adjoint_e_c @ V_c

        return V_e

    def message_is_fresh(
        self,
        last_message_time,
        timeout_sec,
        now
    ):
        """
        判断某一类消息是否在允许时间内更新。

        输出：
            is_fresh:
                是否仍然有效。

            age_sec:
                距离最后一次消息的时间。
                从未收到时返回 None。
        """

        if last_message_time is None:
            return False, None

        age_sec = (
            now -
            last_message_time
        ).nanoseconds * 1e-9

        # 防止 ROS 时间发生小幅回跳
        age_sec = max(
            0.0,
            float(age_sec)
        )

        is_fresh = (
            age_sec <= timeout_sec
        )

        return is_fresh, age_sec

    def compute_limiter_dt(
        self,
        now
    ):
        """
        计算本次速度变化率限制使用的时间间隔。

        当 Python 定时器偶尔延迟时，不允许因为 dt 变大而
        一次性跨越更大的速度步长，因此 dt 最大不超过标称周期。
        """

        nominal_dt = (
            1.0 /
            self.publish_rate_hz
        )

        if self.last_command_time is None:
            dt = nominal_dt
        else:
            dt = (
                now -
                self.last_command_time
            ).nanoseconds * 1e-9

            if (
                not np.isfinite(dt)
                or dt <= 0.0
            ):
                dt = nominal_dt

            # 定时器卡顿后仍只允许走一个正常周期的速度增量。
            dt = min(
                float(dt),
                nominal_dt
            )

        self.last_command_time = now

        return max(
            float(dt),
            1e-6
        )

    def limit_joint_velocity_change(
        self,
        target_q_dot,
        dt
    ):
        """
        限制相邻两次关节速度命令的变化量。

        输入：
            target_q_dot:
                已经过绝对速度限幅的目标关节速度。

            dt:
                本次控制时间间隔，单位 s。

        输出：
            commanded_q_dot:
                可以安全发布的平滑关节速度。

        方法：
            |q_dot[k] - q_dot[k-1]|
                <= max_joint_accelerations * dt
        """

        target_q_dot = np.asarray(
            target_q_dot,
            dtype=float
        ).reshape(7)

        target_q_dot = check_finite_vector(
            target_q_dot,
            "target_q_dot"
        )

        target_q_dot = limit_joint_velocity(
            target_q_dot,
            max_abs=self.max_joint_velocities
        )

        max_delta = (
            self.max_joint_accelerations *
            float(dt)
        )

        delta_q_dot = (
            target_q_dot -
            self.commanded_q_dot
        )

        delta_q_dot = np.clip(
            delta_q_dot,
            -max_delta,
            max_delta
        )

        commanded_q_dot = (
            self.commanded_q_dot +
            delta_q_dot
        )

        # 防止浮点累计误差突破绝对速度上限。
        commanded_q_dot = limit_joint_velocity(
            commanded_q_dot,
            max_abs=self.max_joint_velocities
        )

        # 接近零目标且已经足够小时，明确归零，避免残留极小命令。
        zero_target = np.allclose(
            target_q_dot,
            0.0,
            atol=1e-12
        )

        if (
            zero_target
            and np.all(
                np.abs(commanded_q_dot)
                <= max_delta
            )
        ):
            commanded_q_dot = np.zeros(
                7,
                dtype=float
            )

        self.commanded_q_dot = commanded_q_dot

        return commanded_q_dot.copy()

    def publish_joint_velocity(
        self,
        target_q_dot,
        now=None
    ):
        """
        将目标关节速度经过变化率限制后发布。

        正常速度、目标丢失、消息超时和退出停止都必须调用
        本函数，不能绕过限制器直接发布七维零速度。
        """

        if now is None:
            now = self.get_clock().now()

        dt = self.compute_limiter_dt(now)

        commanded_q_dot = (
            self.limit_joint_velocity_change(
                target_q_dot,
                dt
            )
        )

        if not self.dry_run:
            message = Float64MultiArray()
            message.data = (
                commanded_q_dot.tolist()
            )

            self.publisher.publish(message)

        return commanded_q_dot

    def timer_callback(self):
        """
        周期计算并发送关节速度。

        任何输入失效时，把目标速度设为零，但实际发布速度仍然
        经过 max_joint_accelerations 限制平滑下降，禁止直接跳零。
        """

        now = self.get_clock().now()

        elapsed = (
            now -
            self.start_time
        ).nanoseconds * 1e-9

        zero_q_dot = np.zeros(
            7,
            dtype=float
        )

        # =====================================================
        # 控制持续时间检查
        # =====================================================

        if (
            self.duration_sec > 0.0
            and elapsed >= self.duration_sec
        ):
            self.duration_stop_requested = True
            self.enter_safe_stop(
                "Control duration reached."
            )

            commanded_q_dot = (
                self.publish_joint_velocity(
                    zero_q_dot,
                    now=now
                )
            )

            if np.allclose(
                commanded_q_dot,
                0.0,
                atol=1e-12
            ):
                self.get_logger().info(
                    "Duration reached and joint "
                    "velocity ramped to zero. Stop."
                )
                rclpy.shutdown()

            return

        # =====================================================
        # 关节状态看门狗
        # =====================================================

        (
            joint_state_is_fresh,
            joint_state_age
        ) = self.message_is_fresh(
            self.last_joint_state_time,
            self.joint_state_timeout_sec,
            now
        )

        if (
            self.current_q is None
            or not joint_state_is_fresh
        ):
            if joint_state_age is None:
                reason = (
                    "Waiting for the first "
                    "joint state message."
                )
            else:
                reason = (
                    "Joint state timeout: "
                    f"{joint_state_age:.3f} s."
                )

            self.enter_safe_stop(reason)
            self.publish_joint_velocity(
                zero_q_dot,
                now=now
            )
            return

        # =====================================================
        # 视觉速度看门狗
        # =====================================================

        (
            visual_velocity_is_fresh,
            visual_velocity_age
        ) = self.message_is_fresh(
            self.last_visual_velocity_time,
            self.visual_velocity_timeout_sec,
            now
        )

        if not visual_velocity_is_fresh:
            if visual_velocity_age is None:
                reason = (
                    "Waiting for the first "
                    "visual velocity message."
                )
            else:
                reason = (
                    "Visual velocity timeout: "
                    f"{visual_velocity_age:.3f} s."
                )

            self.enter_safe_stop(reason)
            self.publish_joint_velocity(
                zero_q_dot,
                now=now
            )
            return

        # 两类数据都恢复后，清除安全停止状态。
        if self.safe_stop_reason is not None:
            self.get_logger().info(
                "Joint state and visual velocity "
                "are valid again."
            )

            self.safe_stop_reason = None

        # =====================================================
        # 计算目标关节速度
        # =====================================================

        try:
            J = self.kinematics.compute_jacobian(
                self.current_q
            )

            J = np.asarray(
                J,
                dtype=float
            )

            if J.shape != (6, 7):
                raise ValueError(
                    "Jacobian must have shape "
                    f"(6, 7), but got {J.shape}."
                )

            if not np.all(np.isfinite(J)):
                raise ValueError(
                    "Jacobian contains nan or inf."
                )

            target_q_dot = (
                self.cartesian_velocity_to_joint_velocity(
                    V_e=self.V_e,
                    J=J,
                    damping=self.damping
                )
            )

            target_q_dot = limit_joint_velocity(
                target_q_dot,
                max_abs=(
                    self.max_joint_velocities
                )
            )

            commanded_q_dot = (
                self.publish_joint_velocity(
                    target_q_dot,
                    now=now
                )
            )

        except Exception as error:
            self.enter_safe_stop(
                "Failed to compute safe joint "
                f"velocity: {error}"
            )
            self.publish_joint_velocity(
                zero_q_dot,
                now=now
            )
            return

        self.get_logger().info(
            f"q: "
            f"{np.round(self.current_q, 4).tolist()} | "
            "q_dot_target: "
            f"{np.round(target_q_dot, 5).tolist()} | "
            "q_dot_cmd: "
            f"{np.round(commanded_q_dot, 5).tolist()}",
            throttle_duration_sec=10.0
        )

    def enter_safe_stop(
        self,
        reason
    ):
        """
        进入安全停止状态。

        本函数只把末端目标速度清零并记录停止原因。
        真正发送给机器人的关节速度由 timer_callback 中的
        publish_joint_velocity() 平滑减小到零。
        """

        self.V_e = np.zeros(
            6,
            dtype=float
        )

        if reason != self.safe_stop_reason:
            self.get_logger().warning(
                f"Safety stop: {reason}"
            )

            self.safe_stop_reason = reason

    def publish_zero_velocity(self):
        """
        请求关节速度平滑下降到零，并发布一个限幅后的控制步。

        注意：这里故意不再直接发布 [0.0] * 7，
        以避免非零速度命令突然跳变为零。
        """

        self.V_e = np.zeros(
            6,
            dtype=float
        )

        return self.publish_joint_velocity(
            np.zeros(
                7,
                dtype=float
            )
        )
    
    def cartesian_velocity_to_joint_velocity(self, V_e, J, damping=0.01):
        """
        功能：
            将末端速度 V_e 转换为关节速度 q_dot。

        接口：
            cartesian_velocity_to_joint_velocity(V_e, J, damping=0.01)

        输入：
            V_e: 6维末端速度 [vx, vy, vz, wx, wy, wz]
                shape = (6,)
            J:   当前雅可比矩阵
                shape = (6, 7)
            damping : float
                求最小二乘伪逆时的阻尼因子，默认为 0.01

        输出：
            q_dot: 7维关节速度 [dq1, dq2, dq3, dq4, dq5, dq6, dq7]
                shape = (7,)

        方法：
            使用阻尼最小二乘伪逆：
            q_dot = J.T @ inv(J @ J.T + damping^2 * I) @ V_e
        """

        V_e = np.asarray(V_e, dtype=float).reshape(6)
        J = np.asarray(J, dtype=float)

        if J.shape != (6, 7):
            raise ValueError(f"Expected J shape (6, 7), but got {J.shape}")

        if damping < 0:
            raise ValueError("damping must be non-negative")

        identity_6 = np.eye(6)

        # Damped pseudo-inverse: J^T (J J^T + λ² I)^-1
        normal_matrix = J @ J.T + (damping ** 2) * identity_6
        q_dot = J.T @ np.linalg.solve(normal_matrix, V_e)

        return q_dot



def main(args=None):
    """
    ROS 2 节点入口。
    """

    rclpy.init(args=args)

    node = None

    try:
        node = VelocityMapperNode()
        rclpy.spin(node)

    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info(
                "Keyboard interrupt. "
                "Send zero velocity."
            )

    except Exception as error:
        if node is not None:
            node.get_logger().error(
                f"Unexpected error: {error}"
            )

        raise

    finally:
        # 只要 ROS 上下文仍有效，
        # 在退出节点前再发送一个平滑减速控制步。
        if node is not None:
            if rclpy.ok():
                node.publish_zero_velocity()

            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
