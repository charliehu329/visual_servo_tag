#!/usr/bin/env python3
"""
vision_double_node.py

功能：
    同时读取左右两个普通 USB 相机，分别检测指定 AprilTag，
    向 Simulink Stage 1 发布 8 维双目视觉特征。
    Stage 1 尚未接入变焦硬件，因此同时发布固定 [0.0, 0.0]
    作为左右镜头累计变焦步数占位值。

输入：
    左右 USB 相机 RGB 图像。
    相机 ID、分辨率、目标帧率、AprilTag 参数、超时和 Topic
    均通过 ROS 2 YAML 参数配置。

输出：
    /vision_double/target_features
        std_msgs/msg/Float64MultiArray
        [validL, validR, uL, vL, uR, vR, scaleL, scaleR]

    /vision_double/zoom_position_steps
        std_msgs/msg/Float64MultiArray
        [0.0, 0.0]

调用：
    ros2 run velocity_servo_tag vision_double_node \
      --ros-args --params-file <velocity_servo_tag.yaml>

方法：
    左右相机各使用一个独立采集线程和 AprilTagDetector，避免顺序读取
    导致相互阻塞。ROS 2 定时器以配置频率读取线程安全快照，检查检测
    新鲜度和左右时间差后发布。scale 定义为标签四角像素面积的平方根。
"""

import math
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from velocity_servo_tag.vision.apriltag_detector import (
    AprilTagDetector,
)
from velocity_servo_tag.vision.camera import USBCamera
from velocity_servo_tag.vision.stereo_features import (
    CameraFeature,
    ZOOM_POSITION_PLACEHOLDER,
    build_stereo_feature_vector,
    compute_tag_scale,
)


class StereoCameraWorker:
    """在独立线程中持续采集并检测一侧相机。"""

    def __init__(
        self,
        side_name,
        camera_index,
        camera_width,
        camera_height,
        camera_fps,
        reopen_delay_sec,
        detector_parameters,
        logger,
        keep_display_image,
    ):
        self.side_name = str(side_name)
        self.camera_index = int(camera_index)
        self.camera_width = int(camera_width)
        self.camera_height = int(camera_height)
        self.camera_fps = float(camera_fps)
        self.reopen_delay_sec = float(
            reopen_delay_sec
        )
        self.logger = logger
        self.keep_display_image = bool(
            keep_display_image
        )

        self.detector = AprilTagDetector(
            **detector_parameters
        )

        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread = None

        self._feature = CameraFeature()
        self._image_rgb = None
        self._corners = None

        self._target_was_detected = False
    def start(self):
        """启动后台采集线程。"""

        if self._thread is not None:
            raise RuntimeError(
                f"{self.side_name} camera worker "
                "has already started."
            )

        self._thread = threading.Thread(
            target=self._run,
            name=(
                f"vision_double_"
                f"{self.side_name}_camera"
            ),
            daemon=True,
        )
        self._thread.start()

    def stop(self, timeout_sec=2.0):
        """请求线程停止并等待相机资源释放。"""

        self._stop_event.set()

        if self._thread is not None:
            self._thread.join(
                timeout=float(timeout_sec)
            )

            if self._thread.is_alive():
                self.logger.warning(
                    f"{self.side_name} camera worker "
                    "did not stop before timeout."
                )

    def snapshot(self, include_image=False):
        """返回最近一次检测结果和可选显示图像的副本。"""

        with self._lock:
            feature = self._feature

            image_rgb = None
            corners = None

            if include_image:
                if self._image_rgb is not None:
                    image_rgb = self._image_rgb.copy()

                if self._corners is not None:
                    corners = self._corners.copy()

        return feature, image_rgb, corners

    def _store_result(
        self,
        feature,
        image_rgb=None,
        corners=None,
    ):
        """原子更新该相机的最近检测快照。"""

        with self._lock:
            self._feature = feature

            if self.keep_display_image:
                self._image_rgb = (
                    None
                    if image_rgb is None
                    else image_rgb.copy()
                )
                self._corners = (
                    None
                    if corners is None
                    else corners.copy()
                )

    def _publish_invalid_snapshot(
        self,
        stamp,
        image_rgb=None,
    ):
        """在相机或检测无效时保存零特征。"""

        self._store_result(
            CameraFeature(stamp=float(stamp)),
            image_rgb=image_rgb,
            corners=None,
        )

    def _run(self):
        """循环打开相机、采集图像并检测 AprilTag。"""

        while not self._stop_event.is_set():
            camera = None

            try:
                camera = USBCamera(
                    camera_index=self.camera_index,
                    width=self.camera_width,
                    height=self.camera_height,
                    fps=int(round(self.camera_fps)),
                )

                self.logger.info(
                    f"{self.side_name} camera opened: "
                    f"index={self.camera_index}."
                )

                self._capture_loop(camera)

            except Exception as error:
                if not self._stop_event.is_set():
                    self.logger.error(
                        f"{self.side_name} camera error: "
                        f"{error}. Retrying in "
                        f"{self.reopen_delay_sec:.1f} s."
                    )

                self._publish_invalid_snapshot(
                    time.monotonic()
                )

            finally:
                if camera is not None:
                    camera.release()

                self.detector.reset_tracking_state()

            self._stop_event.wait(
                self.reopen_delay_sec
            )

    def _capture_loop(self, camera):
        """持续读取已打开的相机并更新检测结果。"""

        frame_count = 0
        measurement_start = time.monotonic()

        while not self._stop_event.is_set():
            image_rgb = camera.read()
            capture_stamp = time.monotonic()

            if image_rgb is None:
                self.detector.reset_tracking_state()
                self._publish_invalid_snapshot(
                    capture_stamp
                )

                if self._target_was_detected:
                    self.logger.warning(
                        f"{self.side_name} camera image "
                        "unavailable; target invalid."
                    )

                self._target_was_detected = False
                self._stop_event.wait(0.01)
                continue

            frame_count += 1
            feature_uv = self.detector.detect(
                image_rgb
            )

            if feature_uv is None:
                self._publish_invalid_snapshot(
                    capture_stamp,
                    image_rgb=image_rgb,
                )

                if self._target_was_detected:
                    self.logger.warning(
                        f"{self.side_name} target lost."
                    )

                self._target_was_detected = False

            else:
                corners = np.asarray(
                    self.detector.last_corners,
                    dtype=float,
                )

                try:
                    scale = compute_tag_scale(
                        corners
                    )
                except ValueError as error:
                    self.logger.warning(
                        f"{self.side_name} invalid tag "
                        f"corners: {error}"
                    )
                    self._publish_invalid_snapshot(
                        capture_stamp,
                        image_rgb=image_rgb,
                    )
                    self._target_was_detected = False
                    continue

                camera_feature = CameraFeature(
                    valid=True,
                    u=float(feature_uv[0]),
                    v=float(feature_uv[1]),
                    scale=float(scale),
                    stamp=capture_stamp,
                )

                self._store_result(
                    camera_feature,
                    image_rgb=image_rgb,
                    corners=corners,
                )

                if not self._target_was_detected:
                    self.logger.info(
                        f"{self.side_name} target detected."
                    )

                self._target_was_detected = True

            measurement_elapsed = (
                capture_stamp - measurement_start
            )

            if measurement_elapsed >= 10.0:
                actual_fps = (
                    frame_count / measurement_elapsed
                )
                self.logger.info(
                    f"{self.side_name} camera "
                    f"processing FPS: {actual_fps:.2f}."
                )
                frame_count = 0
                measurement_start = capture_stamp


class VisionDoubleNode(Node):
    """发布 Stage 1 双目 AprilTag 特征和 Zoom 零占位。"""

    def __init__(self):
        """读取 ROS 2 参数并启动两个独立相机工作线程。"""

        super().__init__("vision_double_node")

        self._declare_parameters()
        self._read_parameters()
        self._validate_parameters()

        detector_parameters = {
            "tag_family": self.tag_family,
            "target_tag_id": self.target_tag_id,
            "detector_threads": (
                self.detector_threads_per_camera
            ),
            "quad_decimate": self.quad_decimate,
            "quad_sigma": self.quad_sigma,
            "refine_edges": self.refine_edges,
            "decode_sharpening": (
                self.decode_sharpening
            ),
            "uv_filter_alpha": self.uv_filter_alpha,
        }

        self.left_worker = StereoCameraWorker(
            side_name="left",
            camera_index=self.left_camera_index,
            camera_width=self.camera_width,
            camera_height=self.camera_height,
            camera_fps=self.camera_fps,
            reopen_delay_sec=self.reopen_delay_sec,
            detector_parameters=detector_parameters,
            logger=self.get_logger(),
            keep_display_image=self.show_window,
        )
        self.right_worker = StereoCameraWorker(
            side_name="right",
            camera_index=self.right_camera_index,
            camera_width=self.camera_width,
            camera_height=self.camera_height,
            camera_fps=self.camera_fps,
            reopen_delay_sec=self.reopen_delay_sec,
            detector_parameters=detector_parameters,
            logger=self.get_logger(),
            keep_display_image=self.show_window,
        )

        self.feature_publisher = self.create_publisher(
            Float64MultiArray,
            self.target_features_topic,
            1,
        )
        self.zoom_position_publisher = (
            self.create_publisher(
                Float64MultiArray,
                self.zoom_position_topic,
                1,
            )
        )

        self.last_valid_state = None
        self.window_failed = False

        self.publish_timer = self.create_timer(
            1.0 / self.publish_rate_hz,
            self._publish_callback,
        )

        self.left_worker.start()
        self.right_worker.start()

        self.get_logger().info(
            "VisionDoubleNode started | "
            f"left_camera={self.left_camera_index} | "
            f"right_camera={self.right_camera_index} | "
            f"image={self.camera_width}x"
            f"{self.camera_height}@{self.camera_fps:.1f} Hz | "
            f"publish={self.publish_rate_hz:.1f} Hz | "
            f"features={self.target_features_topic} | "
            f"zoom_placeholder={self.zoom_position_topic}"
        )

    def _declare_parameters(self):
        """声明双目相机、检测器和 ROS 2 参数。"""

        self.declare_parameter("left_camera_index", 0)
        self.declare_parameter("right_camera_index", 1)
        self.declare_parameter("camera_width", 640)
        self.declare_parameter("camera_height", 480)
        self.declare_parameter("camera_fps", 60.0)
        self.declare_parameter("publish_rate_hz", 60.0)

        self.declare_parameter("tag_family", "tag36h11")
        self.declare_parameter("target_tag_id", 0)
        self.declare_parameter(
            "detector_threads_per_camera",
            2,
        )
        self.declare_parameter("quad_decimate", 1.0)
        self.declare_parameter("quad_sigma", 0.0)
        self.declare_parameter("refine_edges", True)
        self.declare_parameter(
            "decode_sharpening",
            0.25,
        )
        self.declare_parameter("uv_filter_alpha", 0.4)

        self.declare_parameter(
            "feature_timeout_sec",
            0.10,
        )
        self.declare_parameter(
            "max_pair_skew_sec",
            0.05,
        )
        self.declare_parameter(
            "camera_reopen_delay_sec",
            2.0,
        )

        self.declare_parameter(
            "target_features_topic",
            "/vision_double/target_features",
        )
        self.declare_parameter(
            "zoom_position_topic",
            "/vision_double/zoom_position_steps",
        )
        self.declare_parameter("show_window", True)

    def _read_parameters(self):
        """从节点参数服务器读取全部配置。"""

        self.left_camera_index = int(
            self.get_parameter(
                "left_camera_index"
            ).value
        )
        self.right_camera_index = int(
            self.get_parameter(
                "right_camera_index"
            ).value
        )
        self.camera_width = int(
            self.get_parameter("camera_width").value
        )
        self.camera_height = int(
            self.get_parameter("camera_height").value
        )
        self.camera_fps = float(
            self.get_parameter("camera_fps").value
        )
        self.publish_rate_hz = float(
            self.get_parameter("publish_rate_hz").value
        )

        self.tag_family = str(
            self.get_parameter("tag_family").value
        )
        self.target_tag_id = int(
            self.get_parameter("target_tag_id").value
        )
        self.detector_threads_per_camera = int(
            self.get_parameter(
                "detector_threads_per_camera"
            ).value
        )
        self.quad_decimate = float(
            self.get_parameter("quad_decimate").value
        )
        self.quad_sigma = float(
            self.get_parameter("quad_sigma").value
        )
        self.refine_edges = bool(
            self.get_parameter("refine_edges").value
        )
        self.decode_sharpening = float(
            self.get_parameter(
                "decode_sharpening"
            ).value
        )
        self.uv_filter_alpha = float(
            self.get_parameter(
                "uv_filter_alpha"
            ).value
        )

        self.feature_timeout_sec = float(
            self.get_parameter(
                "feature_timeout_sec"
            ).value
        )
        self.max_pair_skew_sec = float(
            self.get_parameter(
                "max_pair_skew_sec"
            ).value
        )
        self.reopen_delay_sec = float(
            self.get_parameter(
                "camera_reopen_delay_sec"
            ).value
        )

        self.target_features_topic = str(
            self.get_parameter(
                "target_features_topic"
            ).value
        )
        self.zoom_position_topic = str(
            self.get_parameter(
                "zoom_position_topic"
            ).value
        )
        self.show_window = bool(
            self.get_parameter("show_window").value
        )

    def _validate_parameters(self):
        """检查相机、检测器、超时和 Topic 参数。"""

        if self.left_camera_index < 0:
            raise ValueError(
                "left_camera_index 不能小于 0。"
            )

        if self.right_camera_index < 0:
            raise ValueError(
                "right_camera_index 不能小于 0。"
            )

        if (
            self.left_camera_index
            == self.right_camera_index
        ):
            raise ValueError(
                "左右相机不能使用相同 camera index。"
            )

        positive_parameters = {
            "camera_width": self.camera_width,
            "camera_height": self.camera_height,
            "camera_fps": self.camera_fps,
            "publish_rate_hz": self.publish_rate_hz,
            "detector_threads_per_camera": (
                self.detector_threads_per_camera
            ),
            "feature_timeout_sec": (
                self.feature_timeout_sec
            ),
            "camera_reopen_delay_sec": (
                self.reopen_delay_sec
            ),
        }

        for name, value in positive_parameters.items():
            if (
                not math.isfinite(float(value))
                or value <= 0
            ):
                raise ValueError(
                    f"{name} 必须为有限正数。"
                )

        if (
            not math.isfinite(
                self.max_pair_skew_sec
            )
            or self.max_pair_skew_sec < 0.0
        ):
            raise ValueError(
                "max_pair_skew_sec必须为有限非负数。"
            )

        if not self.target_features_topic:
            raise ValueError(
                "target_features_topic 不能为空。"
            )

        if not self.zoom_position_topic:
            raise ValueError(
                "zoom_position_topic 不能为空。"
            )

    def _publish_callback(self):
        """发布双目特征和固定 Zoom 位置占位值。"""

        now = time.monotonic()

        left_feature, left_image, left_corners = (
            self.left_worker.snapshot(
                include_image=self.show_window
            )
        )
        right_feature, right_image, right_corners = (
            self.right_worker.snapshot(
                include_image=self.show_window
            )
        )

        feature_vector = build_stereo_feature_vector(
            left_feature=left_feature,
            right_feature=right_feature,
            now=now,
            timeout=self.feature_timeout_sec,
            max_pair_skew=self.max_pair_skew_sec,
        )

        feature_message = Float64MultiArray()
        feature_message.data = feature_vector.tolist()
        self.feature_publisher.publish(feature_message)

        zoom_message = Float64MultiArray()
        zoom_message.data = list(
            ZOOM_POSITION_PLACEHOLDER
        )
        self.zoom_position_publisher.publish(
            zoom_message
        )

        valid_state = (
            bool(feature_vector[0]),
            bool(feature_vector[1]),
        )

        if valid_state != self.last_valid_state:
            self.get_logger().info(
                "Stereo validity changed | "
                f"left={int(valid_state[0])} | "
                f"right={int(valid_state[1])}"
            )
            self.last_valid_state = valid_state

        if self.show_window and not self.window_failed:
            self._draw_stereo_view(
                left_image=left_image,
                right_image=right_image,
                left_feature=left_feature,
                right_feature=right_feature,
                left_corners=left_corners,
                right_corners=right_corners,
                left_valid=valid_state[0],
                right_valid=valid_state[1],
            )

    def _draw_camera_view(
        self,
        image_rgb,
        feature,
        corners,
        valid,
        label,
    ):
        """生成一侧相机带检测标记的预览图。"""

        if image_rgb is None:
            display_image = np.zeros(
                (
                    self.camera_height,
                    self.camera_width,
                    3,
                ),
                dtype=np.uint8,
            )
        else:
            display_image = cv2.cvtColor(
                image_rgb,
                cv2.COLOR_RGB2BGR,
            )

            if (
                display_image.shape[1]
                != self.camera_width
                or display_image.shape[0]
                != self.camera_height
            ):
                display_image = cv2.resize(
                    display_image,
                    (
                        self.camera_width,
                        self.camera_height,
                    ),
                )

        color = (0, 255, 0) if valid else (0, 0, 255)
        status_text = "valid" if valid else "invalid/stale"

        if (
            valid
            and corners is not None
            and corners.shape == (4, 2)
        ):
            integer_corners = np.rint(
                corners
            ).astype(np.int32)

            for corner_index in range(4):
                start_point = tuple(
                    int(value)
                    for value in integer_corners[
                        corner_index
                    ]
                )
                end_point = tuple(
                    int(value)
                    for value in integer_corners[
                        (corner_index + 1) % 4
                    ]
                )
                cv2.line(
                    display_image,
                    start_point,
                    end_point,
                    color,
                    2,
                )

            cv2.circle(
                display_image,
                (
                    int(round(feature.u)),
                    int(round(feature.v)),
                ),
                4,
                (0, 0, 255),
                -1,
            )

        cv2.putText(
            display_image,
            f"{label}: {status_text}",
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            color,
            2,
        )

        if valid:
            cv2.putText(
                display_image,
                (
                    f"u={feature.u:.1f}, "
                    f"v={feature.v:.1f}, "
                    f"scale={feature.scale:.1f}"
                ),
                (20, 70),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                color,
                2,
            )

        return display_image

    def _draw_stereo_view(
        self,
        left_image,
        right_image,
        left_feature,
        right_feature,
        left_corners,
        right_corners,
        left_valid,
        right_valid,
    ):
        """显示左右相机预览，按 q 安全退出。"""

        try:
            left_view = self._draw_camera_view(
                image_rgb=left_image,
                feature=left_feature,
                corners=left_corners,
                valid=left_valid,
                label="LEFT",
            )
            right_view = self._draw_camera_view(
                image_rgb=right_image,
                feature=right_feature,
                corners=right_corners,
                valid=right_valid,
                label="RIGHT",
            )

            stereo_view = np.hstack(
                [left_view, right_view]
            )
            cv2.imshow(
                "Vision Double AprilTag",
                stereo_view,
            )

            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                rclpy.shutdown()

        except cv2.error as error:
            self.window_failed = True
            self.get_logger().error(
                "OpenCV preview disabled after error: "
                f"{error}"
            )

    def _publish_safe_shutdown_messages(self):
        """退出前发布一次无效视觉特征和 Zoom 零位置。"""

        feature_message = Float64MultiArray()
        feature_message.data = [0.0] * 8
        self.feature_publisher.publish(feature_message)

        zoom_message = Float64MultiArray()
        zoom_message.data = list(
            ZOOM_POSITION_PLACEHOLDER
        )
        self.zoom_position_publisher.publish(
            zoom_message
        )

    def destroy_node(self):
        """停止线程、释放相机和窗口，然后销毁 ROS 2 节点。"""

        if rclpy.ok():
            self._publish_safe_shutdown_messages()

        self.left_worker.stop()
        self.right_worker.stop()

        if self.show_window:
            try:
                cv2.destroyAllWindows()
            except cv2.error as error:
                self.get_logger().warning(
                    "OpenCV windows could not be closed: "
                    f"{error}"
                )

        return super().destroy_node()


def main(args=None):
    """ROS 2 节点运行入口。"""

    rclpy.init(args=args)
    node = None

    try:
        node = VisionDoubleNode()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
