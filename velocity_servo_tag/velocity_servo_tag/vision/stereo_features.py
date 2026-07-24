#!/usr/bin/env python3
"""
stereo_features.py

功能：
    提供双目 AprilTag 特征的纯算法处理，不访问相机，也不依赖 ROS 2。

输入：
    AprilTag 四个像素角点、左右相机最近一次检测结果、当前单调时钟、
    检测超时时间和允许的双目时间差。

输出：
    AprilTag 像素尺度，以及 Simulink Stage 1 使用的 8 维双目特征：
    [validL, validR, uL, vL, uR, vR, scaleL, scaleR]。

调用：
    compute_tag_scale(corners)
    build_stereo_feature_vector(left, right, now, timeout, max_pair_skew)

方法：
    使用鞋带公式计算四角多边形像素面积，并取面积平方根作为尺度。
    检测结果超时后置为无效；左右结果时间差过大时，将较旧一侧置为无效。
"""

from dataclasses import dataclass
import math

import numpy as np


ZOOM_POSITION_PLACEHOLDER = (0.0, 0.0)


@dataclass(frozen=True)
class CameraFeature:
    """一侧相机最近一次 AprilTag 检测结果。"""

    valid: bool = False
    u: float = 0.0
    v: float = 0.0
    scale: float = 0.0
    stamp: float = 0.0


def compute_tag_scale(corners):
    """
    根据四个像素角点计算 sqrt(area)，单位为 pixel。
    """

    points = np.asarray(
        corners,
        dtype=float,
    )

    if points.shape != (4, 2):
        raise ValueError(
            "corners must have shape (4, 2)."
        )

    if not np.all(np.isfinite(points)):
        raise ValueError(
            "corners contain nan or inf."
        )

    x_coordinates = points[:, 0]
    y_coordinates = points[:, 1]

    area = 0.5 * abs(
        float(
            np.dot(
                x_coordinates,
                np.roll(y_coordinates, -1),
            )
            - np.dot(
                y_coordinates,
                np.roll(x_coordinates, -1),
            )
        )
    )

    if area <= 0.0:
        raise ValueError(
            "corners describe a zero-area polygon."
        )

    return math.sqrt(area)


def is_feature_fresh(feature, now, timeout):
    """检查检测值、时间戳和新鲜度是否合法。"""

    values = (
        feature.u,
        feature.v,
        feature.scale,
        feature.stamp,
    )

    if not feature.valid:
        return False

    if not all(math.isfinite(value) for value in values):
        return False

    if feature.scale <= 0.0:
        return False

    age = now - feature.stamp

    return 0.0 <= age <= timeout


def build_stereo_feature_vector(
    left_feature,
    right_feature,
    now,
    timeout,
    max_pair_skew,
):
    """
    生成 Stage 1 使用的 8 维双目特征向量。
    """

    if not math.isfinite(now):
        raise ValueError("now must be finite.")

    if not math.isfinite(timeout) or timeout <= 0.0:
        raise ValueError("timeout must be positive.")

    if (
        not math.isfinite(max_pair_skew)
        or max_pair_skew < 0.0
    ):
        raise ValueError(
            "max_pair_skew must be non-negative."
        )

    left_valid = is_feature_fresh(
        left_feature,
        now,
        timeout,
    )
    right_valid = is_feature_fresh(
        right_feature,
        now,
        timeout,
    )

    if left_valid and right_valid:
        pair_skew = abs(
            left_feature.stamp
            - right_feature.stamp
        )

        if pair_skew > max_pair_skew:
            if left_feature.stamp < right_feature.stamp:
                left_valid = False
            else:
                right_valid = False

    output = np.zeros(8, dtype=float)
    output[0] = float(left_valid)
    output[1] = float(right_valid)

    if left_valid:
        output[2] = left_feature.u
        output[3] = left_feature.v
        output[6] = left_feature.scale

    if right_valid:
        output[4] = right_feature.u
        output[5] = right_feature.v
        output[7] = right_feature.scale

    return output
