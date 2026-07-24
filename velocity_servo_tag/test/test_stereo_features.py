#!/usr/bin/env python3
"""
test_stereo_features.py

功能：
    验证双目AprilTag像素尺度、超时、双目时间差和Zoom占位值。

输入：
    人工构造的四角像素坐标与左右CameraFeature。

输出：
    unittest测试结果。

调用：
    python3 -m unittest discover -v -s test -p 'test_stereo_features.py'

方法：
    使用已知正方形面积和确定时间戳，对纯算法函数进行无相机、无ROS测试。
"""

import unittest

import numpy as np

from velocity_servo_tag.vision.stereo_features import (
    CameraFeature,
    ZOOM_POSITION_PLACEHOLDER,
    build_stereo_feature_vector,
    compute_tag_scale,
)


class StereoFeaturesTest(unittest.TestCase):
    """双目特征纯算法单元测试。"""

    def test_compute_tag_scale_for_square(self):
        """边长20 pixel的正方形应返回scale=20。"""

        corners = np.asarray(
            [
                [10.0, 20.0],
                [30.0, 20.0],
                [30.0, 40.0],
                [10.0, 40.0],
            ]
        )

        self.assertAlmostEqual(
            compute_tag_scale(corners),
            20.0,
        )

    def test_compute_tag_scale_rejects_zero_area(self):
        """共线角点不能形成有效AprilTag像素面积。"""

        corners = np.asarray(
            [
                [0.0, 0.0],
                [1.0, 0.0],
                [2.0, 0.0],
                [3.0, 0.0],
            ]
        )

        with self.assertRaises(ValueError):
            compute_tag_scale(corners)

    def test_build_stereo_feature_vector_for_fresh_pair(
        self,
    ):
        """新鲜且同步的左右结果应完整写入8维向量。"""

        left = CameraFeature(
            valid=True,
            u=100.0,
            v=110.0,
            scale=25.0,
            stamp=9.98,
        )
        right = CameraFeature(
            valid=True,
            u=200.0,
            v=210.0,
            scale=24.0,
            stamp=9.99,
        )

        output = build_stereo_feature_vector(
            left,
            right,
            now=10.0,
            timeout=0.10,
            max_pair_skew=0.05,
        )

        np.testing.assert_allclose(
            output,
            [
                1.0,
                1.0,
                100.0,
                110.0,
                200.0,
                210.0,
                25.0,
                24.0,
            ],
        )

    def test_stale_feature_is_zeroed(self):
        """超过超时的相机结果应置valid=0并清零数据。"""

        left = CameraFeature(
            valid=True,
            u=100.0,
            v=110.0,
            scale=25.0,
            stamp=9.0,
        )
        right = CameraFeature(
            valid=True,
            u=200.0,
            v=210.0,
            scale=24.0,
            stamp=9.99,
        )

        output = build_stereo_feature_vector(
            left,
            right,
            now=10.0,
            timeout=0.10,
            max_pair_skew=0.05,
        )

        np.testing.assert_allclose(
            output,
            [
                0.0,
                1.0,
                0.0,
                0.0,
                200.0,
                210.0,
                0.0,
                24.0,
            ],
        )

    def test_older_side_is_zeroed_for_large_pair_skew(
        self,
    ):
        """左右时间差超限时只保留较新一侧。"""

        left = CameraFeature(
            valid=True,
            u=100.0,
            v=110.0,
            scale=25.0,
            stamp=9.92,
        )
        right = CameraFeature(
            valid=True,
            u=200.0,
            v=210.0,
            scale=24.0,
            stamp=9.99,
        )

        output = build_stereo_feature_vector(
            left,
            right,
            now=10.0,
            timeout=0.10,
            max_pair_skew=0.05,
        )

        np.testing.assert_allclose(
            output,
            [
                0.0,
                1.0,
                0.0,
                0.0,
                200.0,
                210.0,
                0.0,
                24.0,
            ],
        )

    def test_zoom_position_placeholder_is_two_zeros(self):
        """Stage 1 Zoom位置占位必须严格为两个零。"""

        self.assertEqual(
            ZOOM_POSITION_PLACEHOLDER,
            (0.0, 0.0),
        )


if __name__ == "__main__":
    unittest.main()
