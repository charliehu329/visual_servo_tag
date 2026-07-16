import numpy as np

from velocity_servo_tag.safety import limit_joint_velocity


def test_joint_velocity_scaling_preserves_direction():
    q_dot = np.asarray([0.10, -0.04, 0.02, 0.0, 0.0, 0.0, 0.0])
    limits = np.asarray([0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05])

    result = limit_joint_velocity(q_dot, limits)

    np.testing.assert_allclose(result, q_dot * 0.5)
    assert np.all(np.abs(result) <= limits)


def test_joint_velocity_below_limits_is_unchanged():
    q_dot = np.asarray([0.01, -0.02, 0.03, 0.0, 0.0, 0.0, 0.0])

    result = limit_joint_velocity(q_dot, [0.05] * 7)

    np.testing.assert_allclose(result, q_dot)
