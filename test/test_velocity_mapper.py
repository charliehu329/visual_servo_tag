import numpy as np

from velocity_servo_tag.velocity_mapper import (
    cartesian_velocity_to_joint_velocity,
)


def test_damped_mapper_returns_finite_seven_vector():
    jacobian = np.hstack((np.eye(6), np.zeros((6, 1))))
    velocity = np.asarray([0.01, -0.02, 0.03, 0.1, -0.2, 0.3])

    result = cartesian_velocity_to_joint_velocity(
        velocity,
        jacobian,
        damping=0.02,
    )

    assert result.shape == (7,)
    assert np.all(np.isfinite(result))
    np.testing.assert_allclose(
        result[:6],
        velocity / (1.0 + 0.02 ** 2),
    )
    assert result[6] == 0.0
