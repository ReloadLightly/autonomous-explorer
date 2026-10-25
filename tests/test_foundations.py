"""Physics, reference feasibility and algebraic controller properties."""

import numpy as np
import pytest

from autonomous_explorer.control import Feedback, InvalidSpeed, OpenLoop, invert_acceleration
from autonomous_explorer.model import State, constant_turn_exact, derivative, step, wrap_angle
from autonomous_explorer.reference import CubicReference


@pytest.fixture
def reference():
    return CubicReference(State(0, 0, 0.5, -np.pi / 2), State(5, 5, 0.5, -np.pi / 2), 25)


def test_named_state_and_derivative():
    state = State(1, 2, 0.8, np.pi / 2)
    np.testing.assert_allclose(state.array(), [1, 2, 0.8, np.pi / 2])
    np.testing.assert_allclose(
        derivative(state.array(), [0.2, -0.4]), [0, 0.8, 0.2, -0.4], atol=1e-15
    )
    np.testing.assert_allclose(state.velocity, [0, 0.8], atol=1e-15)
    assert State.from_array(state.array()) == state


@pytest.mark.parametrize("theta", [0, -np.pi / 2, 0.7])
def test_straight_motion_with_acceleration(theta):
    state = State(0.3, -0.1, 0.5, theta)
    actual = step(state.array(), [0.2, 0], 2)
    distance = 0.5 * 2 + 0.5 * 0.2 * 2**2
    expected = state.array() + [distance * np.cos(theta), distance * np.sin(theta), 0.4, 0]
    np.testing.assert_allclose(actual, expected, atol=1e-14)


def test_constant_turn_and_fourth_order_convergence():
    initial = State(0.4, -0.3, 0.8, 0.3)
    exact = constant_turn_exact(initial, 0.8, 0.7, [4.0])[-1]
    errors = []
    for dt in [0.5, 0.25, 0.125]:
        state = initial.array()
        for _ in range(round(4 / dt)):
            state = step(state, [0, 0.7], dt)
        errors.append(np.linalg.norm(state[:2] - exact[:2]))
        np.testing.assert_allclose(state[2:], exact[2:], atol=1e-13)
    ratios = np.array(errors[:-1]) / errors[1:]
    assert np.all((ratios > 15) & (ratios < 17))
    assert errors[-1] < 1e-7


def test_analytical_straight_limit():
    initial = State(1, 2, 0.5, 0)
    np.testing.assert_allclose(
        constant_turn_exact(initial, 0.5, 0, [0, 2]), [[1, 2, 0.5, 0], [2, 2, 0.5, 0]]
    )


def test_cubic_endpoints_and_derivatives(reference):
    for t, position in [(0, [0, 0]), (25, [5, 5])]:
        sample = reference.sample(t)
        np.testing.assert_allclose([sample.state.x, sample.state.y], position, atol=1e-13)
        np.testing.assert_allclose(sample.velocity, [0, -0.5], atol=1e-14)
        assert abs(sample.state.v - 0.5) < 1e-14
        assert abs(wrap_angle(sample.state.theta + np.pi / 2)) < 1e-13
    t, h = 9.0, 1e-4
    left, right, center = reference.sample(t - h), reference.sample(t + h), reference.sample(t)
    numerical_velocity = (right.state.array()[:2] - left.state.array()[:2]) / (2 * h)
    numerical_acceleration = (right.velocity - left.velocity) / (2 * h)
    np.testing.assert_allclose(numerical_velocity, center.velocity, atol=1e-10)
    np.testing.assert_allclose(numerical_acceleration, center.acceleration, atol=1e-10)


def test_reference_speed_checked_between_samples(reference):
    minimum, t = reference.minimum_speed()
    dense = np.array([reference.sample(ti).state.v for ti in np.linspace(0, 25, 10001)])
    assert minimum > 0.1
    assert minimum <= dense.min() + 1e-12
    assert abs(reference.sample(t).state.v - minimum) < 1e-12


@pytest.mark.parametrize("theta,v", [(0, 0.2), (np.pi / 2, 0.5), (-2.5, 1.2)])
def test_inversion_recovers_cartesian_acceleration(theta, v):
    state = State(0, 0, v, theta)
    desired = np.array([0.4, -0.7])
    control = invert_acceleration(state, desired)
    c, s = np.cos(theta), np.sin(theta)
    jacobian = np.array([[c, -v * s], [s, v * c]])
    np.testing.assert_allclose(jacobian @ control, desired, atol=1e-14)


def test_zero_error_feedback_is_feedforward(reference):
    tracker = Feedback()
    for t in np.linspace(0, 25, 31):
        sample = reference.sample(t)
        np.testing.assert_allclose(
            tracker.command(sample.state, sample),
            invert_acceleration(sample.state, sample.acceleration),
            atol=1e-14,
        )


def test_open_loop_has_no_state_input():
    commands = np.array([[0.2, -0.3], [0.4, 0.1]])
    controller = OpenLoop(commands)
    commands[:] = 999  # Construction copied the precomputed controls.
    np.testing.assert_allclose(controller.command(0), [0.2, -0.3])
    u = controller.command(0)
    u[:] = 0
    np.testing.assert_allclose(controller.command(0), [0.2, -0.3])
    with pytest.raises(TypeError):
        controller.command(0, State(1, 2, 3, 4))


@pytest.mark.parametrize("speed", [0, 1e-4, -0.5, np.nan])
def test_singular_and_out_of_domain_speeds_are_explicit(speed):
    with pytest.raises(InvalidSpeed):
        invert_acceleration(State(0, 0, speed, 0), [1, 0])
    assert np.all(np.isfinite(invert_acceleration(State(0, 0, 1.01e-4, 0), [0, 1])))


def test_wrapped_heading_error():
    assert abs(wrap_angle((-np.pi + 0.01) - (np.pi - 0.01)) - 0.02) < 1e-12
    np.testing.assert_allclose(wrap_angle([0, 2 * np.pi, 3 * np.pi]), [0, 0, -np.pi], atol=1e-14)
