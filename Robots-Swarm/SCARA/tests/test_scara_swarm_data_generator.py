"""Controller regression checks that do not require launching Webots."""

import contextlib
import importlib.util
import io
import math
from pathlib import Path
import unittest
from unittest.mock import patch


CONTROLLER = (
    Path(__file__).resolve().parents[1]
    / "controllers/scara-swarm-data-generator/scara-swarm-data-generator.py"
)
SPEC = importlib.util.spec_from_file_location("scara_generator", CONTROLLER)
generator = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(generator)


class FakeMotor:
    def __init__(self):
        self.targets = []

    def setVelocity(self, value):
        self.velocity = value

    def setPosition(self, value):
        self.targets.append(value)


class FakeSensor:
    def __init__(self, robot, joint=None):
        self.robot = robot
        self.joint = joint

    def enable(self, timestep):
        pass

    def getValues(self):
        return self.robot.frame[0]

    def getValue(self):
        return self.robot.frame[1][self.joint]


IDENTITY = (1, 0, 0, 0, 1, 0, 0, 0, 1)


class FakeBase:
    def __init__(self, position=(0, 0, 0), orientation=IDENTITY):
        self.position = position
        self.orientation = orientation

    def getPosition(self):
        return self.position

    def getOrientation(self):
        return self.orientation


class FakeRobot:
    def __init__(self, frames=None, max_steps=None):
        self.frames = frames or [((0, 0, 0.4), (0,) * 4)]
        self.frame = self.frames[0]
        self.steps = 0
        self.max_steps = max_steps
        self.motors = [FakeMotor() for _ in range(4)]
        self.sensors = [FakeSensor(self, i) for i in range(4)]
        self.gps = FakeSensor(self)
        self.base = FakeBase()
        self.supervisor = True

    def getSupervisor(self):
        return self.supervisor

    def getSelf(self):
        return self.base

    def getBasicTimeStep(self):
        return 8

    def getName(self):
        return "test-arm"

    def getDevice(self, name):
        if name == "gps":
            return self.gps
        motor_names = ("base_arm_motor", "arm_motor", "shaft_linear_motor", "shaft_rotation_motor")
        sensor_names = ("base_arm_position", "arm_position", "shaft_linear_position", "shaft_rotation_position")
        if name in motor_names:
            return self.motors[motor_names.index(name)]
        if name in sensor_names:
            return self.sensors[sensor_names.index(name)]
        return None

    def step(self, timestep):
        if self.max_steps is not None and self.steps >= self.max_steps:
            return -1
        self.frame = self.frames[min(self.steps, len(self.frames) - 1)]
        self.steps += 1
        return 0


class CoordinateFrameTests(unittest.TestCase):
    def assertPositionAlmostEqual(self, actual, expected):
        for a, e in zip(actual, expected):
            self.assertAlmostEqual(a, e, places=12)

    def test_base_height_and_grid_offsets_do_not_change_local_coordinates(self):
        expected = (0.1, -0.25, 0.4)
        for base in ((0, 0, 0), (3, -1.5, 0.2), (-3, 3, 0.8)):
            with self.subTest(base=base):
                world = tuple(a + b for a, b in zip(expected, base))
                self.assertPositionAlmostEqual(
                    generator.position_in_base_frame(world, base, IDENTITY), expected
                )

    def test_base_rotation_is_removed_as_well_as_translation(self):
        # A +90 degree yaw maps local (0.1, 0.2, 0.4) to world offset (-0.2, 0.1, 0.4).
        rotation = (0, -1, 0, 1, 0, 0, 0, 0, 1)
        self.assertPositionAlmostEqual(
            generator.position_in_base_frame((2.8, -1.4, 0.6), (3, -1.5, 0.2), rotation),
            (0.1, 0.2, 0.4),
        )

    def test_tilted_base_uses_all_three_local_axes(self):
        # A +90 degree roll maps local (0.1, 0.2, 0.4) to world offset (0.1, -0.4, 0.2).
        rotation = (1, 0, 0, 0, 0, -1, 0, 1, 0)
        self.assertPositionAlmostEqual(
            generator.position_in_base_frame((3.1, -1.9, 0.4), (3, -1.5, 0.2), rotation),
            (0.1, 0.2, 0.4),
        )

    def test_invalid_base_pose_is_not_exported(self):
        with self.assertRaises(ValueError):
            generator.position_in_base_frame((0, 0, 0.4), (0, 0, math.nan), IDENTITY)


class SettlingTests(unittest.TestCase):
    def wait(self, robot, targets=(0,) * 4):
        return generator.wait_until_settled(
            robot, 8, robot.sensors, robot.gps, targets
        )

    def test_requires_half_a_second_at_target(self):
        robot = FakeRobot()
        status, position, joints = self.wait(robot)
        self.assertEqual(status, "settled")
        self.assertGreaterEqual((robot.steps - 1) * 8, 500)
        self.assertEqual(position, (0, 0, 0.4))
        self.assertEqual(joints, (0,) * 4)

    def test_stationary_but_wrong_joint_position_times_out(self):
        for joint in (0, 3):  # Check a moving joint and a nominally fixed wrist joint.
            with self.subTest(joint=joint):
                actual = [0] * 4
                actual[joint] = 0.1
                robot = FakeRobot([((0, 0, 0.4), tuple(actual))])
                self.assertEqual(self.wait(robot)[0], "timeout")
                self.assertEqual(robot.steps * 8, generator.MOVE_TIMEOUT_MS)

    def test_persistent_hand_jitter_has_a_deadline(self):
        frames = [((0.01 * (i % 2), 0, 0.4), (0,) * 4) for i in range(2500)]
        robot = FakeRobot(frames)
        self.assertEqual(self.wait(robot)[0], "timeout")
        self.assertEqual(robot.steps, 2500)

    def test_small_physics_jitter_can_settle(self):
        # Five micrometres per step prevented the original GPS-only loop finishing.
        frames = [((0.000005 * (i % 2), 0, 0.4), (0,) * 4) for i in range(100)]
        self.assertEqual(self.wait(FakeRobot(frames))[0], "settled")

    def test_movement_restarts_the_continuous_settling_window(self):
        frames = [((0, 0, 0.4), (0,) * 4)] * 40
        frames += [((0.01, 0, 0.4), (0,) * 4)] * 100
        robot = FakeRobot(frames)
        self.assertEqual(self.wait(robot)[0], "settled")
        self.assertGreaterEqual((robot.steps - 41) * 8, 500)

    def test_moving_joint_is_not_settled_even_when_hand_is_stationary(self):
        frames = [
            ((0, 0, 0.4), (0.002 * (i % 2), 0, 0, 0))
            for i in range(2500)
        ]
        self.assertEqual(self.wait(FakeRobot(frames))[0], "timeout")

    def test_invalid_gps_or_joint_reading_is_rejected_immediately(self):
        for frame in (
            ((math.nan, 0, 0.4), (0,) * 4),
            ((0, 0, 0.4), (0, 0, 0, math.inf)),
        ):
            with self.subTest(frame=frame):
                robot = FakeRobot([frame])
                self.assertEqual(self.wait(robot)[0], "invalid sensor readings")
                self.assertEqual(robot.steps, 1)

    def test_simulation_shutdown_returns_without_another_step(self):
        robot = FakeRobot(max_steps=0)
        self.assertEqual(self.wait(robot), ("stopped", None, None))
        self.assertEqual(robot.steps, 0)


class CollectionTests(unittest.TestCase):
    def test_ground_level_output_preserves_height_for_different_grid_positions(self):
        for base_x, base_y in ((0, 0), (3, -1.5), (-3, 3)):
            with self.subTest(base=(base_x, base_y, 0)):
                world_position = (base_x + 0.1, base_y + 0.3, 0.4)
                robot = FakeRobot([(world_position, (0,) * 4)], max_steps=70)
                robot.base = FakeBase((base_x, base_y, 0))
                output = io.StringIO()
                with patch.object(generator, "loop_value", 1), patch.object(
                    generator.random, "uniform", return_value=0
                ), contextlib.redirect_stdout(output):
                    generator.run(robot)
                self.assertIn(
                    "X: 0.100000, Y: 0.300000, Z: 0.400000", output.getvalue()
                )

    def test_output_is_base_relative_and_labelled(self):
        robot = FakeRobot([((3.1, -1.2, 0.6), (0,) * 4)], max_steps=70)
        robot.base = FakeBase((3, -1.5, 0.2))
        output = io.StringIO()
        with patch.object(generator, "loop_value", 1), patch.object(
            generator.random, "uniform", return_value=0
        ), contextlib.redirect_stdout(output):
            generator.run(robot)
        self.assertIn(
            "Scara Final Position (base frame, m) -> X: 0.100000, Y: 0.300000, Z: 0.400000",
            output.getvalue(),
        )

    def test_world_without_supervisor_fails_before_collecting_world_coordinates(self):
        robot = FakeRobot()
        robot.supervisor = False
        with self.assertRaisesRegex(RuntimeError, "Reload the updated swarm world"):
            generator.run(robot)
        self.assertTrue(all(not motor.targets for motor in robot.motors))

    def test_sample_count_is_configurable_and_completion_holds_pose(self):
        robot = FakeRobot(max_steps=140)
        output = io.StringIO()
        with patch.object(generator, "loop_value", 2), patch.object(
            generator.random, "uniform", return_value=0
        ), contextlib.redirect_stdout(output):
            generator.run(robot)
        self.assertEqual(output.getvalue().count("Scara Final Position"), 2)
        self.assertIn("Completed 2 samples", output.getvalue())
        self.assertTrue(all(len(m.targets) == 2 for m in robot.motors))
        self.assertEqual(robot.steps, 140)

    def test_timeout_gets_a_replacement_attempt_without_a_bad_sample(self):
        robot = FakeRobot(max_steps=0)
        output = io.StringIO()
        results = [
            ("timeout", (0, 0, 0.4), (0,) * 4),
            ("settled", (0.1, 0, 0.4), (0,) * 4),
        ]
        with patch.object(generator, "loop_value", 1), patch.object(
            generator, "wait_until_settled", side_effect=results
        ), contextlib.redirect_stdout(output):
            generator.run(robot)
        self.assertEqual(output.getvalue().count("Scara Final Position"), 1)
        self.assertIn("Rejected attempt 1: timeout", output.getvalue())
        self.assertIn("Completed 1 samples; rejected 1 movements", output.getvalue())
        self.assertTrue(all(len(m.targets) == 2 for m in robot.motors))

    def test_shutdown_exits_outer_collection_loop(self):
        robot = FakeRobot(max_steps=0)
        with contextlib.redirect_stdout(io.StringIO()):
            generator.run(robot)
        self.assertTrue(all(len(m.targets) == 1 for m in robot.motors))

    def test_repeated_failures_stop_with_an_explicit_error(self):
        robot = FakeRobot()
        with patch.object(
            generator, "wait_until_settled",
            return_value=("timeout", (0, 0, 0.4), (0,) * 4),
        ), contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(RuntimeError, "Simulation fault"):
                generator.run(robot)
        self.assertTrue(
            all(len(m.targets) == generator.MAX_CONSECUTIVE_FAILURES for m in robot.motors)
        )


class ScaraSpecificTests(unittest.TestCase):
    def test_slide_one_millimetre_from_target_is_rejected(self):
        robot = FakeRobot([((0.5, 0, 0.1), (0, 0, -0.099, 0))])
        status, _, _ = generator.wait_until_settled(
            robot, 8, robot.sensors, robot.gps, (0, 0, -0.1, 0)
        )
        self.assertEqual(status, "timeout")

    def test_slide_motion_uses_metres_per_second_tolerance(self):
        # Both positions are within the angle/slide error limits, but the slide
        # is moving at 0.002 m/s, which is too fast to accept a sample.
        frames = [
            ((0.5, 0, 0.1), (0, 0, -0.1 + 0.000016 * (i % 2), 0))
            for i in range(2500)
        ]
        robot = FakeRobot(frames)
        status, _, _ = generator.wait_until_settled(
            robot, 8, robot.sensors, robot.gps, (0, 0, -0.1, 0)
        )
        self.assertEqual(status, "timeout")

    def test_slide_tolerance_allows_small_stationary_error(self):
        robot = FakeRobot([((0.5, 0, 0.1), (0, 0, -0.0999, 0))])
        status, _, _ = generator.wait_until_settled(
            robot, 8, robot.sensors, robot.gps, (0, 0, -0.1, 0)
        )
        self.assertEqual(status, "settled")

    def test_correct_devices_speeds_units_and_fixed_shaft_rotation(self):
        robot = FakeRobot([((0.5, 0.1, 0.1), (0.1, -0.2, -0.1, 0))], max_steps=70)
        output = io.StringIO()
        with patch.object(generator, "loop_value", 1), patch.object(
            generator.random, "uniform", side_effect=(0.1, -0.2, -0.1)
        ), contextlib.redirect_stdout(output):
            generator.run(robot)
        self.assertEqual([m.targets for m in robot.motors], [[0.1], [-0.2], [-0.1], [0]])
        self.assertEqual([m.velocity for m in robot.motors], [0.5, 0.5, 0.05, 0.5])
        self.assertIn("Targets (rad, rad, m): (0.100000, -0.200000, -0.100000)", output.getvalue())
        self.assertIn("Measured joints (rad, rad, m):", output.getvalue())

    def test_missing_slide_sensor_fails_before_moving(self):
        robot = FakeRobot()
        robot.sensors[2] = None
        with self.assertRaisesRegex(RuntimeError, "Missing motor or sensor for shaft_linear_motor"):
            generator.run(robot)
        self.assertTrue(all(not motor.targets for motor in robot.motors))

    def test_missing_gps_fails_before_moving(self):
        robot = FakeRobot()
        robot.gps = None
        with self.assertRaisesRegex(RuntimeError, "Missing GPS"):
            generator.run(robot)
        self.assertTrue(all(not motor.targets for motor in robot.motors))


if __name__ == "__main__":
    unittest.main()
