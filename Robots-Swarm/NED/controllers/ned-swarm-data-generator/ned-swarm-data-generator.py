import math
import random

# set the amount of datapoints you want to record
loop_value = 1000

# Joint targets in radians. The wrist joints stay at zero for this 3-input dataset.
# Limit forward shoulder/elbow rotation so the gripper clears the ground and base.
# These sampling limits are narrower than the motors' mechanical travel limits.
ROTATION_LIMITS = ((-2.7, 2.7), (-0.7, 0.4), (-1.4, 0.8))
JOINT_NAMES = tuple(f"joint_{i}" for i in range(1, 7))
JOINT_TOLERANCE = 0.005  # rad
JOINT_SPEED_TOLERANCE = 0.01  # rad/s
HAND_SPEED_TOLERANCE = 0.001  # m/s
SETTLE_TIME_MS = 500
MOVE_TIMEOUT_MS = 20000  # A full joint_1 sweep at 0.5 rad/s takes about 11 s.
MAX_CONSECUTIVE_FAILURES = 5


def position_in_base_frame(world_position, base_position, base_orientation):
    """Convert metres in world axes to the robot base's origin and axes."""
    if not all(
        math.isfinite(value)
        for values in (world_position, base_position, base_orientation)
        for value in values
    ):
        raise ValueError("Cannot convert an invalid hand position or robot base pose.")
    offset = tuple(hand - base for hand, base in zip(world_position, base_position))
    # Webots returns a row-major local-to-world rotation R. Its inverse is R^T.
    return tuple(
        sum(base_orientation[3 * row + column] * offset[row] for row in range(3))
        for column in range(3)
    )


def wait_until_settled(robot, timestep, sensors, gps, targets):
    """Return a status and measured pose; never wait indefinitely for a target."""
    previous_position = None
    previous_joints = None
    still_time = 0
    elapsed = 0
    seconds_per_step = timestep / 1000.0

    while elapsed < MOVE_TIMEOUT_MS:
        if robot.step(timestep) == -1:
            return "stopped", None, None
        elapsed += timestep
        position = tuple(gps.getValues())
        joints = tuple(sensor.getValue() for sensor in sensors)
        if not all(math.isfinite(value) for value in position + joints):
            return "invalid sensor readings", position, joints

        at_target = all(
            abs(actual - target) <= JOINT_TOLERANCE
            for actual, target in zip(joints, targets)
        )
        # Use speeds so that changing the physics timestep doesn't change the test.
        stationary = previous_position is not None and (
            math.dist(position, previous_position) / seconds_per_step
            <= HAND_SPEED_TOLERANCE
            and all(
                abs(current - previous) / seconds_per_step <= JOINT_SPEED_TOLERANCE
                for current, previous in zip(joints, previous_joints)
            )
        )
        if at_target and stationary:
            still_time += timestep
        else:
            still_time = 0
        previous_position = position
        previous_joints = joints

        if still_time >= SETTLE_TIME_MS:
            return "settled", position, joints
    return "timeout", position, joints


def run(robot):
    timestep = int(robot.getBasicTimeStep())
    name = robot.getName()
    if not robot.getSupervisor():
        raise RuntimeError(
            f"[{name}] Base-relative output requires supervisor TRUE. "
            "Reload the updated swarm world."
        )
    base_node = robot.getSelf()
    motors = []
    sensors = []
    for joint_name in JOINT_NAMES:
        motor = robot.getDevice(joint_name)
        sensor = robot.getDevice(f"{joint_name}_sensor")
        if motor is None or sensor is None:
            raise RuntimeError(f"[{name}] Missing motor or sensor for {joint_name}.")
        motor.setVelocity(0.5)
        sensor.enable(timestep)
        motors.append(motor)
        sensors.append(sensor)

    gps = robot.getDevice("gps")
    if gps is None:
        raise RuntimeError(f"[{name}] Missing GPS. Reload the NedWithGPS world.")
    gps.enable(timestep)

    samples = 0
    failures = 0
    rejected = 0
    # Allow replacement attempts for rejected poses, while bounding a faulty run.
    for attempt in range(1, loop_value * 3 + 1):
        targets = tuple(random.uniform(low, high) for low, high in ROTATION_LIMITS)
        targets += (0.0, 0.0, 0.0)
        for motor, target in zip(motors, targets):
            motor.setPosition(target)

        status, position, joints = wait_until_settled(
            robot, timestep, sensors, gps, targets
        )
        if status == "stopped":
            return
        if status != "settled":
            rejected += 1
            failures += 1
            errors = tuple(actual - target for actual, target in zip(joints, targets))
            print(
                f"[{name}] Rejected attempt {attempt}: {status}; "
                f"targets={targets}, joint_errors={errors}, world_GPS={position}",
                flush=True,
            )
            if status == "invalid sensor readings" or failures >= MAX_CONSECUTIVE_FAILURES:
                raise RuntimeError(
                    f"[{name}] Simulation fault after {samples}/{loop_value} samples. "
                    "Reload the world before collecting more data."
                )
            continue

        # Read the actual base pose so moving/rotating a robot needs no manual offsets.
        # In the ground-level swarm, each base's origin lies on the floor (world Z=0).
        x, y, z = position_in_base_frame(
            position, base_node.getPosition(), base_node.getOrientation()
        )
        failures = 0
        samples += 1
        target_text = ", ".join(f"{value:.6f}" for value in targets[:3])
        measured_text = ", ".join(f"{value:.6f}" for value in joints[:3])
        print(
            f"[{name}] Iteration {samples}/{loop_value} - Targets: ({target_text}) | "
            f"Measured joints: ({measured_text}) | "
            f"Ned Final Position (base frame, m) -> "
            f"X: {x:.6f}, Y: {y:.6f}, Z: {z:.6f}",
            flush=True,
        )
        if samples == loop_value:
            break

    if samples < loop_value:
        raise RuntimeError(
            f"[{name}] Attempt limit reached: {samples}/{loop_value} samples, "
            f"{rejected} rejected movements."
        )
    print(
        f"[{name}] Completed {samples} samples; rejected {rejected} movements. "
        "Holding the final pose.",
        flush=True,
    )
    # Keep the controller connected while the other arms finish their samples.
    while robot.step(timestep) != -1:
        pass


if __name__ == "__main__":
    from controller import Supervisor

    run(Supervisor())
