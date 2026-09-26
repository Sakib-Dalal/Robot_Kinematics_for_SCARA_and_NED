"""Move each SCARA arm to random poses and print its joint positions and hand position.

Start with run() below to follow the main steps. The two helper functions handle
waiting for the arm to stop and converting the hand position to the robot's axes.
"""

import math
import random

# Number of accepted data points to print for EACH robot.
loop_value = 1000

# The first two joints rotate. The third joint slides vertically.
# Negative slide values lower the hand. Stop at -0.15 m to clear the floor.
JOINT_LIMITS = (
    (-0.73, 0.73),  # Joint 1: base arm rotation, in radians.
    (-0.83, 0.83),  # Joint 2: elbow rotation, in radians.
    (-0.15, 0.0),  # Joint 3: vertical slide, in METRES (not millimetres).
)

# The fourth motor rotates the shaft. We hold it at zero for this 3-input dataset.
# All these lists use the same order: base arm, elbow, vertical slide, shaft rotation.
JOINT_NAMES = (
    "base_arm_motor",
    "arm_motor",
    "shaft_linear_motor",
    "shaft_rotation_motor",
)
SENSOR_NAMES = (
    "base_arm_position",
    "arm_position",
    "shaft_linear_position",
    "shaft_rotation_position",
)
MOTOR_SPEEDS = (0.5, 0.5, 0.05, 0.5)  # rad/s, rad/s, m/s, rad/s.

# The slide needs its own tolerances because its readings are in metres.
JOINT_TOLERANCES = (0.005, 0.005, 0.0005, 0.005)  # rad, rad, m, rad.
JOINT_SPEED_TOLERANCES = (0.01, 0.01, 0.001, 0.01)  # rad/s, rad/s, m/s, rad/s.
HAND_SPEED_TOLERANCE = 0.001  # Maximum hand speed counted as still, in metres/s.
SETTLE_TIME_MS = 500  # Stay still for half a second before recording.
MOVE_TIMEOUT_MS = 20000  # Give each movement up to 20 seconds.
MAX_CONSECUTIVE_FAILURES = 5


def position_in_base_frame(world_position, base_position, base_orientation):
    """Return the hand's (x, y, z) in metres, measured from this robot's base."""
    for readings in (world_position, base_position, base_orientation):
        for value in readings:
            # Reject NaN (not a number) and infinity from a faulty simulation.
            if not math.isfinite(value):
                raise ValueError("Cannot convert an invalid hand position or robot base pose.")

    # First, remove the robot's location in the swarm grid.
    dx = world_position[0] - base_position[0]
    dy = world_position[1] - base_position[1]
    dz = world_position[2] - base_position[2]

    # Next, express that offset along the robot's own X, Y, and Z axes.
    # Webots stores the rotation matrix as nine numbers, one row after another:
    #   r[0]  r[1]  r[2]
    #   r[3]  r[4]  r[5]
    #   r[6]  r[7]  r[8]
    # Each column describes one robot axis in the world. For an unrotated base,
    # this calculation simply gives x = dx, y = dy, and z = dz.
    r = base_orientation
    x = r[0] * dx + r[3] * dy + r[6] * dz
    y = r[1] * dx + r[4] * dy + r[7] * dz
    z = r[2] * dx + r[5] * dy + r[8] * dz
    return x, y, z


def wait_until_settled(robot, timestep, joint_sensors, gps, target_positions):
    """Wait for the target pose; return (status, hand position, measured joint positions)."""
    previous_hand_position = None
    previous_joint_positions = None
    still_time_ms = 0
    elapsed_ms = 0
    seconds_per_step = timestep / 1000.0

    while elapsed_ms < MOVE_TIMEOUT_MS:
        # step() advances the simulation. -1 means Webots has stopped it.
        if robot.step(timestep) == -1:
            return "stopped", None, None
        elapsed_ms += timestep

        # Read the hand GPS and all four joint sensors.
        hand_position = tuple(gps.getValues())
        joint_positions = []
        for sensor in joint_sensors:
            joint_positions.append(sensor.getValue())
        joint_positions = tuple(joint_positions)

        for value in hand_position + joint_positions:
            if not math.isfinite(value):
                return "invalid sensor readings", hand_position, joint_positions

        # Check 1: every joint must be close to its requested position.
        at_target = True
        for index in range(len(joint_positions)):
            position_error = abs(joint_positions[index] - target_positions[index])
            if position_error > JOINT_TOLERANCES[index]:
                at_target = False

        # Check 2: the hand AND joints must have almost stopped moving.
        # Speed = change in position / time between readings.
        is_still = False
        if previous_hand_position is not None:
            hand_distance = math.dist(hand_position, previous_hand_position)
            hand_speed = hand_distance / seconds_per_step
            is_still = hand_speed <= HAND_SPEED_TOLERANCE

            for index in range(len(joint_positions)):
                position_change = abs(joint_positions[index] - previous_joint_positions[index])
                joint_speed = position_change / seconds_per_step
                if joint_speed > JOINT_SPEED_TOLERANCES[index]:
                    is_still = False

        # Both checks must pass continuously for half a second.
        if at_target and is_still:
            still_time_ms += timestep
        else:
            still_time_ms = 0
        previous_hand_position = hand_position
        previous_joint_positions = joint_positions

        if still_time_ms >= SETTLE_TIME_MS:
            return "settled", hand_position, joint_positions

    return "timeout", hand_position, joint_positions


def run(robot):
    """Set up this robot, collect data, then hold the last pose."""
    # 1. Connect to the robot's base, motors, and sensors.
    timestep = int(robot.getBasicTimeStep())
    name = robot.getName()
    # Supervisor gives us the base position/rotation for the coordinate conversion.
    if not robot.getSupervisor():
        raise RuntimeError(
            f"[{name}] Base-relative output requires supervisor TRUE. "
            "Reload the updated swarm world."
        )
    base_node = robot.getSelf()
    motors = []
    joint_sensors = []
    for index in range(len(JOINT_NAMES)):
        joint_name = JOINT_NAMES[index]
        motor = robot.getDevice(joint_name)
        sensor = robot.getDevice(SENSOR_NAMES[index])
        if motor is None or sensor is None:
            raise RuntimeError(f"[{name}] Missing motor or sensor for {joint_name}.")
        motor.setVelocity(MOTOR_SPEEDS[index])
        sensor.enable(timestep)
        motors.append(motor)
        joint_sensors.append(sensor)

    gps = robot.getDevice("gps")
    if gps is None:
        raise RuntimeError(f"[{name}] Missing GPS. Reload the ScaraWithGPS world.")
    gps.enable(timestep)

    sample_count = 0
    consecutive_failures = 0
    rejected_count = 0
    # Allow extra attempts when a pose fails, but never retry forever.
    max_attempts = loop_value * 3
    for attempt in range(1, max_attempts + 1):
        # 2. Choose positions for the two arm joints and slide. Hold shaft rotation at zero.
        target_positions = []
        for minimum, maximum in JOINT_LIMITS:
            target_positions.append(random.uniform(minimum, maximum))
        target_positions.append(0.0)

        for index in range(len(motors)):
            motors[index].setPosition(target_positions[index])

        # 3. Wait until the arm reaches those positions and stops moving.
        status, hand_position, joint_positions = wait_until_settled(
            robot, timestep, joint_sensors, gps, target_positions
        )
        if status == "stopped":
            return
        if status != "settled":
            # Skip this pose so an unfinished movement never becomes a data point.
            rejected_count += 1
            consecutive_failures += 1
            position_errors = []
            for index in range(len(joint_positions)):
                position_errors.append(joint_positions[index] - target_positions[index])
            print(
                f"[{name}] Rejected attempt {attempt}: {status}; "
                f"targets={tuple(target_positions)}, joint_errors={tuple(position_errors)}, "
                f"world_GPS={hand_position}",
                flush=True,
            )
            if (
                status == "invalid sensor readings"
                or consecutive_failures >= MAX_CONSECUTIVE_FAILURES
            ):
                raise RuntimeError(
                    f"[{name}] Simulation fault after {sample_count}/{loop_value} samples. "
                    "Reload the world before collecting more data."
                )
            continue

        # 4. Measure the hand from this robot's base, then print the data point.
        base_position = base_node.getPosition()
        base_orientation = base_node.getOrientation()
        x, y, z = position_in_base_frame(
            hand_position, base_position, base_orientation
        )
        consecutive_failures = 0
        sample_count += 1
        # .6f displays each number with six digits after the decimal point.
        print(
            f"[{name}] Iteration {sample_count}/{loop_value} - "
            f"Targets (rad, rad, m): "
            f"({target_positions[0]:.6f}, {target_positions[1]:.6f}, {target_positions[2]:.6f}) | "
            f"Measured joints (rad, rad, m): "
            f"({joint_positions[0]:.6f}, {joint_positions[1]:.6f}, {joint_positions[2]:.6f}) | "
            f"Scara Final Position (base frame, m) -> "
            f"X: {x:.6f}, Y: {y:.6f}, Z: {z:.6f}",
            flush=True,
        )
        if sample_count == loop_value:
            break

    if sample_count < loop_value:
        raise RuntimeError(
            f"[{name}] Attempt limit reached: {sample_count}/{loop_value} samples, "
            f"{rejected_count} rejected movements."
        )
    print(
        f"[{name}] Completed {sample_count} samples; rejected {rejected_count} movements. "
        "Holding the final pose.",
        flush=True,
    )
    # 5. Keep this robot holding its last pose while the other robots finish.
    while robot.step(timestep) != -1:
        pass


if __name__ == "__main__":
    # Webots starts here. Supervisor includes normal Robot controls plus base access.
    from controller import Supervisor

    robot = Supervisor()
    run(robot)
