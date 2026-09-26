"""Move each NED arm to random poses and save joint angles + hand position."""

import math
import random
import csv
import os
import fcntl


# -------------------------------------------------
# Configuration
# -------------------------------------------------

# Number of accepted samples PER ROBOT
loop_value = 1000

# Random joint limits
ROTATION_LIMITS = (
    (-2.7, 2.7),   # Joint 1
    (-0.7, 0.4),   # Joint 2
    (-1.4, 0.8),   # Joint 3
)

JOINT_NAMES = (
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6",
)

JOINT_TOLERANCE = 0.005
JOINT_SPEED_TOLERANCE = 0.01
HAND_SPEED_TOLERANCE = 0.001

SETTLE_TIME_MS = 500
MOVE_TIMEOUT_MS = 20000

MAX_CONSECUTIVE_FAILURES = 5


# -------------------------------------------------
# Robot type
# -------------------------------------------------

IS_SCARA = False
IS_NED = True


# -------------------------------------------------
# Shared CSV
# -------------------------------------------------

CSV_FILENAME = "ned_dataset.csv"

FIELDNAMES = [
    "is_scara",
    "is_ned",
    "q1",
    "q2",
    "q3",
    "x",
    "y",
    "z",
]


# -------------------------------------------------
# Safe CSV writer for multiple robots
# -------------------------------------------------

def save_sample_to_csv(q1, q2, q3, x, y, z):
    """
    Safely append one row to the shared CSV file.

    fcntl locking ensures that only one Webots controller
    writes to the file at a time.
    """

    with open(
        CSV_FILENAME,
        "a+",
        newline=""
    ) as csv_file:

        # Lock file
        fcntl.flock(
            csv_file.fileno(),
            fcntl.LOCK_EX
        )

        try:
            # Move to end
            csv_file.seek(
                0,
                os.SEEK_END
            )

            # Check whether file is empty
            file_is_empty = (
                csv_file.tell() == 0
            )

            writer = csv.DictWriter(
                csv_file,
                fieldnames=FIELDNAMES
            )

            # Only the first robot writes the header
            if file_is_empty:
                writer.writeheader()

            writer.writerow({
                "is_scara": int(IS_SCARA),
                "is_ned": int(IS_NED),
                "q1": q1,
                "q2": q2,
                "q3": q3,
                "x": x,
                "y": y,
                "z": z,
            })

            # Make sure row reaches disk
            csv_file.flush()
            os.fsync(csv_file.fileno())

        finally:
            # Unlock file
            fcntl.flock(
                csv_file.fileno(),
                fcntl.LOCK_UN
            )


# -------------------------------------------------
# Convert GPS position to robot base frame
# -------------------------------------------------

def position_in_base_frame(
    world_position,
    base_position,
    base_orientation
):
    """Convert GPS world coordinates to robot-base coordinates."""

    for readings in (
        world_position,
        base_position,
        base_orientation
    ):
        for value in readings:
            if not math.isfinite(value):
                raise ValueError(
                    "Invalid robot/base position."
                )

    # Position relative to base
    dx = world_position[0] - base_position[0]
    dy = world_position[1] - base_position[1]
    dz = world_position[2] - base_position[2]

    # Rotation matrix
    r = base_orientation

    # Convert world position into local robot axes
    x = (
        r[0] * dx
        + r[3] * dy
        + r[6] * dz
    )

    y = (
        r[1] * dx
        + r[4] * dy
        + r[7] * dz
    )

    z = (
        r[2] * dx
        + r[5] * dy
        + r[8] * dz
    )

    return x, y, z


# -------------------------------------------------
# Wait until robot settles
# -------------------------------------------------

def wait_until_settled(
    robot,
    timestep,
    joint_sensors,
    gps,
    target_angles
):
    """Wait until robot reaches target pose and stops moving."""

    previous_hand_position = None
    previous_joint_angles = None

    still_time_ms = 0
    elapsed_ms = 0

    seconds_per_step = (
        timestep / 1000.0
    )

    hand_position = None
    joint_angles = None

    while elapsed_ms < MOVE_TIMEOUT_MS:

        # Advance simulation
        if robot.step(timestep) == -1:
            return "stopped", None, None

        elapsed_ms += timestep

        # Read GPS
        hand_position = tuple(
            gps.getValues()
        )

        # Read joint sensors
        joint_angles = tuple(
            sensor.getValue()
            for sensor in joint_sensors
        )

        # Validate values
        for value in (
            hand_position
            + joint_angles
        ):
            if not math.isfinite(value):

                return (
                    "invalid sensor readings",
                    hand_position,
                    joint_angles
                )

        # -------------------------------------------------
        # Check whether joints reached target
        # -------------------------------------------------

        at_target = True

        for index in range(
            len(joint_angles)
        ):

            angle_error = abs(
                joint_angles[index]
                - target_angles[index]
            )

            if (
                angle_error
                > JOINT_TOLERANCE
            ):
                at_target = False
                break

        # -------------------------------------------------
        # Check whether robot is still
        # -------------------------------------------------

        is_still = False

        if (
            previous_hand_position is not None
            and
            previous_joint_angles is not None
        ):

            hand_distance = math.dist(
                hand_position,
                previous_hand_position
            )

            hand_speed = (
                hand_distance
                / seconds_per_step
            )

            is_still = (
                hand_speed
                <= HAND_SPEED_TOLERANCE
            )

            for index in range(
                len(joint_angles)
            ):

                angle_change = abs(
                    joint_angles[index]
                    - previous_joint_angles[index]
                )

                joint_speed = (
                    angle_change
                    / seconds_per_step
                )

                if (
                    joint_speed
                    > JOINT_SPEED_TOLERANCE
                ):
                    is_still = False
                    break

        # -------------------------------------------------
        # Must remain still continuously
        # -------------------------------------------------

        if (
            at_target
            and
            is_still
        ):
            still_time_ms += timestep
        else:
            still_time_ms = 0

        previous_hand_position = (
            hand_position
        )

        previous_joint_angles = (
            joint_angles
        )

        if (
            still_time_ms
            >= SETTLE_TIME_MS
        ):

            return (
                "settled",
                hand_position,
                joint_angles
            )

    return (
        "timeout",
        hand_position,
        joint_angles
    )


# -------------------------------------------------
# Main robot routine
# -------------------------------------------------

def run(robot):

    timestep = int(
        robot.getBasicTimeStep()
    )

    name = robot.getName()

    print(
        f"[{name}] Starting NED dataset collection...",
        flush=True
    )

    # -------------------------------------------------
    # Supervisor check
    # -------------------------------------------------

    if not robot.getSupervisor():

        raise RuntimeError(
            f"[{name}] "
            "supervisor TRUE is required."
        )

    base_node = (
        robot.getSelf()
    )

    motors = []
    joint_sensors = []

    # -------------------------------------------------
    # 1. Get motors and joint sensors
    # -------------------------------------------------

    for joint_name in JOINT_NAMES:

        motor = robot.getDevice(
            joint_name
        )

        sensor = robot.getDevice(
            f"{joint_name}_sensor"
        )

        if motor is None:

            raise RuntimeError(
                f"[{name}] "
                f"Motor '{joint_name}' "
                "not found."
            )

        if sensor is None:

            raise RuntimeError(
                f"[{name}] "
                f"Sensor "
                f"'{joint_name}_sensor' "
                "not found."
            )

        motor.setVelocity(
            0.5
        )

        sensor.enable(
            timestep
        )

        motors.append(
            motor
        )

        joint_sensors.append(
            sensor
        )

    # -------------------------------------------------
    # 2. Get hand GPS
    # -------------------------------------------------

    gps = robot.getDevice(
        "gps"
    )

    if gps is None:

        raise RuntimeError(
            f"[{name}] "
            "GPS named 'gps' not found."
        )

    gps.enable(
        timestep
    )

    # Allow sensors to initialize
    if robot.step(timestep) == -1:
        return

    # -------------------------------------------------
    # Counters
    # -------------------------------------------------

    sample_count = 0
    rejected_count = 0
    consecutive_failures = 0

    max_attempts = (
        loop_value * 3
    )

    # -------------------------------------------------
    # 3. Start data collection
    # -------------------------------------------------

    for attempt in range(
        1,
        max_attempts + 1
    ):

        # ---------------------------------------------
        # Random q1, q2, q3
        # ---------------------------------------------

        q1_target = random.uniform(
            ROTATION_LIMITS[0][0],
            ROTATION_LIMITS[0][1]
        )

        q2_target = random.uniform(
            ROTATION_LIMITS[1][0],
            ROTATION_LIMITS[1][1]
        )

        q3_target = random.uniform(
            ROTATION_LIMITS[2][0],
            ROTATION_LIMITS[2][1]
        )

        # q4, q5, q6 remain zero
        target_angles = (
            q1_target,
            q2_target,
            q3_target,
            0.0,
            0.0,
            0.0,
        )

        # ---------------------------------------------
        # Move motors
        # ---------------------------------------------

        for index in range(
            len(motors)
        ):

            motors[index].setPosition(
                target_angles[index]
            )

        # ---------------------------------------------
        # Wait for robot to settle
        # ---------------------------------------------

        (
            status,
            hand_position,
            joint_angles
        ) = wait_until_settled(
            robot,
            timestep,
            joint_sensors,
            gps,
            target_angles
        )

        # Simulation stopped
        if status == "stopped":
            return

        # ---------------------------------------------
        # Reject bad pose
        # ---------------------------------------------

        if status != "settled":

            rejected_count += 1
            consecutive_failures += 1

            print(
                f"[{name}] "
                f"Rejected attempt "
                f"{attempt}: {status}",
                flush=True
            )

            if (
                status
                == "invalid sensor readings"
                or
                consecutive_failures
                >= MAX_CONSECUTIVE_FAILURES
            ):

                raise RuntimeError(
                    f"[{name}] "
                    "Too many failed movements."
                )

            continue

        # Successful movement
        consecutive_failures = 0

        # -------------------------------------------------
        # 4. Get robot base pose
        # -------------------------------------------------

        base_position = (
            base_node.getPosition()
        )

        base_orientation = (
            base_node.getOrientation()
        )

        # -------------------------------------------------
        # 5. Convert GPS world position to base coordinates
        # -------------------------------------------------

        x, y, z = (
            position_in_base_frame(
                hand_position,
                base_position,
                base_orientation
            )
        )

        # -------------------------------------------------
        # 6. Use ACTUAL measured angles
        # -------------------------------------------------

        measured_q1 = (
            joint_angles[0]
        )

        measured_q2 = (
            joint_angles[1]
        )

        measured_q3 = (
            joint_angles[2]
        )

        # -------------------------------------------------
        # 7. Save sample
        # -------------------------------------------------

        save_sample_to_csv(
            measured_q1,
            measured_q2,
            measured_q3,
            x,
            y,
            z
        )

        sample_count += 1

        # -------------------------------------------------
        # Print sample
        # -------------------------------------------------

        print(
            f"[{name}] "
            f"Sample "
            f"{sample_count}/{loop_value} | "
            f"q1={measured_q1:.6f}, "
            f"q2={measured_q2:.6f}, "
            f"q3={measured_q3:.6f} | "
            f"x={x:.6f}, "
            f"y={y:.6f}, "
            f"z={z:.6f}",
            flush=True
        )

        # -------------------------------------------------
        # Stop once this robot collected enough data
        # -------------------------------------------------

        if (
            sample_count
            >= loop_value
        ):
            break

    # -------------------------------------------------
    # 8. Verify completion
    # -------------------------------------------------

    if (
        sample_count
        < loop_value
    ):

        raise RuntimeError(
            f"[{name}] "
            f"Only collected "
            f"{sample_count}/{loop_value} "
            "samples."
        )

    # -------------------------------------------------
    # Finished
    # -------------------------------------------------

    print(
        "\n"
        "====================================",
        flush=True
    )

    print(
        f"[{name}] Dataset collection complete",
        flush=True
    )

    print(
        f"Accepted samples : {sample_count}",
        flush=True
    )

    print(
        f"Rejected samples : {rejected_count}",
        flush=True
    )

    print(
        f"Shared CSV       : {CSV_FILENAME}",
        flush=True
    )

    print(
        "====================================\n",
        flush=True
    )

    # Keep final pose
    while robot.step(
        timestep
    ) != -1:
        pass


# -------------------------------------------------
# Webots entry point
# -------------------------------------------------

if __name__ == "__main__":

    from controller import Supervisor

    robot = Supervisor()

    run(robot)