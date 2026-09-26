"""Move each SCARA arm to random poses and save joint positions + hand position."""

import math
import random
import csv
import os
import fcntl


# ============================================================
# CONFIGURATION
# ============================================================

# Number of accepted samples PER ROBOT
loop_value = 1000


# ------------------------------------------------------------
# SCARA joint limits
#
# q1 = rotational joint in radians
# q2 = rotational joint in radians
# q3 = linear/prismatic joint in metres
# ------------------------------------------------------------

JOINT_LIMITS = (
    (-0.73, 0.73),   # Joint 1 - radians
    (-0.83, 0.83),   # Joint 2 - radians
    (-0.15, 0.0),    # Joint 3 - metres
)


# ------------------------------------------------------------
# Motor names
# ------------------------------------------------------------

JOINT_NAMES = (
    "base_arm_motor",
    "arm_motor",
    "shaft_linear_motor",
    "shaft_rotation_motor",
)


# ------------------------------------------------------------
# Position sensor names
# ------------------------------------------------------------

SENSOR_NAMES = (
    "base_arm_position",
    "arm_position",
    "shaft_linear_position",
    "shaft_rotation_position",
)


# ------------------------------------------------------------
# Motor speeds
#
# Joint 1 = rad/s
# Joint 2 = rad/s
# Joint 3 = m/s
# Joint 4 = rad/s
# ------------------------------------------------------------

MOTOR_SPEEDS = (
    0.5,
    0.5,
    0.05,
    0.5,
)


# ------------------------------------------------------------
# Joint tolerances
# ------------------------------------------------------------

JOINT_TOLERANCES = (
    0.005,      # Joint 1 - radians
    0.005,      # Joint 2 - radians
    0.0005,     # Joint 3 - metres
    0.005,      # Joint 4 - radians
)


JOINT_SPEED_TOLERANCES = (
    0.01,       # Joint 1 - rad/s
    0.01,       # Joint 2 - rad/s
    0.001,      # Joint 3 - m/s
    0.01,       # Joint 4 - rad/s
)


HAND_SPEED_TOLERANCE = 0.001

SETTLE_TIME_MS = 500

MOVE_TIMEOUT_MS = 20000

MAX_CONSECUTIVE_FAILURES = 5


# ============================================================
# ROBOT TYPE
# ============================================================

IS_SCARA = True
IS_NED = False


# ============================================================
# CSV CONFIGURATION
# ============================================================

CSV_FILENAME = "scara_dataset.csv"


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


# ============================================================
# SAFE CSV WRITER
# ============================================================

def save_sample_to_csv(q1, q2, q3, x, y, z):
    """
    Safely append one SCARA sample to the shared CSV.

    File locking prevents multiple SCARA robots from
    writing to the same CSV at the same time.
    """

    with open(
        CSV_FILENAME,
        "a+",
        newline=""
    ) as csv_file:

        # ----------------------------------------------------
        # Lock file so only one robot can write
        # ----------------------------------------------------

        fcntl.flock(
            csv_file.fileno(),
            fcntl.LOCK_EX
        )

        try:

            # ------------------------------------------------
            # Move pointer to end of file
            # ------------------------------------------------

            csv_file.seek(
                0,
                os.SEEK_END
            )


            # ------------------------------------------------
            # Check whether CSV is empty
            # ------------------------------------------------

            file_is_empty = (
                csv_file.tell() == 0
            )


            # ------------------------------------------------
            # Create CSV writer
            # ------------------------------------------------

            writer = csv.DictWriter(
                csv_file,
                fieldnames=FIELDNAMES
            )


            # ------------------------------------------------
            # Only first robot writes header
            # ------------------------------------------------

            if file_is_empty:
                writer.writeheader()


            # ------------------------------------------------
            # Save one complete row
            # ------------------------------------------------

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


            # ------------------------------------------------
            # Force data to disk
            # ------------------------------------------------

            csv_file.flush()

            os.fsync(
                csv_file.fileno()
            )


        finally:

            # ------------------------------------------------
            # Unlock file for next robot
            # ------------------------------------------------

            fcntl.flock(
                csv_file.fileno(),
                fcntl.LOCK_UN
            )


# ============================================================
# WORLD POSITION -> ROBOT BASE POSITION
# ============================================================

def position_in_base_frame(
    world_position,
    base_position,
    base_orientation
):
    """
    Convert the GPS world position into coordinates
    relative to the SCARA robot base.
    """

    # --------------------------------------------------------
    # Validate numbers
    # --------------------------------------------------------

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


    # --------------------------------------------------------
    # Remove robot's world location
    # --------------------------------------------------------

    dx = (
        world_position[0]
        - base_position[0]
    )

    dy = (
        world_position[1]
        - base_position[1]
    )

    dz = (
        world_position[2]
        - base_position[2]
    )


    # --------------------------------------------------------
    # Robot orientation matrix
    # --------------------------------------------------------

    r = base_orientation


    # --------------------------------------------------------
    # Convert world coordinates -> local robot coordinates
    # --------------------------------------------------------

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


# ============================================================
# WAIT UNTIL SCARA STOPS
# ============================================================

def wait_until_settled(
    robot,
    timestep,
    joint_sensors,
    gps,
    target_positions
):
    """
    Wait until all SCARA joints reach their target positions
    and stop moving.
    """

    previous_hand_position = None

    previous_joint_positions = None

    still_time_ms = 0

    elapsed_ms = 0

    seconds_per_step = (
        timestep / 1000.0
    )


    hand_position = None

    joint_positions = None


    # --------------------------------------------------------
    # Wait until timeout
    # --------------------------------------------------------

    while elapsed_ms < MOVE_TIMEOUT_MS:


        # ----------------------------------------------------
        # Advance Webots simulation
        # ----------------------------------------------------

        if robot.step(timestep) == -1:

            return (
                "stopped",
                None,
                None
            )


        elapsed_ms += timestep


        # ----------------------------------------------------
        # Read GPS
        # ----------------------------------------------------

        hand_position = tuple(
            gps.getValues()
        )


        # ----------------------------------------------------
        # Read all four joint sensors
        # ----------------------------------------------------

        joint_positions = tuple(
            sensor.getValue()
            for sensor in joint_sensors
        )


        # ----------------------------------------------------
        # Validate readings
        # ----------------------------------------------------

        for value in (
            hand_position
            + joint_positions
        ):

            if not math.isfinite(value):

                return (
                    "invalid sensor readings",
                    hand_position,
                    joint_positions
                )


        # ====================================================
        # CHECK 1
        # Have all joints reached target?
        # ====================================================

        at_target = True


        for index in range(
            len(joint_positions)
        ):

            position_error = abs(
                joint_positions[index]
                - target_positions[index]
            )


            if (
                position_error
                > JOINT_TOLERANCES[index]
            ):

                at_target = False

                break


        # ====================================================
        # CHECK 2
        # Have joints and hand stopped?
        # ====================================================

        is_still = False


        if (
            previous_hand_position is not None
            and
            previous_joint_positions is not None
        ):


            # ------------------------------------------------
            # Hand speed
            # ------------------------------------------------

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


            # ------------------------------------------------
            # Joint speeds
            # ------------------------------------------------

            for index in range(
                len(joint_positions)
            ):

                position_change = abs(
                    joint_positions[index]
                    - previous_joint_positions[index]
                )


                joint_speed = (
                    position_change
                    / seconds_per_step
                )


                if (
                    joint_speed
                    > JOINT_SPEED_TOLERANCES[index]
                ):

                    is_still = False

                    break


        # ====================================================
        # Must remain settled for SETTLE_TIME_MS
        # ====================================================

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


        previous_joint_positions = (
            joint_positions
        )


        # ----------------------------------------------------
        # Successful position
        # ----------------------------------------------------

        if (
            still_time_ms
            >= SETTLE_TIME_MS
        ):

            return (
                "settled",
                hand_position,
                joint_positions
            )


    # --------------------------------------------------------
    # Movement took too long
    # --------------------------------------------------------

    return (
        "timeout",
        hand_position,
        joint_positions
    )


# ============================================================
# MAIN SCARA CONTROLLER
# ============================================================

def run(robot):


    # --------------------------------------------------------
    # Get timestep and robot name
    # --------------------------------------------------------

    timestep = int(
        robot.getBasicTimeStep()
    )


    name = (
        robot.getName()
    )


    print(
        f"[{name}] "
        "Starting SCARA dataset collection...",
        flush=True
    )


    # ========================================================
    # Supervisor required
    # ========================================================

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


    # ========================================================
    # 1. GET MOTORS + SENSORS
    # ========================================================

    for index in range(
        len(JOINT_NAMES)
    ):


        joint_name = (
            JOINT_NAMES[index]
        )


        sensor_name = (
            SENSOR_NAMES[index]
        )


        # ----------------------------------------------------
        # Motor
        # ----------------------------------------------------

        motor = robot.getDevice(
            joint_name
        )


        # ----------------------------------------------------
        # Sensor
        # ----------------------------------------------------

        sensor = robot.getDevice(
            sensor_name
        )


        # ----------------------------------------------------
        # Check motor
        # ----------------------------------------------------

        if motor is None:

            raise RuntimeError(
                f"[{name}] "
                f"Motor '{joint_name}' "
                "not found."
            )


        # ----------------------------------------------------
        # Check sensor
        # ----------------------------------------------------

        if sensor is None:

            raise RuntimeError(
                f"[{name}] "
                f"Sensor '{sensor_name}' "
                "not found."
            )


        # ----------------------------------------------------
        # Set motor speed
        # ----------------------------------------------------

        motor.setVelocity(
            MOTOR_SPEEDS[index]
        )


        # ----------------------------------------------------
        # Enable sensor
        # ----------------------------------------------------

        sensor.enable(
            timestep
        )


        motors.append(
            motor
        )


        joint_sensors.append(
            sensor
        )


    # ========================================================
    # 2. GPS
    # ========================================================

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


    # --------------------------------------------------------
    # Allow sensors to initialize
    # --------------------------------------------------------

    if robot.step(timestep) == -1:
        return


    # ========================================================
    # 3. COUNTERS
    # ========================================================

    sample_count = 0

    rejected_count = 0

    consecutive_failures = 0


    max_attempts = (
        loop_value * 3
    )


    # ========================================================
    # 4. DATA COLLECTION LOOP
    # ========================================================

    for attempt in range(
        1,
        max_attempts + 1
    ):


        # ====================================================
        # Generate random q1
        # ====================================================

        q1_target = random.uniform(
            JOINT_LIMITS[0][0],
            JOINT_LIMITS[0][1]
        )


        # ====================================================
        # Generate random q2
        # ====================================================

        q2_target = random.uniform(
            JOINT_LIMITS[1][0],
            JOINT_LIMITS[1][1]
        )


        # ====================================================
        # Generate random q3
        #
        # IMPORTANT:
        # q3 is linear displacement in METRES.
        # ====================================================

        q3_target = random.uniform(
            JOINT_LIMITS[2][0],
            JOINT_LIMITS[2][1]
        )


        # ====================================================
        # Joint 4 is fixed at zero
        # ====================================================

        target_positions = (
            q1_target,
            q2_target,
            q3_target,
            0.0,
        )


        # ====================================================
        # 5. MOVE SCARA
        # ====================================================

        for index in range(
            len(motors)
        ):

            motors[index].setPosition(
                target_positions[index]
            )


        # ====================================================
        # 6. WAIT FOR SCARA TO STOP
        # ====================================================

        (
            status,
            hand_position,
            joint_positions
        ) = wait_until_settled(
            robot,
            timestep,
            joint_sensors,
            gps,
            target_positions
        )


        # ----------------------------------------------------
        # Simulation closed
        # ----------------------------------------------------

        if status == "stopped":
            return


        # ====================================================
        # Reject failed movement
        # ====================================================

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


        # ----------------------------------------------------
        # Successful movement
        # ----------------------------------------------------

        consecutive_failures = 0


        # ====================================================
        # 7. GET ROBOT BASE POSITION
        # ====================================================

        base_position = (
            base_node.getPosition()
        )


        base_orientation = (
            base_node.getOrientation()
        )


        # ====================================================
        # 8. CONVERT GPS -> ROBOT BASE FRAME
        # ====================================================

        x, y, z = (
            position_in_base_frame(
                hand_position,
                base_position,
                base_orientation
            )
        )


        # ====================================================
        # 9. GET ACTUAL MEASURED JOINT VALUES
        # ====================================================

        measured_q1 = (
            joint_positions[0]
        )


        measured_q2 = (
            joint_positions[1]
        )


        measured_q3 = (
            joint_positions[2]
        )


        # ====================================================
        # 10. SAVE TO SHARED CSV
        # ====================================================

        save_sample_to_csv(
            measured_q1,
            measured_q2,
            measured_q3,
            x,
            y,
            z
        )


        sample_count += 1


        # ====================================================
        # 11. PRINT SAMPLE
        # ====================================================

        print(
            f"[{name}] "
            f"Sample "
            f"{sample_count}/{loop_value} | "
            f"q1={measured_q1:.6f} rad, "
            f"q2={measured_q2:.6f} rad, "
            f"q3={measured_q3:.6f} m | "
            f"x={x:.6f} m, "
            f"y={y:.6f} m, "
            f"z={z:.6f} m",
            flush=True
        )


        # ====================================================
        # Stop after 1000 samples for this robot
        # ====================================================

        if (
            sample_count
            >= loop_value
        ):

            break


    # ========================================================
    # 12. VERIFY DATASET COMPLETION
    # ========================================================

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


    # ========================================================
    # FINISHED
    # ========================================================

    print(
        "\n"
        "====================================",
        flush=True
    )


    print(
        f"[{name}] "
        "SCARA dataset collection complete",
        flush=True
    )


    print(
        f"Accepted samples : "
        f"{sample_count}",
        flush=True
    )


    print(
        f"Rejected samples : "
        f"{rejected_count}",
        flush=True
    )


    print(
        f"Shared CSV       : "
        f"{CSV_FILENAME}",
        flush=True
    )


    print(
        "====================================\n",
        flush=True
    )


    # ========================================================
    # HOLD FINAL POSITION
    # ========================================================

    while robot.step(
        timestep
    ) != -1:

        pass


# ============================================================
# WEBOTS ENTRY POINT
# ============================================================

if __name__ == "__main__":

    from controller import Supervisor

    robot = Supervisor()

    run(robot)