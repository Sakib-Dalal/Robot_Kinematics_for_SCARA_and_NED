# Copyright 1996-2024 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0

"""SCARA keyboard controller."""

from controller import Robot, Keyboard

# ---------------------------------------------------------
# Initialize robot
# ---------------------------------------------------------

robot = Robot()
timeStep = int(robot.getBasicTimeStep())

# ---------------------------------------------------------
# Keyboard
# ---------------------------------------------------------

keyboard = robot.getKeyboard()
keyboard.enable(timeStep)

# ---------------------------------------------------------
# Motors
# ---------------------------------------------------------

base_arm = robot.getDevice("base_arm_motor")
arm = robot.getDevice("arm_motor")
shaft = robot.getDevice("shaft_linear_motor")

# Position sensors
base_arm_pos = robot.getDevice("base_arm_position")
arm_pos = robot.getDevice("arm_position")

base_arm_pos.enable(timeStep)
arm_pos.enable(timeStep)

# Optional LED
led = robot.getDevice("epson_led")

# ---------------------------------------------------------
# Motor speeds
# ---------------------------------------------------------

base_arm.setVelocity(1.0)
arm.setVelocity(1.0)
shaft.setVelocity(0.1)

# ---------------------------------------------------------
# Initial positions
# ---------------------------------------------------------

base_target = 0.0
arm_target = 0.0
shaft_target = 0.0

base_arm.setPosition(base_target)
arm.setPosition(arm_target)
shaft.setPosition(shaft_target)

# ---------------------------------------------------------
# Movement step size
# ---------------------------------------------------------

ROTATION_STEP = 0.02     # radians
SHAFT_STEP = 0.005       # meters

# Get actual limits from Webots motor definition
base_min = base_arm.getMinPosition()
base_max = base_arm.getMaxPosition()

arm_min = arm.getMinPosition()
arm_max = arm.getMaxPosition()

shaft_min = shaft.getMinPosition()
shaft_max = shaft.getMaxPosition()


def clamp(value, minimum, maximum):
    return max(minimum, min(value, maximum))


print("---------------------------------------")
print("SCARA Keyboard Controller")
print("---------------------------------------")
print("LEFT / RIGHT : Base joint")
print("UP / DOWN    : Arm joint")
print("W / S        : Shaft up/down")
print("R            : Reset robot")
print("---------------------------------------")


# ---------------------------------------------------------
# Main simulation loop
# ---------------------------------------------------------

while robot.step(timeStep) != -1:

    key = keyboard.getKey()

    while key != -1:

        # -------------------------------------------------
        # Joint 1 - Base
        # -------------------------------------------------

        if key == Keyboard.LEFT:
            base_target += ROTATION_STEP

        elif key == Keyboard.RIGHT:
            base_target -= ROTATION_STEP

        # -------------------------------------------------
        # Joint 2 - Arm
        # -------------------------------------------------

        elif key == Keyboard.UP:
            arm_target += ROTATION_STEP

        elif key == Keyboard.DOWN:
            arm_target -= ROTATION_STEP

        # -------------------------------------------------
        # Joint 3 - Linear shaft
        # -------------------------------------------------

        elif key == ord('W'):
            shaft_target += SHAFT_STEP

        elif key == ord('S'):
            shaft_target -= SHAFT_STEP

        # -------------------------------------------------
        # Reset
        # -------------------------------------------------

        elif key == ord('R'):
            base_target = 0.0
            arm_target = 0.0
            shaft_target = 0.0

            print("Robot reset")

        # Get next key if multiple keys are waiting
        key = keyboard.getKey()

    # -----------------------------------------------------
    # Keep joints inside their limits
    # -----------------------------------------------------

    base_target = clamp(
        base_target,
        base_min,
        base_max
    )

    arm_target = clamp(
        arm_target,
        arm_min,
        arm_max
    )

    shaft_target = clamp(
        shaft_target,
        shaft_min,
        shaft_max
    )

    # -----------------------------------------------------
    # Move robot
    # -----------------------------------------------------

    base_arm.setPosition(base_target)
    arm.setPosition(arm_target)
    shaft.setPosition(shaft_target)