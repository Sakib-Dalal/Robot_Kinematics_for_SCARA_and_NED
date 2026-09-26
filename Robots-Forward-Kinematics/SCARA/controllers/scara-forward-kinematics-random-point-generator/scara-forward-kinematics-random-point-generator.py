from controller import Robot
import random

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Joint targets in radians.
q1 = 0.0
q2 = 0.0
q3 = 0.0

# Move the three joints.
joint_names = ('base_arm_motor', 'arm_motor', 'shaft_linear_motor')
motors = []

for name in joint_names:
    motors.append(robot.getDevice(name))

gps = robot.getDevice("gps")
if gps is None:
    raise RuntimeError("Reopen worlds to load the GPS.")
gps.enable(timestep)

# set the amount of datapoints you want to record
loop_value = 1000

# scara robot configurations/motor limits
q1_rotation_limits = (-0.73, 0.73) # in radians
q2_rotation_limits = (-0.83, 0.83) # in radians
q3_linear_limits = (-0.2, 0) # in mm


for i in range(1000):
    # randomly generated values
    q1 = random.uniform(q1_rotation_limits[0], q1_rotation_limits[1])
    q2 = random.uniform(q2_rotation_limits[0], q2_rotation_limits[1])
    q3 = random.uniform(q3_linear_limits[0], q3_linear_limits[1])

    # Move the three joints
    for motor, target in zip(motors, (q1, q2, q3)):
        motor.setVelocity(0.5)
        motor.setPosition(target)

    previous_position = None
    still_time = 0

    # Wait for the robot to settle at the random position
    while robot.step(timestep) != -1:
        position = gps.getValues()

        # Check if position has barely changed
        if previous_position is not None and all(
            abs(current - previous) <= 0.000001
            for current, previous in zip(position, previous_position)
            ):
            still_time += timestep
        else:
            still_time = 0

        previous_position = position

        # Once the position is still for 0.5 seconds (500 ms)
        if still_time >= 500:
            x, y, z = position
            print(f"Iteration {i+1}/1000 - Targets: ({q1:.2f}, {q2:.2f}, {q3:.2f}) | Ned Final Position -> X: {x:.4f}, Y: {y:.4f}, Z: {z:.4f}", flush=True)
            break # Break the while loop to start the next iteration of the for loop