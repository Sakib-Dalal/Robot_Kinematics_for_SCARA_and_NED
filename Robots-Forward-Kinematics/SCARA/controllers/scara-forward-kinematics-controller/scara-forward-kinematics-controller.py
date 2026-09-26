from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# q1 and q2 are radians; q3 is metres.
q1 = 0.0
q2 = 0.0
q3 = -1.0

# Move the three joints.
joint_names = ('base_arm_motor', 'arm_motor', 'shaft_linear_motor')
for name, target in zip(joint_names, (q1, q2, q3)):
    motor = robot.getDevice(name)
    motor.setVelocity(0.5)
    motor.setPosition(target)

gps = robot.getDevice("gps")
if gps is None:
    raise RuntimeError("Reopen worlds/scara_robot.wbt to load the GPS.")
gps.enable(timestep)

# Print once after the tool position stays still for 0.5 seconds.
previous_position = None
still_time = 0
position_printed = False

while robot.step(timestep) != -1:
    if position_printed:
        continue

    position = gps.getValues()
    if previous_position is not None and all(
        abs(current - previous) <= 0.000001
        for current, previous in zip(position, previous_position)
    ):
        still_time += timestep
    else:
        still_time = 0
    previous_position = position

    if still_time >= 500:
        x, y, z = position
        print(f"SCARA Final Position -> X: {x:.4f}, Y: {y:.4f}, Z: {z:.4f}", flush=True)
        position_printed = True