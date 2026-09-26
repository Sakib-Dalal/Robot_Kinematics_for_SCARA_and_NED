# SCARA swarm data generator

Open `worlds/scara-forward-kinematics-swarm.wbt` in Webots R2025a and press Run.
After editing the world, controller, or robot model, reload the world to restart
all robots with the new settings.

The world contains **25 SCARA robots in a 5 x 5 grid**, spaced 1.5 metres apart
on a 9 x 9 metre floor. Each base is fixed with its origin at world Z = 0.
Every robot runs its own copy of `scara-swarm-data-generator` and prints its name
alongside each data point. The physics timestep is 8 ms.

## Reading the controller

Start at `run(robot)` in
`controllers/scara-swarm-data-generator/scara-swarm-data-generator.py`.
Its numbered comments follow five steps:

1. Connect to the motors, joint sensors, GPS, and robot base.
2. Choose random positions for the two arm joints and vertical slide.
3. Wait until all four joints reach their targets and the hand stops moving.
4. Convert the GPS reading to the robot's coordinates and print the data point.
5. Hold the final pose while the other robots finish.

Change `loop_value` near the top to choose the number of accepted data points
**per robot**. The default is 1000 per robot, or 25,000 for the whole swarm.
Data is printed to the Webots console; the controller does not save a CSV file.

## Joint settings

| Joint | Device | Target range | Unit | Movement speed |
| --- | --- | --- | --- | --- |
| 1: base arm | `base_arm_motor` | -0.73 to 0.73 | radians | 0.5 rad/s |
| 2: elbow | `arm_motor` | -0.83 to 0.83 | radians | 0.5 rad/s |
| 3: vertical slide | `shaft_linear_motor` | -0.15 to 0 | metres | 0.05 m/s |
| 4: shaft rotation | `shaft_rotation_motor` | held at 0 | radians | 0.5 rad/s |

**The third input is a distance in metres.** For example, `-0.10` lowers the
shaft by 10 cm. It is not an angle or a value in millimetres.

The original slide motor allows -0.20 m, but that extends the shaft below the
floor with this ground-level base. Collection stops at -0.15 m to leave clearance.
Check clearance again before widening this range or adding a tool to the hand.
The fourth joint is also controlled and checked, even though its zero angle is
not one of the three varying dataset inputs.

`protos/ScaraWithGPS.proto` is a local variant of the
[Webots R2025a SCARA model](https://github.com/cyberbotics/webots/blob/R2025a/projects/robots/epson/scara_t6/protos/ScaraT6.proto).
It adds a GPS at the hand-slot origin, exposes self-collision checking, and sets
gentle motor accelerations (1 rad/s² for rotations and 0.2 m/s² for the slide).
The upstream geometry, masses, and motor force/torque limits are retained.
The model's visual assets are loaded from the versioned Webots URLs on first use.

## When a data point is accepted

`wait_until_settled()` requires every rotating joint to be within 0.005 rad of
its target and the slide within 0.0005 m. Rotating joint speeds must be at most
0.01 rad/s, slide speed at most 0.001 m/s, and hand speed at most 0.001 m/s.
All checks must pass continuously for 0.5 simulated seconds.

Each movement has a 20-second simulation-time deadline. A timed-out pose is
logged and replaced without counting it as a sample. Invalid readings, five
consecutive failures, or exceeding three attempts per requested sample stop
collection with an error. Reload the world after a simulation fault.

## Understanding the output

Each line contains the robot's name, three requested joint positions, three
**measured** joint positions, and hand XYZ in metres. The joint order is always
base arm rotation, elbow rotation, then vertical slide: `(rad, rad, m)`.
Pair the measured joint values with the printed XYZ when using the data.

The hand position is measured from that robot's base origin, along the base's
own axes. `Supervisor` supplies the base position and rotation, and
`position_in_base_frame()` removes them from the world GPS reading using
`p_base = R_base.T * (p_world - t_base)`.
Robots in different grid positions therefore use the same coordinate convention.
For the unrotated bases on the floor, printed Z is the hand-slot height above the
floor. No extra grid or height offset should be subtracted.

## Checks

Run the controller checks without opening Webots:

```sh
python3 -B -m unittest discover -s SCARA/tests -v
```

These cover coordinate conversion, movement settling, slide-specific units and
tolerances, shaft rotation, failure handling, missing devices, and completion.

Validated in Webots R2025a with **100 poses per robot (2500 samples total)**:
all eight range corners, a 5 x 5 arm-joint sweep with the slide at -0.075 m,
then 67 seeded random poses per robot. All robots completed with no rejected
movements. Contact checks at every 8 ms step detected no collisions, and the
hand GPS stayed at least 0.0305 m above the floor throughout the run.
The 33 shared poses produced matching base-relative XYZ to six decimal places
across all 25 grid positions. All 25 controller checks passed as well.
