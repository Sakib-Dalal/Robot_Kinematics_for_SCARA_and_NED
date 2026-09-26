# NED swarm data generator

Open `worlds/ned-forward-kinematics-swarm.wbt` in Webots R2025a. After changing
the world or PROTO, **reload the world** so Webots loads the new physics and
motor settings. Reloading restarts the simulation and controllers.

The 25 arms sit directly on the floor, with every robot base at **world Z = 0**.
They use a 1.5 m grid and an 8 ms physics timestep. There are no raised mounts.
Self-collision detection is enabled on every robot. The gripper sliders have
mechanical stops at ±0.01 m, preventing external forces from pushing the fingers
far out of travel. The unloaded arm's simulation motor limits are 10 Nm for
joints 1–3 and 3 Nm for joints 4–6,
with a maximum speed of 0.5 rad/s and acceleration of 1 rad/s². These are
simulation settings, not the physical robot's rated limits.

Set `loop_value` in the controller to choose the number of accepted samples
per robot (default: 1000). Joints 1–3 move to random targets; joints 4–6 hold
zero. A sample is printed only after all six joints are within 0.005 rad of
their targets, joint speeds are at most 0.01 rad/s, and hand speed is at most
0.001 m/s for 0.5 simulated seconds.

The ground-level sampling limits are:

| Joint | Minimum (rad) | Maximum (rad) |
| --- | ---: | ---: |
| 1 | -2.7 | 2.7 |
| 2 | -0.7 | 0.4 |
| 3 | -1.4 | 0.8 |
| 4–6 | 0 | 0 |

The forward shoulder and elbow limits keep the gripper clear of the floor and
the robot's base during movement. The previous upper limits (0.7 and 1.2 rad)
allowed floor collisions. These are dataset sampling limits, not changes to
the robot's mechanical travel. Recheck clearance before widening them or moving
the wrist joints.

Validated in Webots R2025a with 100 samples per arm (2500 total): all eight limit
corners, a 5×5 shoulder/elbow sweep, then seeded random targets. Contact tracking
at every 8 ms physics step reported no collisions, and every arm completed
without rejected moves or broken joints. The 33 shared test poses gave identical
base-frame XYZ to six decimal places across all 25 grid positions.

Each move has a 20-second simulation-time deadline. A timed-out move is logged
and replaced with a new target, without counting it as a sample. Five consecutive
failures, invalid sensor readings, or exhausting three attempts per requested
sample ends collection with an explicit error. Reload the world after a physics
fault. On normal completion, the controller reports its sample count and holds
the last pose while other arms finish.

Output includes the robot name, requested angles, **measured angles**, and hand
position in metres in the **robot base frame**, labelled `base frame, m`.
The origin `(0, 0, 0)` is each NED robot's base origin on the floor,
and the axes follow that robot's base. Use measured angles when pairing joint
positions with these coordinates.

The controller automatically converts the world GPS reading using
`p_base = R_base.T * (p_world - t_base)`. It reads the actual base position and
orientation through Webots' Supervisor API; every robot in this swarm world has
`supervisor TRUE` enabled for that purpose. There are no hardcoded height or
grid offsets. Raising, moving, or rotating a base therefore does not change the
reported position for the same arm configuration, apart from simulation error.
For example, a base at `(3, -1.5, 0)` and world hand position `(3.1, -1.2, 0.4)`
produce `(0.1, 0.3, 0.4)` when the base is unrotated.

For this world, X and Y exclude each robot's grid position, and Z is the hand's
height above the floor. **Do not subtract any extra offset** from the printed data.
Older output without the `base frame, m` label used world coordinates and must
be converted before mixing it with new samples. Rejected-move diagnostics retain
the raw GPS reading, explicitly labelled `world_GPS`.
The controller prints samples to the console; it does not save a CSV file.

Run the controller regression checks from the repository root:

```sh
python3 -m unittest discover -s NED/tests -v
```
