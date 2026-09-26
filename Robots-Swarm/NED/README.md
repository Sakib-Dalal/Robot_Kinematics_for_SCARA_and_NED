# NED swarm data generator

[Project overview and media](../../README.md#media)

Open [ned-forward-kinematics-swarm.wbt](worlds/ned-forward-kinematics-swarm.wbt)
in Webots R2025a and press **Run**. After changing the world, controller, or PROTO,
**reload the world** so Webots loads the new settings and restarts the controllers.
Use Python 3 on macOS or Linux; the CSV writer uses the Unix-only `fcntl` module.

## Reading the controller

Start at `run(robot)` in
`controllers/ned-swarm-data-generator/ned-swarm-data-generator.py`.
Its numbered comments follow the main steps: connect to the devices, choose
random angles, wait for the arm to stop, save and print a data point, and hold the final pose.
Each robot runs its own copy of this controller.

- Change `loop_value` near the top to choose the number of data points per robot.
- `wait_until_settled()` checks the measured angles and movement before accepting a pose.
- `position_in_base_frame()` converts the hand GPS position into coordinates
  measured from that robot's base. `Supervisor` supplies the base position and rotation.

The controller appends accepted samples to a shared CSV file and prints progress
to the Webots console. Set `CSV_FILENAME` to choose the output file.

## Simulation and data collection

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
zero. A sample is saved and printed only after all six joints are within 0.005 rad of
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

Each move has a 20-second simulation-time deadline. A timed-out move is logged
and replaced with a new target, without counting it as a sample. Five consecutive
failures, invalid sensor readings, or exhausting three attempts per requested
sample ends collection with an explicit error. Reload the world after a physics
fault. On normal completion, the controller reports its sample count and holds
the last pose while other arms finish.

## Understanding the output

Console samples include the robot name, sample count, **measured angles**
(`q1`, `q2`, `q3`, in radians), and hand position (`x`, `y`, `z`, in metres)
in the **robot base frame**. Requested targets are not included in the current
sample line, and the line does not carry a `base frame, m` label.
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
For older recordings, check the controller version to identify the coordinate
frame; the presence or absence of a console label alone does not establish it.
Current rejected-move messages contain the robot name, attempt number, and failure reason.

## CSV dataset

The included [ned_dataset.csv](controllers/ned-swarm-data-generator/ned_dataset.csv)
contains 25,000 data rows. The default filename is `ned_dataset.csv`, relative
to the controller's working directory, normally the controller folder in Webots.
All 25 arms append to the same file using an exclusive file lock, with one header:

```csv
is_scara,is_ned,q1,q2,q3,x,y,z
```

NED rows use `is_scara=0` and `is_ned=1`. All three joint values are measured
angles in radians; XYZ is in metres relative to the base. The CSV does not
include a robot-instance name, timestamp, or run identifier.

The default 1,000 samples per robot add 25,000 rows when every arm completes.
Existing data is preserved and appended to on later runs. Change `CSV_FILENAME`
before starting a separate collection if you want to preserve the included dataset.
The repository's [ignore rules](../../.gitignore) keep this controller-folder
dataset tracked while ignoring the same filename at the repository root,
where regression tests can create temporary samples.

## Checks

Run the controller regression checks from the repository root:

```sh
python3 -B -m unittest discover -s Robots-Swarm/NED/tests -v
```

These checks use fake devices and do not need Webots. Collection tests can append
CSV samples in the working directory; use a temporary working directory and an
absolute test path to isolate them. The suite currently has failures involving
console/error messages and shutdown motor-command counts. See the
[recorded test results](../../README.md#tests).

### Historical simulation validation

Earlier project notes report a Webots R2025a run with 100 samples per arm
(2500 total): all eight limit corners, a 5×5 shoulder/elbow sweep, then seeded
random targets. Contact tracking at every 8 ms physics step reported no collisions,
and every arm completed without rejected moves or broken joints. The 33 shared
test poses gave identical base-frame XYZ to six decimal places across all 25
grid positions. This is a record of that earlier run, not a fresh validation of
the current controller revision.
