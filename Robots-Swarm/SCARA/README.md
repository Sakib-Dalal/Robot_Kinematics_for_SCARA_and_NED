# SCARA swarm data generator

[Project overview and media](../../README.md#media)

Open [scara-forward-kinematics-swarm.wbt](worlds/scara-forward-kinematics-swarm.wbt)
in Webots R2025a and press **Run**.
After editing the world, controller, or robot model, reload the world to restart
all robots with the new settings.
Use Python 3 on macOS or Linux; the CSV writer uses the Unix-only `fcntl` module.

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
4. Convert the GPS reading to the robot's coordinates, save the data point, and print it.
5. Hold the final pose while the other robots finish.

Change `loop_value` near the top to choose the number of accepted data points
**per robot**. The default is 1000 per robot, or 25,000 for the whole swarm.
Accepted samples are appended to a shared CSV file and printed to the Webots
console. Set `CSV_FILENAME` to choose the output file.

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

Each sample line contains the robot's name, sample count, three **measured**
joint positions, and hand XYZ in metres. Requested targets are not included in
the current sample line. The joint order is always
base arm rotation, elbow rotation, then vertical slide: `(rad, rad, m)`.
Pair the measured joint values with the printed XYZ when using the data.

The hand position is measured from that robot's base origin, along the base's
own axes. `Supervisor` supplies the base position and rotation, and
`position_in_base_frame()` removes them from the world GPS reading using
`p_base = R_base.T * (p_world - t_base)`.
Robots in different grid positions therefore use the same coordinate convention.
For the unrotated bases on the floor, printed Z is the hand-slot height above the
floor. No extra grid or height offset should be subtracted.

## CSV dataset

The included [scara_dataset.csv](controllers/scara-swarm-data-generator/scara_dataset.csv)
contains 25,000 data rows. The default filename is `scara_dataset.csv`, relative
to the controller's working directory, normally the controller folder in Webots.
All 25 arms append to the same file using an exclusive file lock, with one header:

```csv
is_scara,is_ned,q1,q2,q3,x,y,z
```

SCARA rows use `is_scara=1` and `is_ned=0`. `q1` and `q2` are measured angles
in radians, `q3` is the measured slide position in metres, and XYZ is in metres
relative to the base. The CSV does not include a robot-instance name, timestamp,
or run identifier.

The default 1,000 samples per robot add 25,000 rows when every arm completes.
Existing data is preserved and appended to on later runs. Change `CSV_FILENAME`
before starting a separate collection if you want to preserve the included dataset.
The repository's [ignore rules](../../.gitignore) keep this controller-folder
dataset tracked while ignoring the same filename at the repository root,
where regression tests can create temporary samples.

## Checks

Run the controller checks from the repository root without opening Webots:

```sh
python3 -B -m unittest discover -s Robots-Swarm/SCARA/tests -v
```

These cover coordinate conversion, movement settling, slide-specific units and
tolerances, shaft rotation, failure handling, missing devices, and completion.
Collection tests can append CSV samples in the working directory; use a temporary
working directory and an absolute test path to isolate them. The suite currently
has failures involving console/error messages and shutdown motor-command counts.
See the [recorded test results](../../README.md#tests).

### Historical simulation validation

Earlier project notes report a Webots R2025a run with
**100 poses per robot (2500 samples total)**:
all eight range corners, a 5 x 5 arm-joint sweep with the slide at -0.075 m,
then 67 seeded random poses per robot. All robots completed with no rejected
movements. Contact checks at every 8 ms step detected no collisions, and the
hand GPS stayed at least 0.0305 m above the floor throughout the run.
The 33 shared poses produced matching base-relative XYZ to six decimal places
across all 25 grid positions. Those notes also reported 25 passing controller
checks at that time. This is a record of that earlier run, not a fresh validation
of the current controller revision; the current test status is described above.
