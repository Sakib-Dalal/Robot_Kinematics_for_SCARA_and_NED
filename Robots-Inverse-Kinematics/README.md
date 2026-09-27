# Inverse kinematics

Both robots use the trained network and scalers in `Notebook/Models` to move their hand toward a blue ball. The controllers follow the same steps as `Notebook/Model.ipynb`: scale the position, predict three joint values, undo the target scaling, and move the motors.

## Run

1. Open [the NED world](NED/worlds/ned-inverse-kinematics.wbt) or [the SCARA world](SCARA/worlds/scara-inverse-kinematics.wbt) in Webots.
2. Press **Run**. The arm moves toward the blue ball.
3. Select **TARGET** in the scene tree and change its **translation** to test another position. The controller automatically predicts new joint values.
4. Read the Webots console for the predicted joints and the distance from the hand GPS to the ball's centre after the joints settle.

The ball is a fixed visual marker without collision geometry or gravity, so the hand can reach its centre. Its translation is in world coordinates; each controller converts it to the robot's base coordinates before making a prediction. Rotating or moving the robot's base is supported.

## Python and model files

The `runtime.ini` files point directly to this checkout's `Notebook/.venv/bin/python`, so Webots uses the notebook's installed packages. If this environment is missing, run `uv sync` from the `Notebook` folder. If you move the project or use another computer, change `COMMAND` in both runtime files to the full path of that computer's notebook Python interpreter. On Windows, this is `Notebook/.venv/Scripts/python.exe`.

Both controllers load these files directly, without copying or retraining them:

- `Notebook/Models/model.safetensors`
- `Notebook/Models/input_scaler.joblib`
- `Notebook/Models/target_scaler.joblib`

The network matches the notebook's `5 → 128 → 256 → 128 → 3` layers with ReLU activations. If you retrain this architecture, run the notebook's export cells and restart the simulation to load the new files. Changing the network architecture also requires updating the controller classes.

## Joints and accuracy

| Robot | Model inputs | Predicted joints | Joints held at zero |
| --- | --- | --- | --- |
| NED | `0, 1, x, y, z` | `joint_1`, `joint_2`, `joint_3` | `joint_4`, `joint_5`, `joint_6` |
| SCARA | `1, 0, x, y, z` | `base_arm_motor`, `arm_motor`, `shaft_linear_motor` | `shaft_rotation_motor` |

NED's three predictions are radians. For SCARA, the first two are radians and the third is a slide distance in metres. Predictions are limited to the ranges used to collect the training data:

| Robot | q1 | q2 | q3 |
| --- | --- | --- | --- |
| NED | −2.7 to 2.7 | −0.7 to 0.4 | −1.4 to 0.8 |
| SCARA | −0.73 to 0.73 | −0.83 to 0.83 | −0.15 to 0 |

The saved model gives an approximate solution. Its training data includes different joint configurations for similar hand positions, so the hand may stop short of the ball even when the motors reach the predicted joints. The reported distance measures this error; the controller does not use GPS to correct the prediction. Moving the ball outside the trained workspace will not produce a reliable solution.
