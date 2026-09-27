from pathlib import Path

import joblib
import numpy as np
import pandas as pd
import torch
import torch.nn as nn
from controller import Supervisor
from safetensors.torch import load_file


class InverseKinematicNet(nn.Module):
    def __init__(self, input_size, output_size):
        super(InverseKinematicNet, self).__init__()

        self.layer1 = nn.Linear(input_size, 128)
        self.layer2 = nn.Linear(128, 256)
        self.layer3 = nn.Linear(256, 128)
        self.layer4 = nn.Linear(128, output_size)
        self.relu = nn.ReLU()

    def forward(self, x):
        x = self.relu(self.layer1(x))
        x = self.relu(self.layer2(x))
        x = self.relu(self.layer3(x))
        return self.layer4(x)


model_path = Path(__file__).resolve().parents[4] / "Notebook" / "Models"
input_scaler = joblib.load(model_path / "input_scaler.joblib")
target_scaler = joblib.load(model_path / "target_scaler.joblib")

torch.set_num_threads(1)
model = InverseKinematicNet(input_size=5, output_size=3)
model.load_state_dict(load_file(str(model_path / "model.safetensors")))
model.eval()

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())
base = robot.getSelf()
ball = robot.getFromDef("TARGET")

joint_names = ["base_arm_motor", "arm_motor", "shaft_linear_motor", "shaft_rotation_motor"]
motors = [robot.getDevice(name) for name in joint_names]
sensors = [motor.getPositionSensor() for motor in motors]
speeds = [0.5, 0.5, 0.05, 0.5]

for motor, sensor, speed in zip(motors, sensors, speeds):
    motor.setVelocity(speed)
    motor.setPosition(0.0)
    sensor.enable(timestep)

gps = robot.getDevice("gps")
gps.enable(timestep)

previous_target = None
settled_time = 0
reported = False

while robot.step(timestep) != -1:
    ball_position = np.array(ball.getPosition())
    base_position = np.array(base.getPosition())
    base_rotation = np.array(base.getOrientation()).reshape(3, 3)
    target = base_rotation.T @ (ball_position - base_position)

    if previous_target is None or np.linalg.norm(target - previous_target) > 0.0001:
        X = pd.DataFrame([[1, 0, *target]], columns=["is_scara", "is_ned", "x", "y", "z"])
        X_scaled = input_scaler.transform(X)
        X_ts = torch.tensor(X_scaled, dtype=torch.float32)

        with torch.no_grad():
            y_pred = model(X_ts).numpy()

        q = target_scaler.inverse_transform(y_pred)[0]
        q = np.clip(q, [-0.73, -0.83, -0.15], [0.73, 0.83, 0.0])
        joint_targets = [*q, 0.0]

        for motor, position in zip(motors, joint_targets):
            motor.setPosition(float(position))

        print(f"SCARA target: {target.round(4)} | Predicted joints: {q.round(4)}", flush=True)
        previous_target = target.copy()
        settled_time = 0
        reported = False

    joint_positions = np.array([sensor.getValue() for sensor in sensors])
    if np.all(np.abs(joint_positions - joint_targets) < [0.005, 0.005, 0.0005, 0.005]):
        settled_time += timestep
    else:
        settled_time = 0

    if settled_time >= 500 and not reported:
        distance = np.linalg.norm(np.array(gps.getValues()) - ball_position)
        print(f"SCARA distance to blue ball: {distance:.4f} m", flush=True)
        reported = True
