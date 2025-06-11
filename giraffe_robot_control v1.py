# Giraffe Robot: URDF + Pinocchio Setup + Trajectory Planning + Control

import pinocchio as pin
from pinocchio.robots import RobotWrapper
from pinocchio.utils import zero
import numpy as np
import example_robot_data as robex
import matplotlib.pyplot as plt

# 1. Load URDF Model
model_path = "./urdf"
mesh_dir = model_path
urdf_filename = "giraffe_robot.urdf"
urdf_model_path = model_path + "/" + urdf_filename

# Load the robot
robot = RobotWrapper.BuildFromURDF(urdf_model_path, [mesh_dir])
model = robot.model
data = robot.data

# 2. Set Initial Configuration
q_home = pin.neutral(model)
robot.q0 = q_home

# 3. Forward Kinematics
pin.forwardKinematics(model, data, q_home)
end_effector_frame = model.getFrameId("microphone_link")
pin.updateFramePlacement(model, data, end_effector_frame)
print("End-effector position:", data.oMf[end_effector_frame].translation)

# 4. Compute Jacobian
J = pin.computeFrameJacobian(model, data, q_home, end_effector_frame, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
print("Jacobian:\n", J)

# 5. Polynomial Trajectory (Minimum Jerk)
def min_jerk_trajectory(p0, pf, T, dt):
    times = np.arange(0, T, dt)
    traj = []
    for t in times:
        tau = t / T
        pos = p0 + (pf - p0) * (10 * tau**3 - 15 * tau**4 + 6 * tau**5)
        traj.append(pos)
    return times, np.array(traj)

p0 = np.array([0.0, 0.0, 0.0])
pf = np.array([1.0, 2.0, 1.0])
T = 7.0
dt = 0.01
times, trajectory = min_jerk_trajectory(p0, pf, T, dt)

# 6. Plot Trajectory
plt.plot(times, trajectory)
plt.title("Minimum Jerk Trajectory")
plt.xlabel("Time [s]")
plt.ylabel("Position")
plt.legend(["x", "y", "z"])
plt.grid()
plt.show()

# 7. Inverse Dynamics Controller (Computed Torque Control)
Kp = np.diag([100.0] * model.nv)
Kd = np.diag([20.0] * model.nv)

q = q_home.copy()
dq = np.zeros(model.nv)

def compute_control(q, dq, q_des, dq_des, ddq_des):
    pin.forwardKinematics(model, data, q, dq, np.zeros(model.nv))
    M = pin.crba(model, data, q)
    b = pin.rnea(model, data, q, dq, np.zeros(model.nv))
    tau = M @ (ddq_des + Kd @ (dq_des - dq) + Kp @ (q_des - q)) + b
    return tau

# 8. Null-Space Optimization (towards preferred q0)
q0 = q_home.copy()
K_null = 5.0

def apply_null_space_control(tau, J):
    J_pinv = np.linalg.pinv(J)
    null_proj = np.eye(model.nv) - J.T @ J_pinv.T
    tau_null = -K_null * (q - q0)
    return tau + null_proj @ tau_null

# 9. Simulate Tracking (Placeholder: one step)
q_des = q_home.copy()
dq_des = np.zeros(model.nv)
ddq_des = np.zeros(model.nv)

# Compute base torque
tau = compute_control(q, dq, q_des, dq_des, ddq_des)

# Add null-space term
tau_total = apply_null_space_control(tau, J)
print("Control torques (with null-space):", tau_total)

# TODO: Integrate over time to simulate tracking trajectory
# TODO: Tune Kp, Kd for 7-second settling time with no overshoot
