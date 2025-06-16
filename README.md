# Giraffe Robot for Q&A Sessions

This project implements a ceiling-mounted robot designed to autonomously bring a microphone to participants in a conference room during Q&A sessions. It combines URDF modeling, kinematics, trajectory planning, and inverse dynamics control using the Pinocchio robotics library.


---

## 🤖 Robot Specifications

- **Mounting**: Ceiling at 4 meters height
- **Reachable Area**: 5 × 12 meters (horizontal), down to 1 meter height
- **Degrees of Freedom**: 5
  - 2 Revolute joints (spherical base)
  - 1 Prismatic joint (telescopic extension)
  - 2 Revolute joints (microphone orientation)
- **End-effector Task**: Reach any location with a 30° pitch orientation of the microphone

---

## 🧰 Dependencies

- Python 3.8+
- [Pinocchio](https://github.com/stack-of-tasks/pinocchio)
- NumPy
- Matplotlib
- Example Robot Data (for utils)

Install with:

```bash
pip install numpy matplotlib example-robot-data pin
```

---

## 🚀 How to Run

1. Place the `giraffe_robot.urdf` inside a folder named `./urdf`
2. Run the main control script:

```bash
python giraffe_robot_control.py
```

This will:
- Load the robot model
- Compute forward kinematics and Jacobians
- Generate a minimum jerk trajectory
- Apply task-space computed torque control with null-space optimization
- Print control torques and plot the trajectory

---

## 📌 To Do

- Implement a time-based simulation loop with integration
- Add MeshCat or RViz visualization
- Tune PD gains for 7s settling time without overshoot
- Add launch files for ROS/Gazebo integration (optional)

---

## 📷 Visualization

You can visualize the URDF in RViz or using MeshCat via Pinocchio.

---

## 🧠 Authors

- Designed for educational robotics control project.
- Based on [Pinocchio](https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/index.html) dynamics library.

