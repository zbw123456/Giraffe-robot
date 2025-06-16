# 🦒 Giraffe Robot Control (C++ Version)

This C++ project implements the modeling, kinematics, and control of a ceiling-mounted robotic arm ("Giraffe Robot") for automated microphone delivery during Q&A sessions.

## 📁 Project Structure

```
giraffe_robot_cpp/
├── CMakeLists.txt
├── include/
│   ├── trajectory.hpp         # Min-jerk trajectory planner
│   └── controller.hpp         # PD controller with CRBA + RNEA
├── src/
│   └── main.cpp               # Main test and demo executable
├── urdf/
│   └── giraffe_robot.urdf     # Your URDF model goes here
```

## 🔧 Dependencies

- [Pinocchio](https://github.com/stack-of-tasks/pinocchio) (C++ version)
- [Eigen3](https://eigen.tuxfamily.org/)
- CMake ≥ 3.1

## 🛠️ Build Instructions

```bash
cd giraffe_robot_cpp
mkdir build && cd build
cmake ..
make
./giraffe_control
```

## 🧠 Functionality

- ✅ Load URDF model of a redundant 5-DoF robot
- ✅ Compute forward kinematics & Jacobian of end-effector
- ✅ Trajectory planning using 5th-order min-jerk curves
- ✅ Task-space PD control with torque computation via:
  - CRBA (inertia matrix)
  - RNEA (nonlinear dynamics)
- ✅ Modular code structure for easy extension

## 🚀 Future Extensions

- [ ] Integrate ROS for real-time deployment
- [ ] Add inverse kinematics solver
- [ ] Export trajectory logs (e.g., CSV)
- [ ] Real-time control with sensor feedback

## 📎 Notes

- Replace `urdf/giraffe_robot.urdf` with your actual robot model.
- The default controller tracks a static configuration and prints the end-effector position + Jacobian.
