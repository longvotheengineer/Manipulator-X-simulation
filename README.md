# Manipulator-X: 4-DOF Robotic Arm Simulation

[![MATLAB](https://img.shields.io/badge/MATLAB-R2020b%2B-orange.svg?style=flat&logo=matlab)](https://www.mathworks.com/products/matlab.html)
[![Status](https://img.shields.io/badge/Status-Active-brightgreen.svg?style=flat)]()
[![License](https://img.shields.io/badge/License-MIT-blue.svg?style=flat)]()
[![Platform](https://img.shields.io/badge/Platform-Windows%20%7C%20Linux%20%7C%20macOS-lightgrey.svg?style=flat)]()

<p align="center">
  <img width="1751" height="1000" alt="1" src="https://github.com/user-attachments/assets/86b21c6d-c659-40cd-b65d-c662271e3b94" />
  <br>
  <em>Figure 1: The Interactive Manipulator-X GUI in Action</em>
</p>

## Table of Contents
- [Project Overview](#-project-overview)
- [Key Features](#-key-features--technical-highlights)
- [Technical Implementation](#-technical-implementation-details)
  - [Forward Kinematics](#1-forward-kinematics-fk)
  - [Inverse Kinematics](#2-inverse-kinematics-ik)
  - [Trajectory Generation](#3-trajectory-generation-cubic-polynomials)
  - [Jacobian & Performance](#4-performance-analysis-and-jacobian)
- [Configuration](#-manipulator-configuration)
- [Repository Structure](#-repository-structure)
- [Getting Started](#-getting-started)
- [Future Improvements](#-potential-extensions-and-improvements)
- [License](#-license)

---

## Project Overview

[cite_start]**Manipulator-X** is a robust simulation and control framework for a custom 4 Degree-of-Freedom (DOF) serial robotic manipulator. Developed entirely within **MATLAB**, this project bridges the gap between theoretical robotics and practical application.

The system serves as a comprehensive demonstration of:
* **Kinematic Analysis** (Forward & Inverse)
* **Path Planning** (Joint & Cartesian Space)
* **Real-time Visualization** (High-fidelity 3D modeling)

[cite_start]Users can interact with the robot via a custom **App Designer GUI**, enabling instant analysis of motion profiles, including Position, Velocity, and Acceleration.

---

## Key Features & Technical Highlights

* ** High-Fidelity Visualization**
    Detailed 3D geometric modeling using `hgtransform` and `patch` primitives allows for realistic rendering of the robot's physical structure.
* ** Complete Kinematic Engine**
    Built-in Forward and Inverse Kinematics solutions form the foundation of the control system, allowing for precise end-effector positioning.
* ** Smooth Trajectory Generation**
    Utilizes **Cubic Polynomials** to generate motion profiles. This ensures zero initial and final velocity, guaranteeing safe and smooth transitions between waypoints.
* ** Dual-Space Planning**
    Versatile control strategies allow for planning in both **Joint Space** (controlling angles) and **Cartesian Space** (controlling XYZ position).
* ** Real-time Performance Analytics**
    Dynamically calculates and plots the End-Effector velocity (derived via the Jacobian matrix) and joint-level kinematics against time.
* ** Workspace Mapping**
    Includes a dedicated function to generate and visualize the total reachable operational space based on physical joint limits.

---

## Technical Implementation Details

### 1. Forward Kinematics (FK)
The FK solution (`ForwardKinematics.m`) employs the standard **Denavit-Hartenberg (D-H)** convention. [cite_start]The homogeneous transformation matrix $\mathbf{A}_{i}^{0}$ is calculated sequentially:

$$\mathbf{A}_{i}^{0} = \mathbf{A}_{1}^{0} \mathbf{A}_{2}^{1} \cdots \mathbf{A}_{i}^{i-1}$$

[cite_start]The final end-effector position $P_{EE}(x, y, z)$ is extracted directly from the last column of $\mathbf{A}_{5}^{0}$.

### 2. Inverse Kinematics (IK)
Crucial for Cartesian control, the IK solution (`InverseKinematics.m`) uses a **geometric approach**. [cite_start]It solves for joint angles ($\theta_1$ to $\theta_4$) based on the arm's geometry ($L_1$ to $L_5$).

> [cite_start]**Note:** The algorithm first calculates the wrist center $(p_{x4}, p_{y4}, p_{z4})$ by offsetting the end-effector position by length $L_5$ along the fixed pitch axis.

### 3. Trajectory Generation (Cubic Polynomials)
To ensure smooth start/stop motion, the system uses third-order polynomials satisfying zero-velocity boundary conditions. [cite_start]The profile $q(t)$ is defined as:

$$q(t) = a_0 + a_1 t + a_2 t^2 + a_3 t^3$$

**Coefficients:**
* $a_0 = q_0$
* $a_1 = 0$
* $a_2 = \frac{3(q_f - q_0)}{T^2}$
* $a_3 = \frac{-2(q_f - q_0)}{T^3}$

### 4. Performance Analysis and Jacobian
[cite_start]The End-Effector velocity $\dot{P}_{EE}$ is computed in `PathPlanning.m` using the **Jacobian matrix** $\mathbf{J}(\mathbf{\theta})$:

$$\dot{P}_{EE} = \mathbf{J}(\mathbf{\theta}) \dot{\mathbf{\theta}}$$

[cite_start]The system implements a $6 \times 4$ Jacobian matrix to map joint velocities to Cartesian linear and angular velocities.

---

## Manipulator Configuration

[cite_start]The robot's physical dimensions are defined in `main.m` as follows:

| Parameter | Variable | Value (m) | Description |
| :--- | :---: | :---: | :--- |
| **Link 1** | $L_1$ | `0.077` | Base link height |
| **Link 2** | $L_2$ | `0.128` | First arm segment length |
| **Link 3** | $L_3$ | `0.024` | Second joint offset |
| **Link 4** | $L_4$ | `0.124` | Forearm segment length |
| **Link 5** | $L_5$ | `0.126` | End-effector/Gripper length |

---

## Repository Structure

```text
Manipulator-X/
├── GUI.mlapp               # Main Interactive App Designer GUI
├── main.m                  # Initialization & Global Constants
├── InitModel.m             # 3D Visualization Initialization
├── WorkSpace.m             # Workspace generation logic
├── Kinematics/
│   ├── ForwardKinematics.m # D-H Matrix Calculations
│   └── InverseKinematics.m # Geometric IK Solver
├── Trajectory/
│   ├── PathCubic.m         # Joint Space Polynomials
│   ├── TrajectoryCubic.m   # Cartesian Space Polynomials
│   ├── PathPlanning.m      # Joint Space Animation & Plotting
│   └── TrajectoryPlanning.m# Cartesian Animation & Plotting
└── README.md
```

## Getting Started

### Prerequisites

* MATLAB (R2018a or newer is recommended for App Designer compatibility).
* Ensure all `.m` and `.mlapp` files are in the MATLAB path.

### Running the Project

1.  **Clone the repository:**
    ```bash
    git clone [https://github.com/YourUsername/Manipulator-X.git](https://github.com/YourUsername/Manipulator-X.git)
    ```
2.  **Open MATLAB** and navigate to the project directory.
3.  **Launch the GUI:** Open the `GUI.mlapp` file and click **Run** within the App Designer environment.

### Core Simulation Files

| File | Function Summary |
| :--- | :--- |
| `main.m` | Initializes global robot constants (link lengths, joint limits) and calls the primary visualization routines. |
| `PathPlanning.m` | Implements **Joint Space** control, animating movement based on generated $\theta, v, a$ profiles. |
| `TrajectoryPlanning.m` | Implements **Cartesian Space** control, animating movement based on generated $p_x, p_y, p_z, v, a$ profiles and real-time IK calculation. |

## Potential Extensions and Improvements

* **Quintic Polynomial Trajectories:** Implement 5th-order polynomials to enforce constraints on acceleration (jerk) at the start and end points.
* **Collision Avoidance:** Integrate basic obstacle avoidance using proximity sensors or path adjustments.
* **Force Control:** Implement a compliance control loop for interaction with the environment.
* **Orientation Control:** Currently, the orientation is semi-fixed. Introduce Euler angle control for the wrist joint to follow a full 6-DOF path.

## Contribution

Contributions are welcome! Please feel free to open issues to report bugs or suggest features. For code contributions, submit a pull request against the `main` branch.

## License

This project is open-source and licensed under the MIT License.
