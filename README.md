# RBE502_Final_Project

## Sliding Mode Control for Drone Trajectory Following

![Project Banner](./videos/ezgif.com-video-to-gif2.gif)

### Objective

The objective of this project is to develop a robust control scheme to enable a quadrotor to track desired trajectories in the presence of external disturbances.

---

## Part 1: Trajectory Generation

### Quintic Trajectory

Consider a quintic trajectory of the form \( q_d(t) = a_0 + a_1t + a_2t^2 + a_3t^3 + a_4t^4 + a_5t^5 \).

#### Coefficients

The coefficients \( a_0, a_1, a_2, a_3, a_4 \) and \( a_5 \) can be found by solving the matrix equation below:

![Matrix Equation](./videos/Matrix.png.png)


#### Velocity and Acceleration

Differentiating \( q<sub>d</sub>(t) \) with respect to time, we get:

- Velocity: \( q'<sub>d</sub>(t) = a<sub>1</sub> + 2a<sub>2</sub>t + 3a<sub>3</sub>t<sup>2</sup> + 4a<sub>4</sub>t<sup>3</sup> + 5a<sub>5</sub>t<sup>4</sup> \)
- Acceleration: \( q''<sub>d</sub>(t) = 2a<sub>2</sub> + 6a<sub>3</sub>t + 12a<sub>4</sub>t<sup>2</sup> + 20a<sub>5</sub>t<sup>3</sup> \)

#### Desired Trajectory Plot

![Desired Trajectory](./videos/demo_trajectory_plot.png)

---

## Part 2: Controller Design

### Control Law

Control laws derived in handwritten notes are as follows:

#### Tuned Parameters

- **PD Controller**: \( K<sub>p</sub> = 110 \), \( K<sub>d</sub> = 8 \)
- **Lambda Parameters**: \( &lambda;<sub>z</sub> = 12 \), \( &lambda;<sub>&phi;</sub> = 13 \), \( &lambda;<sub>&theta;</sub> = 19 \), \( &lambda;<sub>&psi;</sub> = 5 \)
- **Gain Parameters**: \( k<sub>z</sub> = 6 \), \( k<sub>&phi;</sub> = 140 \), \( k<sub>&theta;</sub> = 111 \), \( k<sub>&psi;</sub> = 25 \)

---
## Part 3: Code Explanation

### Odom Callback and SMC Control

Upon receiving a call to `odom_callback`, the time (`self.t`) is initialized and relevant information such as the drone's current position, velocity, orientation, and angular velocity along the 3 axes are extracted from the odometry message. These values are then fed into the `smc_control` function.

#### SMC Control Function

`smc_control` function first invokes the `traj_evaluate` function which predicts the drone's trajectory and invokes the `generate_trajectory` to obtain the desired position, velocity, and acceleration for the x, y, z axis at that instant.

---

## Part 4: Trajectory 3D and Analysis

### 3D Trajectory

![3D Trajectory](./videos/3d-trajectory.gif)

### Analysis

The 3D plot reveals that the drone's trajectory slightly deviates from the actual path, which can be attributed to the saturation function implemented to prevent chattering. Therefore, it can be concluded that the controller functions effectively and possesses the ability to handle external disturbances reasonably well, while still being able to closely track the desired trajectory.

---

## Getting Started

To get started with this project, clone the repository:

```bash
git clone https://github.com/Shivam7Sharma/RBE502_Final_Project.git
