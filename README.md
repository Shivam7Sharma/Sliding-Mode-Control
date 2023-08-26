# RBE502_Final_Project
This project uses sliding mode control for controlling a drone to follow a trajectory


RBE 502 - Robot Control Final Project

The objective of this project is to develop a robust control scheme to enable a quadrotor to track
desired trajectories in the presence of external disturbances.

Part 1: Trajectory Generation

Consider a quintic trajectory of the form
qd(t) = a0 + a1*t+ a2*t2 + a3*t3 + a4*t4 + a5*t5
The coefficients a0, a1, a2, a3, a4 and a5 can be found by solving the matrix equation
below:



Here a0, a1, a2, a3, a3, a4 and a5 are unknowns and has values:

• p0 = (0, 0, 0) to p1 = (0, 0, 1) in 5 seconds
• p1 = (0, 0, 1) to p2 = (1, 0, 1) in 15 seconds
• p2 = (1, 0, 1) to p3 = (1, 1, 1) in 15 seconds
• p3 = (1, 1, 1) to p4 = (0, 1, 1) in 15 seconds
• p4 = (0, 1, 1) to p5 = (0, 0, 1) in 15 seconds

Differentiating qd(t) wrt. time we get velocity as:
qd`(t) = a1+ 2*a2*t + 3*a3*t2+ 4*a4*t3 + 5*a5*t4


And again differentiating velocity wrt. time we get acceleration as:
qd``(t) = 2*a2* + 6*a3*t + 12*a4*t2 + 20*a5*t3

Desired trajectory plot is as follows:














Part 2: Controller Design
Control Law derived in handwritten notes are as follows:





Tuned Parameters:

a) The PD controller utilizes Kp and Kd parameters to determine the speed at which the system approaches the trajectory. Increasing these parameters will result in faster convergence. In this case, Kp is set to 110 and Kd to 8.
b) The lambda parameters are employed in the sliding mode control to determine the speed at which the system transitions from the initial state to the designated sliding surface. As with the previous case, larger values will result in faster convergence. The values assigned to the lambd_z, lambd_phi, lambd_theta, and lambd_psi parameters are 12, 13, 19, and 5, respectively.
c) The gain parameters of the sliding mode control determine the speed at which the system descends down the sliding surface. Larger values result in faster convergence towards the desired trajectory. The k_z, k_phi, k_theta, and k_psi parameters are assigned values of 6, 140, 111, and 25, respectively.
d) The saturation function constant is used to prevent chattering in sliding mode control, and is used to determine the boundary region around the desired trajectory. A smaller value will place the boundary closer to the desired trajectory. If chattering persists despite the implementation of the saturation function, the constant value should be increased. In this case, a value of 1.3,1,1,and 1 is assigned to the constant a, a_phi, a_theta, a_psi.


Part 3: Code Explanation

Code Explanation:


Upon receiving a call to odom_callback, the time (self.t) is initialized and relevant information such as the drone's current position, velocity, orientation, and angular velocity along the 3 axes are extracted from the odometry message. These values are then fed into the smc_control function. smc_control function first invokes the traj_evaluate function which predicts the drone's trajectory and invokes the generate_trajectory to obtain the desired position, velocity, and acceleration for the x,y,z axis at that instant. Based on the derivation in part 1, the desired position, velocity, and acceleration are calculated and returned to smc_control by traj_evaluate. smc_control then determines the desired roll, pitch, and yaw angles and defines errors in altitude z, roll, pitch, and yaw angles. The sliding surface is defined using these errors, and then the values of u1, u2, u3, and u4 are calculated based on the calculations in part 2. The motor_speed function takes these values as input and returns the motor angular velocities based on the allocation matrix. These angular velocities are then clamped between motor min and max speeds, and finally, they are published to the motor_speed topic to actuate the drone.






Part 4 Trajectory 3D and analysis:


Drone’s trajectory in 3D space is shown as follows:



The 3D plot reveals that the drone's trajectory slightly deviates from the actual path, which can be attributed to the saturation function implemented to prevent chattering. Therefore, it can be concluded that the controller functions effectively and possesses the ability to handle external disturbances reasonably well, while still being able to closely track the desired trajectory.


