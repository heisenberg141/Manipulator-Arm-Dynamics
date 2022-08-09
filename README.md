# Dynamic-Modelling-of-Manipulator-Arm
Inverse dynamics control using dynamic modelling for multi link serial manipulator arm.

The code present in the src folder organised in the following manner:
1. ikin.m contains the code necessary to converge the inverse kinematics to the desired point.
2. fwd_kin.m contains the runner code to find the forward kinematics of the robot.
3. make_kinematics.m contains the code to generate forward kinematics for the robot
4. make_dynamics_model.m contains the code for generating G_List an M_List matrices for robot dynamics.
5. fwd_dyn.m contains the code for forward dynamics and gravity compensation. It generates joint torques for holding the robot stable.
6. inv_dynamics.m contains the code for inverse dynamics in newton euler representation. It generates the Torque plots for getting traversing the trajectory.

# Results

![illustration](src/dyn_gif.gif)
![illustration](joint_values.png)

