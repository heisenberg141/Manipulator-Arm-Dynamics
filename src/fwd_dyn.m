clear, clc, close all
addpath('utils');

%% Create the environment
g = [0 0 -9.81]; % Gravity Vector [m/s^2]

%% Create the dynamics model of the robot
[S,M] = make_kinematics_model();
[Mlist, Glist] = make_dynamics_model();

%% YOUR CODE HERE
q0 = zeros(6,1);  % initial configuration
qd0 = zeros(6,1); % initial velocities

% Invoke the `GravityForces` function. This function calculates the inverse
% dynamics of the robot when the robot is not moving and the only force
% acting on it is gravity.

grav = GravityForces(q0, g, Mlist, Glist, S);

fprintf('Joint Torques: ');
fprintf('[%f %f %f %f %f %f] Nm\n', grav(1), grav(2), grav(3), grav(4), grav(5), grav(6));

