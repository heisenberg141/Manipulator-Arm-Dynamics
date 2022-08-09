% RBE 501 - Robot Dynamics - Spring 2022
% Final Exam
% Worcester Polytechnic Institute
%
% Instructor: L. Fichera <lfichera@wpi.edu>
% Last modified: 05/02/2021
clear, clc, close all
addpath('utils');

%% Create the environment
g = [0 0 -9.81]; % Gravity Vector [m/s^2]

%% Create the dynamics model of the robot
[S,M] = make_kinematics_model();
[Mlist, Glist] = make_dynamics_model();

n = 6; % degrees of freedom
L1 = 0.4; % Lenght of Link 1 [m]
L2 = 0.2; % Lenght of Link 2 [m]
L3 = 0.2; % Lenght of Link 3 [m]
L4 = 0.17; % Lenght of Link 4 [m]
L5 = 0.17; % Lenght of Link 5 [m]
L6 = 0.126; % Lenght of Link 6 [m]

robot = SerialLink([Revolute('d', L1,     'a', 0,  'alpha', pi/2), ...
    Revolute('d', 0,      'a', 0, 'alpha', pi/2,     'offset', pi/2), ...
    Revolute('d', L2+L3,      'a', 0,  'alpha', -pi/2), ...
    Revolute('d', 0,  'a', 0,  'alpha', -pi/2, 'offset', -pi/2), ...
    Revolute('d', L4+L5,      'a', 0,  'alpha', pi/2), ...
    Revolute('d', 0, 'a', L6,  'alpha', 0)], ...
    'name', 'Multi_link_robot');

%% Define the path we wish to trace, and solve the IK
nPts = 100;
x = 0.6 * ones(1,nPts);
t = linspace(0, pi*2, nPts);
y = 0.4*cos(t)./(1+sin(t).*sin(t));
z = 0.4*sin(t).*cos(t)./(1+sin(t).*sin(t))+0.3;
path = [x; y; z];
fprintf('Done.\n');

% Initialize the matrix to store the IK result
targetQ = zeros(n,nPts);
scatter3(path(1,:), path(2,:), path(3,:), 'filled');
% Iterate over the target points
for ii = 1 : nPts
    if ii == 1
        % Set the initial joint variables to zero
        currentQ = zeros(n,1);
    else
        % Read the latest set of joint variables
        currentQ = targetQ(:,ii-1);
    end

    targetQ(:,ii) = ikin(S,M,currentQ,path(:,ii));
end

%% *** YOUR CODE HERE ***

% Now, for each pair of consecutive set points, we will first calculate a
% trajectory between these two points, and then calculate the torque
% profile necessary to move from one point to the next.
fprintf('Generate the Joint Torque Profiles... ');

% Initialize the arrays where we will accumulate the output of the robot
% dynamics, so that we can display it later
qtt = []; % Joint Variables
tau = [];

for jj = 1 : nPts - 1
    t0 = 0; tf = 0.5; % Starting and ending time of each trajectory
    N = 30;          % Number of intermediate setpoints
    t = linspace(t0, tf, N); % time vector

    q = zeros(n,N);   % joint variables
    qd = zeros(n,N);  % joint velocities
    qdd = zeros(n,N); % joint accelerations

    for ii = 1 : n
        % Calculate the coefficients of the quintic polynomial
        a = quinticpoly(t0, tf, ...
            targetQ(ii,jj), targetQ(ii,jj+1), ...
            0, 0, 0, 0);

        % Generate the joint profiles (position, velocity, and
        % acceleration)
        q(ii,:) = a(1) + a(2) * t + a(3) * t.^2 + a(4) * t.^3 + a(5) * t.^4 + a(6) * t.^5;
        qd(ii,:) = a(2) + 2*a(3)*t + 3*a(4)*t.^2 + 4*a(5)*t.^3 + 5*a(6)*t.^4;
        qdd(ii,:) = 2*a(3) + 6*a(4)*t + 12*a(5)*t.^2 + 20*a(6)*t.^3;
    end

    % Use the equations of motion to calculate the necessary torques to trace
    % the trajectory
    Ftipmat = zeros(N,6); % no end effector force
    
    taumat = InverseDynamicsTrajectory(q', qd', qdd', ...
        g, Ftipmat, Mlist, Glist, S);

    % Use the Forward Dynamics to simulate the robot behavior
    dt = tf/N;  % time step
    intRes = 1; % Euler integration constant
    [qt, qdt] = ForwardDynamicsTrajectory(q(:,1), qd(:,1), taumat, g, ...
        Ftipmat, Mlist, Glist, S, dt, ...
        intRes);

    qtt = [qtt; qt]; % Accumulate the results
    tau = [tau; taumat];
end

fprintf('Done.\n');

fprintf('Simulate the robot...');
title('Inverse Dynamics Control');
robot.plot(qtt(1:10:end,:));
fprintf('Done.\n');

% Display the Joint Torques
figure, hold on, grid on
plot((1:length(tau))*dt, tau(:,1), 'Linewidth', 2);
plot((1:length(tau))*dt, tau(:,2), 'Linewidth', 2);
plot((1:length(tau))*dt, tau(:,3), 'Linewidth', 2);
plot((1:length(tau))*dt, tau(:,4), 'Linewidth', 2);
plot((1:length(tau))*dt, tau(:,5), 'Linewidth', 2);
plot((1:length(tau))*dt, tau(:,6), 'Linewidth', 2);
xlim([0 max((1:length(tau))*dt)]);
xlabel('Time [s]'), ylabel('Torque [Nm]');
legend({'Joint 1', 'Joint 2', 'Joint 3','Joint 4','Joint 5','Joint 6'});
set(gca, 'FontSize', 14);

fprintf('Program completed successfully.\n');


