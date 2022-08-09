
clear, clc, close all
addpath('utils');
plotOn = false;

%% Create the manipulator
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
    'name', 'multi_link_robot');

%% Inverse Kinematics
% Define the path that we wish the robot to trace
fprintf('Generating task space path... ');
nPts = 100;
x = 0.6 * ones(1,nPts);
t = linspace(0, pi*2, nPts);
y = 0.4*cos(t)./(1+sin(t).*sin(t));
z = 0.4*sin(t).*cos(t)./(1+sin(t).*sin(t))+0.3;
path = [x; y; z];
fprintf('Done.\n');

% Display the path
figure
robot.plot(zeros(1,6), 'jaxes'), hold on;
scatter3(path(1,:), path(2,:), path(3,:), 'filled');
title('Inverse Kinematics')

%% *** YOUR CODE HERE ***
fprintf('Calculating the Inverse Kinematics... ');

% Create the kinematics model of the robot
[S,M] = make_kinematics_model();

% Initialize the matrix to store the IK result
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
%     fprintf('Done!');
end

robot.plot(targetQ','trail',{'r', 'LineWidth', 2});

fprintf('Done.\n');
close all;
