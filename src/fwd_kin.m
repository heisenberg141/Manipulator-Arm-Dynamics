addpath('utils');

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
    'name', 'multi_link');

% Display the manipulator in the home configuration
qz = zeros(1,6);
robot.teach(qz, 'jaxes');
hold on;
%% Create the kinematics model of the robot
[S,M] = make_kinematics_model();

%% Forward Kinematics Test
fprintf('---------------------Forward Kinematics Test---------------------\n');
nTests = 20;
fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
fprintf('Progress: ');
nbytes = fprintf('0%%');

for ii = 1 : nTests
    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('%0.f%%', ceil(ii/nTests*100));

    % Generate a random configuration
    q = rand(1,6) * 2 * pi;

    % Calculate the forward kinematics
    T = fkine(S,M,q,'space');
    
    assert(all(all(abs(double(robot.fkine(q)) - T) < 1e-4)));
end
fprintf('\nTest passed successfully.\n');
