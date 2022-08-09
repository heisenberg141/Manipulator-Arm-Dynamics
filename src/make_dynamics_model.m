function [Mlist,Glist] = make_dynamics_model()
% MAKE_KINEMATICS_MODEL Creates the dynamics model of the robot
%
% Inputs: None
%
% Output: Mlist - 4x4x7 matrix containing all the transformation matrices between consecutive link frames
%         Glist - 6x6x6 matrix containing the spatial inertia matrices of each link


%% Define the link lengths and masses
L1 = 0.4; % Lenght of Link 1 [m]
L2 = 0.2; % Lenght of Link 2 [m]
L3 = 0.2; % Lenght of Link 3 [m]
L4 = 0.17; % Lenght of Link 4 [m]
L5 = 0.17; % Lenght of Link 5 [m]
L6 = 0.126; % Lenght of Link 6 [m]

m1 = 3;   % Mass of Link 1 [kg]
m2 = 0.5;   % Mass of Link 2 [kg]
m3 = 0.5;   % Mass of Link 3 [kg]
m4 = 0.5;   % Mass of Link 4 [kg]
m5 = 0.5;   % Mass of Link 5 [kg]
m6 = 0.4;   % Mass of Link 6 [kg]

r = 2.5e-2; % Link radius [m]

m_vec = [m1 m2 m3 m4 m5 m6];
Ixx = (3*r^2 +[L1^2 L2^2 L3^2 L4^2 L5^2 L6^2])./12.*m_vec;
Iyy = Ixx;
Izz = m_vec.*r^2/2;

Ib1 = [Ixx(1) 0 0
        0 Iyy(1) 0
        0 0 Izz(1)].*m1;
Ib2 = [Ixx(2) 0 0
        0 Iyy(2) 0
        0 0 Izz(2)].*m2;
Ib3 = [Ixx(3) 0 0
        0 Iyy(3) 0
        0 0 Izz(3)].*m3;
Ib4 = [Ixx(4) 0 0
        0 Iyy(4) 0
        0 0 Izz(4)].*m4;
Ib5 = [Ixx(5) 0 0
        0 Iyy(5) 0
        0 0 Izz(5)].*m5;
Ib6 = [Ixx(6) 0 0
        0 Iyy(6) 0
        0 0 Izz(6)].*m6;

G1 = [Ib1 zeros(3,3)
      zeros(3,3) eye(3,3).*m1];% Spatial Inertia Matrix for link 1

G2 = [Ib2 zeros(3,3)
      zeros(3,3) eye(3,3).*m2]; %Spatial Inertia Matrix for link 2

G3 = [Ib3 zeros(3,3)
      zeros(3,3) eye(3,3).*m3]; %Spatial Inertia Matrix for link 3

G4 = [Ib4 zeros(3,3)
      zeros(3,3) eye(3,3).*m4];% Spatial Inertia Matrix for link 1

G5 = [Ib5 zeros(3,3)
      zeros(3,3) eye(3,3).*m5]; %Spatial Inertia Matrix for link 2

G6 = [Ib6 zeros(3,3)
      zeros(3,3) eye(3,3).*m6]; %Spatial Inertia Matrix for link 3

Glist = cat(3, G1, G2, G3, G4, G5, G6);

M01 =[eye(3),[0,0,L1/2]'
      0 0 0 1];

M12 = [0 0 1 L2/2
       0 1 0 0
       -1 0 0 L1/2
       0 0 0 1];

M23 =[eye(3),[0,0,L2/2+L3/2]'
      0 0 0 1];

M34 = [0 0 -1 -L4/2
       0 1 0 0
       1 0 0 L3/2
       0 0 0 1];

M45 = [eye(3), [0 0 L4/2+L5/2]'
        0 0 0 1];

M56 =[0 0 1 L6/2
      0 1 0 0
     -1 0 0 L5/2
      0 0 0 1];

M67 = [0 -1 0 0
       0 0 -1 0
       1 0 0 L6/2
       0 0 0 1];

Mlist = cat(3, M01, M12, M23, M34, M45, M56, M67);


end

