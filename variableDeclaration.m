%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Variable Declaration
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%{
% @author Howard Chiang
%   
% [0] = Distance of robot arm base from first linkage
% [1] = Length of bicep
% [2] = Length of forearm
% [3] = Length of hand
%
%}

global L1 L2 L3 L0 m1 m2 m3 m0 g;

% Acceleration due to gravity
g = 9.81;

% Length of each frame
L1 = 10;
L2 = 10;
L3 = 5;
L0 = 2;

% Mass of frames
m1 = 1;
m2 = 1;
m3 = 0.5;
m0 = 0.1;

% Moment of Inertia about each Joint
I1 = m1 * L1^2 /12;
I2 = m2 * L2^2 /12;
I3 = m3 * L3^2 /12;
I0 = m0 * L0^2 /12;

