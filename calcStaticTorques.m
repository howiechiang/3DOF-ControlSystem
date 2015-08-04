%{ 
% @author Howard Chiang
% 
% This function accounts for the static forces on the end effector of the 
% arm, and determines the reacting forces & torques that propagates through 
% the entire arm. 
%
% As the forces travel through the arm and down to the base, these forces
% will create a torque at each connecting joint of the arm. If one
% was to imagine this process backwards, it can be seen as the torques
% required by each joint to deliver the requested force to the end
% effector, to balance the inverted pendulum.  
%
% @param t0 = current theta0
% @param t1 = current theta1
% @param t2 = current theta2
% @param t3 = current theta3
% @param Fx = reacting force in the x-direction
% @param Fy = reacting force in the y-direction
% @param Fz = reacting force in the z-direction
%   
%
% @return T00 = required torque by motor 0
% @return T11 = required torque by motor 1
% @return T22 = required torque by motor 2
% @return T33 = required torque by motor 3
%}


function [T00, T11, T22, T33] = calcStaticTorques(theta0, theta1, theta2, theta3, Fx, Fy, Fz)
    global L0 L1 L2 L3
    z = [0 ; 0 ; 1]; % Rotation about the z-axis

    % These are the basic rotation matrices of the robotic arm
    R01 = [cos(theta0) 0 sin(theta0) ; sin(theta0) 0 -cos(theta0) ; 0 1 0];
    R12 = [cos(theta1) -sin(theta1) 0 ; sin(theta1) cos(theta1) 0 ; 0 0 1];
    R23 = [cos(theta2) -sin(theta2) 0 ; sin(theta2) cos(theta2) 0 ; 0 0 1];
    R34 = [cos(theta3) -sin(theta3) 0 ; sin(theta3) cos(theta3) 0 ; 0 0 1];
    R04 = R01*R12*R23*R34; 



    % These are the basic vectors that describe the dimensions of the robot, 
    P01 = [0 ; 0; L0];
    P12 = [L1 ; 0 ; 0];
    P23 = [L2 ; 0 ; 0];
    P34 = [L3 ; 0 ; 0];


    % F44 is the reacting force required by the arm to balance the balance the
    % inverted pendulum. This section calculates how the force will propagate
    % through the entire arm, down to the base. 
    F44 = [Fx; Fy; Fz];
    F33 = transpose(R34)*F44;
    F22 = transpose(R23)*F33;
    F11 = transpose(R12)*F22;
    F00 = transpose(R01)*F11;

    % This section calculates the reacting torques matrix at each joint of 
    % the arm. It then simplifies the the torque matrix into a single
    % net torque required by the motors. 
    n33 = cross(P34, F33);
    n22 = R23*n33 + cross(P23, F22);
    n11 = R12*n22 + cross(P12, F11);
    n00 = R01*n11 + cross(P01, F00);

    T33 = transpose(n33)*z;
    T22 = transpose(n22)*z;
    T11 = transpose(n11)*z;
    T00 = transpose(n00)*z;
end


