%{
% @author Howard Chiang
%   
% This function calculates the Jacobian matrix of the end effector from the
% perspective of frame {0}. 
% 
% @param t0 = current theta0
% @param t1 = current theta1
% @param t2 = current theta2
% @param t3 = current theta3
%
% @return b = Jacobian of frame {0} to {4} (J04)
%}

function b = calcJacobian(theta0, theta1, theta2, theta3)

    d_theta0 = 0; % This is assumed to be true.
    d_theta1 = sym('d_theta1');
    d_theta2 = sym('d_theta2');
    d_theta3 = sym('d_theta3');

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

    % This establishes the equations for angular velocity of each joint and 
    % describes how it propagates through the arm. 
    w00 = [0 ; 0 ; d_theta0];
    w11 = transpose(R01)*w00 + d_theta1*z;
    w22 = transpose(R12)*w11 + d_theta2*z;
    w33 = transpose(R23)*w22 + d_theta3*z;
    w44 = transpose(R34)*w33;

    % This establishes the equations for the velocity of each joint based on 
    % the angular velocities and dimensions of the arm. Lastly, the velocity of
    % the end effector is multiplied by the rotation matrix(base to end effector)
    % to express in the velocity relative to the global axis.
    v00 = [0 ; 0 ; 0];
    v11 = transpose(R01)*v00 + transpose(R01)*cross(w00,P01);
    v22 = transpose(R12)*v11 + transpose(R12)*cross(w11,P12);
    v33 = transpose(R23)*v22 + transpose(R23)*cross(w22,P23);
    v44 = transpose(R34)*v33 + transpose(R34)*cross(w33,P34);
    v04 = simplify(R04*v44);


    % This generates the Jacobian Matrix using the V04 and coeff() function. 
    % The coeff() function is used to separate the angular velocity components 
    % in each row of v04 so it can be placed in the proper location of the 
    % Jacobian matrix. 
    % 
    % NOTE: The expected code for row2_J will generate the a 'empty sym', which
    % cannot be used to form the Jacobian matrix. This is a problem that occurs
    % when the first argument of coeff() is 0. Therefore the values of the
    % second row of the Jacobian will be set to 0 automatically. 

    J_11 = coeffs(v04(1), d_theta1);
    J_12 = coeffs(v04(1), d_theta2);
    J_13 = coeffs(v04(1), d_theta3);

    J_21 = 0;    %J21 = coeffs(v04(2), d_theta1);
    J_22 = 0;    %J22 = coeffs(v04(2), d_theta2);
    J_23 = 0;    %J23 = coeffs(v04(2), d_theta3);

    J_31 = coeffs(v04(3), d_theta1);
    J_32 = coeffs(v04(3), d_theta2);
    J_33 = coeffs(v04(3), d_theta3);

    % Note the coefficients yields two outputs. The second output is the desired
    % value of the coefficient for the called equation. 
    b = double([J_11(1,2) J_12(1,2) J_13(1,2); J_21 J_22 J_23; J_31(1,2) J_32(1,2) J_33(1,2)]);
end