%{
% @author Howard Chiang
% 
% This utilizes Newton and Euler's equations to derive the mannipulator
% dynamics of the 3DOF robotic arm system. This accounts for all mechanical
% aspects; inertial, gravitational, coriolis, and centrifugial forces about
% the center of mass of each frame. The calculations are shifted from each
% joint to the center of mass by method of the parallel axis theorem. 
% This does stuff.
% 
%
% @param theta1    = Joint 1 [position]
% @param theta1_d  = Joint 1 [velocity]
% @param theta1_dd = Joint 1 [acceleration]
%
% @param theta2    = Joint 2 [position]
% @param theta2_d  = Joint 2 [velocity]
% @param theta2_dd = Joint 2 [acceleration]
%
% @param theta3    = Joint 3 [position]
% @param theta3_d  = Joint 3 [velocity]
% @param theta3_dd = Joint 3 [acceleration]
% 
% @param vel1      = Joint 1 [Current Velocity?]
% @param accel1    = Joint 1 [Current Acceleration?]
%
% @param vel2      = Joint 1 [Current Velocity?]
% @param accel2    = Joint 1 [Current Acceleration?]
%
% @param vel3      = Joint 1 [Current Velocity?]
% @param accel3    = Joint 1 [Current Acceleration?]
%}

function [torque1, torque2, torque3] = calcManipulatorDynamicsRev2(theta1, theta1_d, theta1_dd, theta2,  theta2_d,  theta2_dd, theta3,  theta3_d,  theta3_dd, theta0, theta0_d, theta0_dd, vel1, accel1, vel2, accel2, vel3, accel3)
    global L1 L2 L3 L0 m1 m2 m3 m0 g;

    % Rotation Matrices
    r12 = [cos(theta1) -sin(theta1) 0; sin(theta1) cos(theta1) 0; 0 0 1];
    r23 = [cos(theta2) -sin(theta2) 0; sin(theta2) cos(theta2) 0; 0 0 1];
    r34 = [cos(theta3) -sin(theta3) 0; sin(theta3) cos(theta3) 0; 0 0 1];
    r45 = [cos(theta0) -sin(theta0) 0; sin(theta0) cos(theta0) 0; 0 0 1];

    % A third rotation matrix is used to translate the counter force 
    % induced by the cart during balancing. The force will always be in the 
    % x-direction in the Cartesian, and r23 will translate it to so it can 
    % be expressed with respect to the frame of joint 2. This theta3 is the
    % angle required to return the end effector back to its original 
    % orientation. 
    theta3 = theta1;
    r23 = [cos(theta3) -sin(theta3) 0; sin(theta3) cos(theta3) 0; 0 0 1];

    % Displacement due to each Joint
    P12 = [L1; 0; 0];
    P23 = [L2; 0; 0];
    P34 = [L3/2; 0; 0];

    % Distance to the Center of Mass
    P1C = [L1/2; 0; 0];
    P2C = [L2/2; 0; 0];
    P3C = [L3/2; 0; 0];
    P4C = [0; L0/2; 0];

    % Rotation only about the Z-axis
    Z = [0; 0; 1];

    % There is no v00, but will be used to apply gravity to the system
    v00 = [0; g; 0];

    % Inertia Tensor of the system
    IT1_C = m1 * L1^2 / 12;
    IT2_C = m2 * L2^2 / 12;
    IT3_C = m3 * L3^2 / 12;
    IT4_C = m0 * L0^2 / 12;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Outward Iteration
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    w11 = (theta1_d + vel1) * Z;
    w11_d = (theta1_dd + accel1) * Z;
    w22 = transpose(r12) * w11 + (theta2_d + vel2) * Z;
    w22_d = transpose(r12) * w11_d + cross(transpose(r12) * w11, theta2_d * Z) + (theta2_dd + accel2) * Z;
    w33 = transpose(r23) * w22 + (theta3_d + vel3) * Z;
    w33_d = transpose(r23) * w22_d + cross(transpose(r23) * w22, theta3_d * Z) + (theta3_dd + accel3) * Z;
    w44 = transpose(r34) * w33 + theta0_d * Z;
    w44_d = transpose(r34) * w33_d + cross(transpose(r34) * w33, theta0_d * Z) + theta0_dd * Z;

    % This utilizes the parallel axis theorem to derive the the velocity
    % and acceleration at the center of mass. 
    v11_d = transpose(r12) * v00;
    v1C1_d = cross(w11_d, P1C) + cross(w11, cross(w11, P1C)) + v11_d;
    v22_d = transpose(r12) * (cross(w11_d, P12) + cross(w11, cross(w11, P12)) + v11_d);
    v2C2_d = cross(w22_d, P2C) + cross(w22, cross(w22, P2C)) + v22_d;
    v33_d = transpose(r23) * (cross(w22_d, P23) + cross(w22, cross(w22, P23)) + v22_d);
    v3C3_d = cross(w33_d, P3C) + cross(w33, cross(w33, P3C)) + v33_d;
    v44_d = transpose(r34) * (cross(w33_d, P34) + cross(w33, cross(w33, P34)) + v33_d);
    v4C4_d = cross(w44_d, P4C) + cross(w44, cross(w44, P4C)) + v44_d;
    
    % It is desired that all forces acting on F44 (the inverted pendulum)
    % be zero. So therefore it should be zero. Theta3 can be used to
    % balance the disturbance. 
    F11 = m1 * v1C1_d;
    F22 = m2 * v2C2_d;
    F33 = m3 * v3C3_d;
    F44 = m0 * v4C4_d;

    N11 = IT1_C * w11_d + cross(w11, (IT1_C * w11));
    N22 = IT2_C * w22_d + cross(w22, (IT2_C * w22));
    N33 = IT3_C * w33_d + cross(w33, (IT3_C * w33));
    N44 = IT4_C * w44_d + cross(w44, (IT4_C * w44));
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Inward Iteration
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % f33 is the force applied to the linkage at the end of the 2nd joint. 
    % This is included as the counter force required to balance the 
    % inverted pendulum. In f22, the force is rotated by r23 so the force 
    % is applied with respect to the frame of the 2nd link. 
    
    f44 = F44;
    f33 = r34 * f44 + F33;
    f22 = r23 * f33 + F22;
    f11 = r12 * f22 + F11;
    
    n44 = N44 + cross(P4C, F44);
    n33 = N33 + r34 * n44 + cross(P3C, F33) + cross(P34, (r34 * f44));
    n22 = N22 + r23 * n33 + cross(P2C, F22) + cross(P23, (r23 * f33));
    n11 = N11 + r12 * n22 + cross(P1C, F11) + cross(P12, (r12 * f22));

    torque1 = simplify((transpose(n11) * Z));
    torque2 = simplify((transpose(n22)* Z));
    torque3 = simplify((transpose(n33) * Z));
end


