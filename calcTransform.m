%{
% @author Howard Chiang
%   
% This function determines the orientation and position based on the
% current values of theta on the robotic arm
% 
% @param t1 = current theta1
% @param t2 = current theta2
% @param t3 = current theta3
%
% @return b = 4x4 matrices of the end effector transform
%}
function [posXC, posYC] = calcTransform(theta1, theta2, theta3)
   global L1 L2 L3;

    T12 = [cos(theta1) -sin(theta1) 0 0; sin(theta1) cos(theta1) 0 0; 0 0 1 0; 0 0 0 1];
    T23 = [cos(theta2) -sin(theta2) 0 L1; sin(theta2) cos(theta2) 0 0; 0 0 1 0; 0 0 0 1];
    T34 = [cos(theta3) -sin(theta3) 0 L2; sin(theta3) cos(theta3) 0 0; 0 0 1 0; 0 0 0 1];
    T45 = [1 0 0 L3/2; 0 1 0 0; 0 0 1 0; 0 0 0 1];
    T15 = T12 * T23 * T34 * T45;
    posXC = T15(1,4);
    posYC = T15(2,4);
end