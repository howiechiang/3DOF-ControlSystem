%{
% @author Howard Chiang
%   
% This functions performs inverse kinematics on the modelled system. The 
% required inputs are the trajectory and desired position of the end 
% effector. The function will compare the current configuration with the
% with possible ones. 
%
% @param posX     = Desired final X position
% @param posY     = Desired final Y position
% @param theta1C  = Current theta1 value
% @param theta2C  = Current theta2 value
%
% @return theta1F = Final theta1 value
% @return theta2F = Final theta2 value
% @return theta3F = Final theta3 value
%}

function [theta1F, theta2F, theta3F] = InverseKinematics(posX, posY, theta1C, theta2C)
    L1 = 10;
    L2 = 10;
    
    beta  =  atan2(posY,posX);
    gamma =  acos((posX^2 + posY^2 + L1^2 - L2^2) / (2*L1*sqrt(posX^2+posY^2)));

    % There are two different solutions for theta1 & theta2
    sol1A = beta - gamma;
    sol1B = beta + gamma;
    sol2A =  acos((posX^2 + posY^2 - L1^2 - L2^2) / (2*L1*L2));
    sol2B = -acos((posX^2 + posY^2 - L1^2 - L2^2) / (2*L1*L2));

    % Since there are two available solutions, the configuration most
    % similar to the current one is desired. If both solutions yield the
    % same error, the second solutions will be chosen.
    errorA = abs(theta1C - sol1A) + abs(theta2C - sol2A); 
    errorB = abs(theta1C - sol1B) + abs(theta2C - sol2B); 

    if errorA < errorB
        theta1F = double(sol1A);
        theta2F = double(sol2A);
        theta3F = double(-(sol1A + sol2A));
    else
        theta1F = double(sol1B);
        theta2F = double(sol2B);
        theta3F = double(-(sol1B + sol2B));
    end
end