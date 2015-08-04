%{
% @author Howard Chiang
%   
% This calculates the trajectory of the arm using cubic polynomials;
% position, velocity, and acceleration.  
% 
% @param theta1C = current theta1
% @param theta2C = current theta2
% @param theta3C = current theta3
% @param theta1F = Final theta1 
% @param theta2F = Final theta2 
% @param theta3F = Final theta3 
%
% @return a0     = Joint 1 [zero polynomial]
% @return a1     = Joint 1 [first polynomial]
% @return a2     = Joint 1 [second polynomial]
% @return a3     = Joint 1 [third polynomial]
% @return b0     = Joint 2 [zero polynomial]
% @return b1     = Joint 2 [first polynomial]
% @return b2     = Joint 2 [second polynomial]
% @return b3     = Joint 2 [third polynomial]
% @return c0     = Joint 3 [zero polynomial]
% @return c1     = Joint 3 [first polynomial]
% @return c2     = Joint 3 [second polynomial]
% @return c3     = Joint 3 [third polynomial]
%}

function [a0, a1, a2, a3, b0, b1, b2, b3, c0, c1, c2, c3] = calcTrajectoryRev2(theta1C, theta2C, theta3C, theta1F, theta2F, theta3F)
    
    % Coefficients to be found
    syms a2;
    syms a3;
    syms b2;
    syms b3;
    syms c2;
    syms c3;
    
    % This sets the given amount of time to reach the target trajectory. 
    tF = 5;
    
    % This initializes the known theta values to the coefficients of each
    % cubic polynomial. Since the system is be at rest at (t=0), the
    % coefficients of all first polynonmials is 0.
    a0 = theta1C;
    b0 = theta2C;
    c0 = theta3C;

    a1 = 0;
    b1 = 0;
    c1 = 0;
    
    % Full List of Cubic Polynomial Equations
    %{
    % Initial position, velocity, acceleration for joint 1
    position1I = a0 + a1 * t0 + a2 * t0^2 + a3 * t0^3;
    velocity1I = a1 + 2 * a2 * t0 + 3 * a3 * t0^2;
    acceleration1I = 2 * a2 + 6 * a3 * t0;
    
    % Final position, velocity, accleration for joint 1
    position1F = a0 + a1 * tF + a2 * tF^2 + a3 * tF^3;
    velocity1F = a1 + 2 * a2 * tF + 3 * a3 * tF^2;
    acceleration1F = 2 * a2 + 6 * a3 * tF;
    
    % Initial position, velocity, acceleration for joint 2
    position2I = b0 + b1 * t0 + b2 * t0^2 + b3 * t0^3;
    velocity2I = b1 + 2 * b2 * t0 + 3 * b3 * t0^2;
    acceleration2I = 2 * b2 + 6 * b3 * t0;
    
    % Final position, velocity, accleration for joint 2
    position2F = b0 + b1 * tF + b2 * tF^2 + b3 * tF^3;
    velocity2F = b1 + 2 * b2 * tF + 3 * b3 * tF^2;
    acceleration2F = 2 * b2 + 6 * b3 * tF;
    
    % Initial position, velocity, acceleration for joint 3
    position3I = c0 + c1 * t0 + c2 * t0^2 + c3 * t0^3;
    velocity3I = c1 + 2 * c2 * t0 + 3 * c3 * t0^2;
    acceleration3I = 2 * c2 + 6 * c3 * t0;
    
    % Final position, velocity, accleration for joint 3
    position3F = c0 + c1 * tF + c2 * tF^2 + c3 * tF^3;
    velocity3F = c1 + 2 * c2 * tF + 3 * c3 * tF^2;
    acceleration3F = 2 * c2 + 6 * c3 * tF;
    %}
    
    % Essential Cubic Polynomial Equations
    velocity1F = a1 + 2 * a2 * tF + 3 * a3 * tF^2;
    velocity2F = b1 + 2 * b2 * tF + 3 * b3 * tF^2;
    velocity3F = c1 + 2 * c2 * tF + 3 * c3 * tF^2;
    
    % This section computes the remaining coefficients. 
    % [NOTE: The position equations are called again for the values of a2]
    a2 = solve(velocity1F == 0, a2);
    b2 = solve(velocity2F == 0, b2);
    c2 = solve(velocity3F == 0, c2);
    
    position1F = a0 + a1 * tF + a2 * tF^2 + a3 * tF^3;
    position2F = b0 + b1 * tF + b2 * tF^2 + b3 * tF^3;
    position3F = c0 + c1 * tF + c2 * tF^2 + c3 * tF^3;
    
    a3 = double(solve(position1F == theta1F, a3));
    b3 = double(solve(position2F == theta2F, b3));
    c3 = double(solve(position3F == theta3F, c3));
    
    a2 = double(eval(a2));
    b2 = double(eval(b2));
    c2 = double(eval(c2));
end
