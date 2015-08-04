%{
% @author Howard Chiang
%   
% This calculates the trajectory of the arm using cubic polynomials.  
% 
% 
% @param theta1C = current theta1
% @param theta2C = current theta2
% @param theta1F = Final theta1 position
% @param theta2F = Final theta2 position
%
% @return 
%}

function [position1, velocity1, acceleration1, position2, velocity2, acceleration2, position3, velocity3, acceleration3] = calcTrajectory(theta1C, theta2C, theta3C, theta1F, theta2F, theta3F)
    
    % Coefficients to be found
    syms a2;
    syms a3;
    syms b2;
    syms b3;
    syms c2;
    syms c3;
    tF = 5; % This can be changed/raised to relax the system.
    
    % This sets the given theta values to the coefficients of the cubic 
    % polynomials. 
    a0 = theta1C;
    a1 = 0;        % this is known to be true 
    
    b0 = theta2C;
    b1 = 0;        % this is known to be true
    
    c0 = theta3C;
    c1 = 0;        % this is known to be true 
    
    
    % Initial position, velocity, acceleration for joint 1
    %position1I = a0 + a1 * t0 + a2 * t0^2 + a3 * t0^3;
    %velocity1I = a1 + 2 * a2 * t0 + 3 * a3 * t0^2;
    %acceleration1I = 2 * a2 + 6 * a3 * t0;
    
    % Final position, velocity, accleration for joint 1
    %position1F = a0 + a1 * tF + a2 * tF^2 + a3 * tF^3;
    velocity1F = a1 + 2 * a2 * tF + 3 * a3 * tF^2;
    %acceleration1F = 2 * a2 + 6 * a3 * tF;
    
    % Initial position, velocity, acceleration for joint 2
    %position2I = b0 + b1 * t0 + b2 * t0^2 + b3 * t0^3;
    %velocity2I = b1 + 2 * b2 * t0 + 3 * b3 * t0^2;
    %acceleration2I = 2 * b2 + 6 * b3 * t0;
    
    % Final position, velocity, accleration for joint 2
    %position2F = b0 + b1 * tF + b2 * tF^2 + b3 * tF^3;
    velocity2F = b1 + 2 * b2 * tF + 3 * b3 * tF^2;
    %acceleration2F = 2 * b2 + 6 * b3 * tF;
    
    % Initial position, velocity, acceleration for joint 3
    %position3I = c0 + c1 * t0 + c2 * t0^2 + c3 * t0^3;
    %velocity3I = c1 + 2 * c2 * t0 + 3 * c3 * t0^2;
    %acceleration3I = 2 * c2 + 6 * c3 * t0;
    
    % Final position, velocity, accleration for joint 3
    %position3F = c0 + c1 * tF + c2 * tF^2 + c3 * tF^3;
    velocity3F = c1 + 2 * c2 * tF + 3 * c3 * tF^2;
    %acceleration3F = 2 * c2 + 6 * c3 * tF;
    
    % This section computes the remaining coefficients. Note that the
    % position equations are called again to replace the values of a2
    a2 = solve(velocity1F == 0, a2);
    position1F = a0 + a1 * tF + a2 * tF^2 + a3 * tF^3;
    a3 = double(solve(position1F == theta1F, a3));
    a2 = double(eval(a2));
    
    b2 = solve(velocity2F == 0, b2);
    position2F = b0 + b1 * tF + b2 * tF^2 + b3 * tF^3;
    b3 = double(solve(position2F == theta2F, b3));
    b2 = double(eval(b2));
    
    c2 = solve(velocity3F == 0, c2);
    position3F = c0 + c1 * tF + c2 * tF^2 + c3 * tF^3;
    c3 = double(solve(position3F == theta3F, c3));
    c2 = double(eval(c2));
    
 
    position1 = tf([a3 a2 a1 a0]);
    velocity1 = tf([3*a3 2*a2 a1]);
    acceleration1 = tf([6*a3 2*a2]);
    
    position2 = tf([b3 b2 b1 b0]);
    velocity2 = tf([3*b3 2*b2 b1]);
    acceleration2 = tf([6*b3 2*b2]);
    
    position3 = tf([c3 c2 c1 c0]);
    velocity3 = tf([3*c3 2*c2 c1]);
    acceleration3 = tf([6*c3 2*c2]);

    %{
    % Final Equations
    position1 = a0 + a1 * t + a2 * t^2 + a3 * t^3;
    velocity1 = a1 + 2 * a2 * t + 3 * a3 * t^2;
    acceleration1 = 2 * a2 + 6 * a3 * t;
    

    position2 = b0 + b1 * t + b2 * t^2 + b3 * t^3;
    velocity2 = b1 + 2 * b2 * t + 3 * b3 * t^2;
    acceleration2 = 2 * b2 + 6 * b3 * t;
    
    position3 = c0 + c1 * t + c2 * t^2 + c3 * t^3;
    velocity3 = c1 + 2 * c2 * t + 3 * c3 * t^2;
    acceleration3 = 2 * c2 + 6 * c3 * t;
    %}
end
