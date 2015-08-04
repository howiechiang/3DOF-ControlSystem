%{
% @author Howard Chiang
% 
% This function calculates the cartesian position of the end effector for
% all possible configurations of theta. These matrix of these positions are
% then plotted to create a map of the workspace. 
%
%}


function a = showWorkspace()
    theta1 =  -pi*2/3:0.025:pi*2/3;
    theta2 =  -pi:0.025:pi;

    for i = 1:length(theta1)
        for j = 1:length(theta2)
                Px(i,j) = 2.5 + 10*cos(theta1(i) + theta2(j)) + 10*cos(theta1(i));
                Py(i,j) = 10*sin(theta1(i) + theta2(j)) + 10*sin(theta1(i));
        end
    end

    figure
    hold on
    for i=1:length(theta1)
        %if Px(i,:) > 0
            scatter(Px(i,:),Py(i,:));
        %end
    end
    
    axis([-25 25 -20 20])
end 
