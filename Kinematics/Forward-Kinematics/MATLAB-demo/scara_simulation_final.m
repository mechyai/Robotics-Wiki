% ECE 5463 - HW: PA2 (MATLAB version)
% Chris Eubel.1
% 10/4/2019
clear, clc, close all

% Fixed Geometry of SCARA
d1_SCARA = 300; % (mm)
a1_SCARA = 200; % (mm)
a2_SCARA = 150; % (mm) [a2 < a1]
d3_extension_restriction = d1_SCARA - 25;
theta1_angle_restriction = 360; % (degrees) [non-continuous rotation]
theta2_angle_restriction = 170; % (degrees) [<180 degrees]

% Gather input from users
while 1
        theta1_var = input('Please enter the angle value (degrees) for Theta1: ');
        if(abs(theta1_var) <= theta1_angle_restriction)
            break;
        end
        fprintf('\nERROR! Theta1 must be an angle between +- %i degrees \n', theta1_angle_restriction);
end
while 1
        theta2_var = input('\nPlease enter the angle value (degrees) for Theta2: ');
        if (abs(theta2_var) <= theta2_angle_restriction)
            break;
        end
        fprintf('\nERROR! Theta2 must be an angle value between +- %i degrees \n', theta2_angle_restriction);
end
while 1
        d3_var = input('\nPlease enter the distance value (mm) for D3 (prismatic extenstion) : ');
        if (d3_var < d3_extension_restriction && d3_var >= 0)
            break;
        end
        fprintf('\nERROR! D3 must be a positive distance value less than %i \n ', d3_extension_restriction);
end
        

% Geometric Parameters for SCARA
% Link 1:
a0 = 0; % var (mm)
alpha0 = 0;  % fixed (degrees)
d1 = 0; % change above (mm)
theta1 = theta1_var; % VAR* (degrees)

% Link 2:
a1 = a1_SCARA; % change above (mm)
alpha1 = 0;  % fixed (degrees)
d2 = d1_SCARA; % fixed (mm)
theta2 = theta2_var; % VAR* (degrees)

% Link 3:
a2 = a2_SCARA; % change above (mm)
alpha2 = 180;  % fixed (degrees)
d3 = 0; % var (mm)
theta3 = 0; % fixed (degrees)

% Link 4 (EE):
% Link 2:
a3 = 0; % fixed (mm)
alpha3 = 0;  % fixed (degrees)
d4 = d3_var; % VAR* (mm)
theta4 = 0; % fixed (degrees)


%% MOVIE ASPECTS
total_frames = 50;

% MOVEMENT MATRIX ------------ adjust
movement_matrix = [
    45, -45, 150
    45, 20, 79
    120, -90, 5
    120, -90, 200
    0, 0, 0
    theta1, theta2, d4];

movement_matrix = [theta1, theta2, d4];

theta1_now = 0; % (degrees)
theta2_now = 0; % (degrees)
d4_now = 0; % (mm)
 
% PLOTTING
close all
figure()
size_mm = size(movement_matrix);
num_movements = size_mm(:,1);
j = 1;
while ( j <= num_movements)
    i = 1;
    theta1_current = movement_matrix(j,1); % (degrees)
    theta2_current = movement_matrix(j,2); % (degrees)
    d4_current = movement_matrix(j,3); % (mm)
    
    % incremental changes
    d_theta1 = (theta1_current-theta1_now)/total_frames;  
    d_theta2 = (theta2_current-theta2_now)/total_frames;
    d_d4 = (d4_current-d4_now)/total_frames;
    
     while (i <= total_frames+1)
         clf
         plot3(0,0,0)
         hold on; grid on
         xlabel('X-coordinate'); ylabel('Y-coordinate'); zlabel('Z-coordinate')
         title(['SCARA Robot Simulation [movement # = ',num2str(j),', frame = ',num2str(i),' ]'])
%          legend('a','b','c','d')
         xlim([-(a1_SCARA+a2_SCARA+50), a1_SCARA+a2_SCARA+50]); 
         ylim([-(a1_SCARA+a2_SCARA+50) , a1_SCARA+a2_SCARA+50]); 
         zlim([0,d1_SCARA+d3_var+50])
         xticks(-(a1_SCARA+a2_SCARA+50):100: a1_SCARA+a2_SCARA+50)
         yticks(-(a1_SCARA+a2_SCARA+50): 100: a1_SCARA+a2_SCARA+50)
         zticks(0:100: d1_SCARA+d3_var+50)
         pbaspect([2*(a1_SCARA+a2_SCARA+50), 2*(a1_SCARA+a2_SCARA+50), d1_SCARA+d3_var+50])

    % function [T01] = transformationMatrixFromDhTable(distance_d_i, angle_theta_i, distance_a_i_minus1, angle_alpha_i_minus1)
        T01 = transformationMatrixFromDhTable(d1,theta1_now, a0, alpha0);
        T12 = transformationMatrixFromDhTable(d2,theta2_now, a1, alpha1);
        T23 = transformationMatrixFromDhTable(d3,theta3, a2, alpha2);
        T3EE = transformationMatrixFromDhTable(d4_now,theta4, a3, alpha3);
        T0EE = T01*T12*T23*T3EE;


         % VISUALS
         % {0}
        plot3(0,0,0,'s','LineWidth',15,'color','k')

        % LINK 1
        link1_end = T01*[a1_SCARA, 0, d1_SCARA, 1]';

        link1_x = [0, 0, link1_end(1)];
        link1_y = [0, 0, link1_end(2)];
        link1_z = [0, d1_SCARA, link1_end(3) ];

        plot3(link1_x,link1_y,link1_z,'o-','LineWidth',7,'color','k'); 

        % LINK 2

        link2_begin =  link1_end;
        link2_end = T01*T12*[a2, 0, 0, 1]';

        link2_x = [link2_begin(1), link2_end(1)];
        link2_y = [link2_begin(2), link2_end(2)];
        link2_z = [link2_begin(3), link2_end(3)];

        plot3(link2_x, link2_y, link2_z,'o-','LineWidth',5,'color','b')

        % LINK 3

        link3 = link2_end;

        link3_x = link3(1);
        link3_y = link3(2);
        link3_z = link3(3);

        plot3(link3_x, link3_y, link3_z, 's','LineWidth',12,'color','r')

        % LINK 4 (EE) - Assuming telescoping arm

        link4_begin = link3;
        link4_end = T01*T12*T23*T3EE*[0, 0, d3, 1]';
        EE_x = [link4_begin(1), link4_end(1)];
        EE_y = [link4_begin(2), link4_end(2)];
        EE_z = [link4_begin(3), link4_end(3)];

        plot3(EE_x, EE_y, EE_z,'-^','LineWidth',3,'color','r');

        % Update joint parameters
        theta1_now = theta1_now+d_theta1;
        theta2_now = theta2_now+d_theta2;
        d4_now = d4_now + d_d4;

        i = i +1;
        movieVector(i) = getframe; %NEW
     end
     j = j +1;
end 
 fprintf('\nEnd Effector final position in space:\n')
 final_coordinates = link4_end(1:3,:);
 disp(final_coordinates) 
 video = VideoWriter('SCARAvid'); %NEW
 open(video) %NEW
 writeVideo(video, movieVector) %NEW
 close(video) %NEW