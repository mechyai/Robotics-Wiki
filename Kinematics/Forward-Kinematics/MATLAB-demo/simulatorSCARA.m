function simulatorSCARA(movement_matrix, robot_size, num_frames)
% simulatorSCARA simulates multiple movements of SCARA type robot
%   movement matrix is [j x 3] matrix where j_i is a signifies a 
%   joint parameters at one instance. The 3 columns of the matrix are
%   theta1, theta2, and EE extension of the SCARA.
%   Robot size is the x, y, z bounds limits of the robot and will be the
%   viewport of the simulation
%   num_frames is the int number of frames you want your simulated video to
%   be

figure()

x_reach = robot_size(:,1);
y_reach = robot_size(:,2);
z_reach = robot_size(:,3);

mm_size = size(movement_matrix);
for (j=mm_size(:,1) )
    
    theta1_now = movement_matrix(j,1);
    theta2_now = movement_matrix(j,2);
    d4_now = movement_matrix(j,3);
    
    d_theta1 = theta1_now/total_frames;
    d_theta2 = theta2_now/total_frames;
    d_d4 = d4_now/total_frames;

    for(i=1:total_frames)
         clf
         plot3(0,0,0)
         hold on; grid on
         xlabel('X-coordinate'); ylabel('Y-coordinate'); zlabel('Z-coordinate')
         title(['SCARA Robot Simulation {frame = ',num2str(i),' }'])
         xlim([-x_reach, x_reach]) 
         ylim([-y_reach, y_reach]) 
         zlim([0, z_reach])
         xticks(-x_reach: 100: x_reach)
         yticks(-y_reach: 100: y_reach)
         zticks(0:100: z_reach)
         pbaspect([2*x_reach, 2*y_reach, z_reach])

    % function [T01] = transformationMatrixFromDhTable(distance_d_i, angle_theta_i, distance_a_i_minus1, angle_alpha_i_minus1)
        T01 = transformationMatrixFromDhTable(d1,theta1current, a0, alpha0);
        T12 = transformationMatrixFromDhTable(d2,theta2current, a1, alpha1);
        T23 = transformationMatrixFromDhTable(d3,theta3, a2, alpha2);
        T3EE = transformationMatrixFromDhTable(d4current,theta4, a3, alpha3);
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

        getframe

        theta1current = theta1current+d_theta1;
        theta2current = theta2current+d_theta2;
        d4current = d4current + d_d4;

    %     if (i == (total_frames) && count <4)
    %             i =0;
    %             theta1current = 0;
    %             theta2current = 0;
    %             d4current = 0;
    %             count = count+1;
    %             pause(3);
    %     end
     end
end

end

