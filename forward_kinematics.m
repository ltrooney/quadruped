
global L1 L2 L3
L1 = 0.1;
L2 = 0.4;
L3 = 0.4;
l = 1;  % length %
w = 0.4; % width %
timesteps = 100;

position = cell(1, timesteps);
orientation = cell(1, timesteps);
theta = cell(1, timesteps);


for x = 1:timesteps
    position{1,x} = [0; 0; 0; 1]; % x, y, z, 1 %
    
    orientation{1,x} = [0.2*sin(x / 2); 0; 0; 1];  % roll, yaw, pitch, 1 %
    
    theta{1,x} = [
        0 0 0;  % leg 1 %    
        0 0 0;  % leg 2 %
        0 0 0;  % leg 3 %
        0 0 0;  % leg 4 %
    ];  
end


for t = 1:timesteps  
    % construct XYZ rotation matrix %
    Rx = [1 0 0 0; 
        0 cos(orientation{t}(1)) -sin(orientation{t}(1)) 0;
        0 sin(orientation{t}(1)) cos(orientation{t}(1)) 0;
        0 0 0 1];
    Ry = [cos(orientation{t}(2)) 0 sin(orientation{t}(2)) 0; 
        0 1 0 0;
        -sin(orientation{t}(2)) 0 cos(orientation{t}(2)) 0;
        0 0 0 1];
    Rz = [cos(orientation{t}(3)) -sin(orientation{t}(3)) 0 0; 
        sin(orientation{t}(3)) cos(orientation{t}(3)) 0 0;
        0 0 1 0;
        0 0 0 1];

    Rxyz = Rx*Ry*Rz;

    % body transform matrix *
    Tm = Rxyz * [1 0 0 position{t}(1); 0 1 0 position{t}(2); 0 0 1 position{t}(3); 0 0 0 1];

    % transformation from body to leg frames %
    Trb = Tm * [0 0 1 -l/2; 0 1 0 0; -1 0 0 w/2; 0 0 0 1];
    Trf = Tm * [0 0 1 l/2; 0 1 0 0; -1 0 0 w/2; 0 0 0 1];
    Tlf = Tm * [0 0 -1 l/2; 0 1 0 0; 1 0 0 -w/2; 0 0 0 1];
    Tlb = Tm * [0 0 -1 -l/2; 0 1 0 0; 1 0 0 -w/2; 0 0 0 1];

    body_coords = Tm * [0;0;0;1];

    leg_1_coords = coords(1, theta{t}, Trb);
    leg_2_coords = coords(2, theta{t}, Trf);
    leg_3_coords = coords(3, theta{t}, Tlf);
    leg_4_coords = coords(4, theta{t}, Tlb);

    leg_coords = {leg_1_coords, leg_2_coords, leg_3_coords, leg_4_coords};
    
    render(body_coords, leg_coords);
    pause(0.01);
end


function render(body_coords, leg_coords)
    
    % draw coordinate frame origins %
    h = plot3d(body_coords);
    hold on
    plotLeg3d(cell2mat(leg_coords));
    
    % draw body plane %
    body_plane_x = zeros(4,1);
    body_plane_y = zeros(4,1);
    body_plane_z = zeros(4,1);

    for i = 1:4
        body_plane_x(i) = leg_coords{1,i}(1,1);
        body_plane_y(i) = leg_coords{1,i}(3,1);
        body_plane_z(i) = leg_coords{1,i}(2,1);
    end
    
    patch(body_plane_x, body_plane_y, body_plane_z, 'b');
    
    % draw links
    for i = 1:4
        leg = leg_coords{1,i};
        line([leg(1,1); leg(1,2)], [leg(3,1); leg(3,2)], [leg(2,1); leg(2,2)], 'LineWidth', 4);
        line([leg(1,3); leg(1,4)], [leg(3,3); leg(3,4)], [leg(2,3); leg(2,4)], 'LineWidth', 4);
        line([leg(1,4); leg(1,5)], [leg(3,4); leg(3,5)], [leg(2,4); leg(2,5)], 'LineWidth', 4);
    end    
    
    % set axis boundaries %
    xlim([-1 1])
    ylim([-1 1])
    zlim([-1 0.5])
    view(3)

    hold off
    
%     drawnow
end

function leg_coords = coords(leg_num, theta, T_leg_base)
% Compute foot coordinates of the leg given by leg_num
% Returns a 4x5 matrix
    ee_vec = [0; 0; 0; 1];
    
    [T01, T02, T03, T04] = legTransformation(theta(leg_num,:));   % leg transformations %
    
    leg_coords = zeros(4,5);
    leg_coords(:,1) = T_leg_base * ee_vec;
    leg_coords(:,2) = T_leg_base * T01 * ee_vec;
    leg_coords(:,3) = T_leg_base * T02 * ee_vec;
    leg_coords(:,4) = T_leg_base * T03 * ee_vec;
    leg_coords(:,5) = T_leg_base * T04 * ee_vec;
end


function [T01, T02, T03, T04] = legTransformation(theta)
% LEGTRANSFORMATION Gets the transformation matrix from leg trunk to foot
    
    global L1 L2 L3;

    % transformation from base of leg to hip %    
    T01 = [cos(theta(1)) -sin(theta(1)) 0 -L1*cos(theta(1));
            sin(theta(1)) cos(theta(1)) 0 -L1*sin(theta(1));
            0 0 1 0;
            0 0 0 1];

    T12 = [0 0 -1 0; -1 0 0 0; 0 1 0 0; 0 0 0 1];
    
    % transformation from base of leg to hip % 
    T02 = T01 * T12;

    T23 = [cos(theta(2)) -sin(theta(2)) 0 L2*cos(theta(2));
            sin(theta(2)) cos(theta(2)) 0 L2*sin(theta(2));
            0 0 1 0;
            0 0 0 1];
        
    % transformation from base of leg to knee % 
    T03 = T02 * T23;
    
    T34 = [cos(theta(3)) -sin(theta(3)) 0 L3*cos(theta(3));
            sin(theta(3)) cos(theta(3)) 0 L3*sin(theta(3));
            0 0 1 0;
            0 0 0 1];
        
    % transformation from base of leg to foot % 
    T04 = T03 * T34;
end

function plotLeg3d(leg_coords)
% leg_coords is a 4x20 double, every 5 cols represents the 5 coordinate
% frames of each leg
    for i = 1:size(leg_coords,2)
        plot3d(leg_coords(:,i));
    end
end

function p = plot3d(coords)
    p = plot3(coords(1), coords(3), coords(2), '-o');
end

    
    
    



