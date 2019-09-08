

L1 = 0.1;
L2 = 0.4;
L3 = 0.4;
l = 1;  % length %
w = 0.4; % width %

position = [0 0 0]; % x, y, z %
orientation = [0 0 0];  % roll, yaw, pitch %

theta = [0 -pi/6 pi/4];    % theta_1, theta_2, theta_3 %

% construct XYZ rotation matrix %
Rx = [1 0 0 0; 
    0 cos(orientation(1)) -sin(orientation(1)) 0;
    0 sin(orientation(1)) cos(orientation(1)) 0;
    0 0 0 1];
Ry = [cos(orientation(2)) 0 sin(orientation(2)) 0; 
    0 1 0 0;
    -sin(orientation(2)) 0 cos(orientation(2)) 0;
    0 0 0 1];
Rz = [cos(orientation(3)) -sin(orientation(3)) 0 0; 
    sin(orientation(3)) cos(orientation(3)) 0 0;
    0 0 1 0;
    0 0 0 1];

Rxyz = Rx*Ry*Rz;

% body transform matrix *
Tm = Rxyz * [1 0 0 position(1); 0 1 0 position(2); 0 0 1 position(3); 0 0 0 1];

% transformation from body to leg frames %
Trb = Tm * [0 0 1 -l/2; 0 1 0 0; -1 0 0 w/2; 0 0 0 1];
Trf = Tm * [0 0 1 l/2; 0 1 0 0; -1 0 0 w/2; 0 0 0 1];
Tlf = Tm * [0 0 -1 l/2; 0 1 0 0; 1 0 0 -w/2; 0 0 0 1];
Tlb = Tm * [0 0 -1 -l/2; 0 1 0 0; 1 0 0 -w/2; 0 0 0 1];

T01 = [cos(theta(1)) -sin(theta(1)) 0 -L1*cos(theta(1));
        sin(theta(1)) cos(theta(1)) 0 -L1*sin(theta(1));
        0 0 1 0;
        0 0 0 1];

T12 = [0 0 -1 0; -1 0 0 0; 0 1 0 0; 0 0 0 1];

T23 = [cos(theta(2)) -sin(theta(2)) 0 L2*cos(theta(2));
        sin(theta(2)) cos(theta(2)) 0 L2*sin(theta(2));
        0 0 1 0;
        0 0 0 1];
T34 = [cos(theta(3)) -sin(theta(3)) 0 L3*cos(theta(3));
        sin(theta(3)) cos(theta(3)) 0 L3*sin(theta(3));
        0 0 1 0;
        0 0 0 1];

% transformation from base of leg to foot %
T02 = T01 * T12;
T03 = T02 * T23;
T04 = T03 * T34;    

ee_vec = [0;0;0;1];

body_coords = Tm*ee_vec;
l20 = Trf * ee_vec
l30 = Tlf * ee_vec;
l10 = Trb * ee_vec;
l40 = Tlb * ee_vec;

l21 = Trf * T01 * ee_vec
l22 = Trf * T02 * ee_vec
l23 = Trf * T03 * ee_vec
l24 = Trf * T04 * ee_vec

hold on
plotSide(l20);
plotSide(l21);
plotSide(l22);
plotSide(l23);
plotSide(l24);
axis([-0.5 1.5 -1 0.2])
xlabel("X")
ylabel("Y")

% hold on
% plotTop(body_coords);
% plotTop(l20);
% plotTop(l30);
% plotTop(l10);
% plotTop(l40);
% axis([-2 2 -2 2])
% xlabel("Z")
% ylabel("X")
% hold off

% hold on
% plotSide(body_coords);
% plotSide(l20);
% plotSide(l30);
% plotSide(l10);
% plotSide(l40);
% plotSide(l21);
% plotSide(l22);
% plotSide(l23);
% plotSide(l24);
% axis([-2 2 -2 2])
% xlabel("X")
% ylabel("Y")
% hold off





function plotTop(coords)
    scatter(coords(3), coords(1));
end

function plotSide(coords)
    scatter(coords(1), coords(2));
end

function plot3vec(coords)
    scatter(coords(1), coords(2), coords(3));
end

    
    
    



