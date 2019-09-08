

L1 = 0.1;
L2 = 0.4;
L3 = 0.4;
l = 1;  % length %
w = 0.4; % width %

position = [0 0 0]; % x, y, z %
orientation = [0 0 0];  % roll, pitch, yaw %

theta = [0 0 0];    % theta_1, theta_2, theta_3 %

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
        1 0 0 0;
        0 0 0 1];

T12 = [0 0 -1 0; -1 0 0 0; 0 0 1 0; 0 0 0 1];

T23 = [cos(theta(2)) -sin(theta(2)) 0 L2*cos(theta(2));
        sin(theta(2)) cos(theta(2)) 0 L2*sin(theta(2));
        0 0 1 0;
        0 0 0 1];
T34 = [cos(theta(3)) -sin(theta(3)) 0 L3*cos(theta(3));
        sin(theta(3)) cos(theta(3)) 0 L3*sin(theta(3));
        0 0 1 0;
        0 0 0 1];

% transformation from base of leg to foot %
T = T01 * T12 * T23 * T34;    

disp(T * [0; 0; 0; 1])


    
    
    


