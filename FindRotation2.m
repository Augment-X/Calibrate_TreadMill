clc; clear all; close all; 


% Define parmeters

rot = [0; 15.7; 180];       % Rotation as entered into Vicon   
%rot = [0; 15.7; 180];  

MakersFromVicon= ...                 % value from Nexus measurment [Marker1(top left), Marker2(top right), Marker3(bottom left)]
  [202.7236, -197.2386,  197.0820; ...
 -241.9874, -246.9749,  250.4348 ; ...
   54.2698 ,  51.9645 , -32.2667] ;


%% DEFINE FORCE PLATE OBKJECT        
% Object Parameters
FPLenght = 600; % mm
FPwidth = 500; % mm
edgeBand = 50; % Width death zone mm (for marker position) 
UnitVecFactor = 100; % Scaling Factor (mm) of unitary Frame of Reference
MarkerHeight = 10; % Marker height base to centroid 
EdgeVec = ...
    [FPwidth/2, -FPwidth/2 , -FPwidth/2,  FPwidth/2, FPwidth/2;...
    FPLenght/2, FPLenght/2, -FPLenght/2, -FPLenght/2,  FPLenght/2;...
    0, 0 , 0 , 0 , 0];
MarkersVec = ...
    [-FPwidth/2+edgeBand, FPwidth/2-edgeBand , -FPwidth/2+edgeBand;...
    FPLenght/2-edgeBand, FPLenght/2-edgeBand, -FPLenght/2+edgeBand;...
    MarkerHeight, MarkerHeight , MarkerHeight];
unitVecX= [0, UnitVecFactor; 0 , 0 ; 0 , 0]; 
unitVecY= [0, 0; 0 , UnitVecFactor ; 0 , 0]; 
unitVecZ= [0, 0; 0 , 0 ; 0 , UnitVecFactor]; 

%% VISUALIZE %%
figure(); hold on; axis equal; grid on;  

plot3(EdgeVec(1,:), EdgeVec(2,:), EdgeVec(3,:),':k')
plot3(MarkersVec(1,:), MarkersVec(2,:), MarkersVec(3,:),'*k')
plot3(unitVecX(1,:), unitVecX(2,:), unitVecX(3,:),':r')
plot3(unitVecY(1,:), unitVecY(2,:), unitVecY(3,:),':g')
plot3(unitVecZ(1,:), unitVecZ(2,:), unitVecZ(3,:),':b')

%% ROTATION 
% rotation matrix 

steps = 40; 
rot = rot*pi()/180/steps;
ROT = eye(3);
for i = 1 : steps
    rotX = [ 1, 0, 0; 0, cos(rot(1)), -sin(rot(1)); 0, sin(rot(1)), cos(rot(1))]; 
    rotY = [cos(rot(2)),0 ,sin(rot(2)); 0, 1, 0 ; -sin(rot(2)), 0, cos(rot(2))]; 
    rotZ = [cos(rot(3)), -sin(rot(3)),0 ; sin(rot(3)), cos(rot(3)), 0 ; 0, 0, 1 ]; 
    ROT = rotZ*rotY*rotX*ROT;
end

% Transform Object 
EdgeVec = ROT*EdgeVec;
MarkersVec = ROT*MarkersVec;
unitVecX = ROT*unitVecX;
unitVecY= ROT*unitVecY;
unitVecZ = ROT*unitVecZ;

%% VISUALIZE %%
%figure(); hold on; axis equal; grid on;  

plot3(EdgeVec(1,:), EdgeVec(2,:), EdgeVec(3,:),'-k')
plot3(MarkersVec(1,:), MarkersVec(2,:), MarkersVec(3,:),'*k')
plot3(unitVecX(1,:), unitVecX(2,:), unitVecX(3,:),'-r')
plot3(unitVecY(1,:), unitVecY(2,:), unitVecY(3,:),'-g')
plot3(unitVecZ(1,:), unitVecZ(2,:), unitVecZ(3,:),'-b')
plot3(MakersFromVicon(1,:), MakersFromVicon(2,:), MakersFromVicon(3,:),'ok')


