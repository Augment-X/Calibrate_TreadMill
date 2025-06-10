clc; clear all; close all; 

%% DEFINE FORCE PLATE OBJECT        
FPLenght = 600; % mm
FPwidth = 500; % mm
edgeBand = 40; % Width death zone mm (for marker position) 
UnitVecFactor = 100; % Scaling Factor (mm) of unitary Frame of Reference
MarkerHeight = 0; % Marker height base to centroid 

global MarkersVec EdgeVec unitVecX unitVecY unitVecZ MakersFromViconRotCalc ;

 % MarkersVec = ...
 %     [-FPwidth/2+edgeBand, FPwidth/2-edgeBand , -FPwidth/2+edgeBand;...
 %     FPLenght/2-edgeBand, FPLenght/2-edgeBand, -FPLenght/2+edgeBand;...
 %     MarkerHeight, MarkerHeight , MarkerHeight];

theta=pi;
ROT_init=[cos(theta) -sin(theta) 0;...
          sin(theta) cos(theta) 0;...
          0 0 1];

MarkersVec_1=[
    -FPwidth/2+edgeBand, FPwidth/2-edgeBand, -FPwidth/2+edgeBand;...
    FPLenght/2-edgeBand, FPLenght/2-edgeBand, -FPLenght/2+edgeBand;...
    MarkerHeight, MarkerHeight, MarkerHeight];

MarkersVec = ROT_init*MarkersVec_1;
EdgeVec = ...
    [FPwidth/2, -FPwidth/2 , -FPwidth/2,  FPwidth/2, FPwidth/2;...
    FPLenght/2, FPLenght/2, -FPLenght/2, -FPLenght/2,  FPLenght/2;...
    0, 0 , 0 , 0 , 0];

unitVecX= [0, UnitVecFactor; 0 , 0 ; 0 , 0]; 
unitVecY= [0, 0; 0 , UnitVecFactor ; 0 , 0]; 
unitVecZ= [0, 0; 0 , 0 ; 0 , UnitVecFactor]; 

%% VICON MEASUREMENT DATA

%%%%%% Useing the algorithm with Vicon Workspace %%%%%%%
% Value from Nexus measurment [Marker1(top left), Marker2(top right), Marker3(bottom left)]
% Connect to Vicon Nexus
% vicon = ViconNexus();
% Markers = vicon.GetMarkerNames('ForcePlate3Points');
% n = length(Markers);
% 
% trajX = cell(1, n); trajY = cell(1, n); trajZ = cell(1, n);
% for i = 1:n
%  [trajX{i}, trajY{i}, trajZ{i}, ~] = vicon.GetTrajectory('ForcePlate3Points', Markers{i});
% end
% X = cellfun(@mean, trajX);
% Y = cellfun(@mean, trajY);
% Z = cellfun(@mean, trajZ);
% 
% M1 = [X(1); Y(1); Z(1)];
% M2 = [X(2); Y(2); Z(2)];
% M3 = [X(3); Y(3); Z(3)];
% 
% Center = (M2+M3)/2;
% disp('position of Force Plate Center:');
% disp(Center);
% MakersFromViconRotCalc =[M1-Center,M2-Center,M3-Center];

%%%%%% INNER Script TESTING %%%%%%%
% We create fake measurement data to simulate the Vicon output to help develop the algorithm. 
% Uncomment the following lines to use the hardcoded values for testing

testRot = [-80; 15; -85]; 
testRot = testRot*pi/180; % Rotation to be applied to markers from vicon (in radians)
testROT = rotationMatrix(testRot); % build test rotation matrix
MakersFromViconRotCalc = testROT * MarkersVec; % apply rotation to markers

%% VISUALIZE %% initial position
figure(1); hold on; axis equal; grid on;  xlim([-1000, 1000]); ylim([-1000, 1000])

plot3(EdgeVec(1,:), EdgeVec(2,:), EdgeVec(3,:),':k')
plot3(MarkersVec(1,:), MarkersVec(2,:), MarkersVec(3,:),'*k')
plot3(unitVecX(1,:), unitVecX(2,:), unitVecX(3,:),':r')
plot3(unitVecY(1,:), unitVecY(2,:), unitVecY(3,:),':g')
plot3(unitVecZ(1,:), unitVecZ(2,:), unitVecZ(3,:),':b')
plot3(MakersFromViconRotCalc(1,:), MakersFromViconRotCalc(2,:), MakersFromViconRotCalc (3,:),'ok')


%% OPTIMIZATION

global err 
err = [200];
Xub = [pi; pi; pi]; Xlb = -Xub; % Upper and lower bounds
X0 = [0; 0; 0];      % Innitial seed rotation 

while err(end) >=10
    OPTIONS = optimoptions('fmincon','Algorithm','SQP');  % alternatives : interior point, SQP,  active-set, and trust-region-reflective
    Xopt = fmincon(@objective, X0, [], [], [], [], Xlb, Xub, [], OPTIONS );  
    X0 = randn(3,1)*pi; % New seed for next iteration
end


% Output to terminal
disp(Xopt*180/pi); % FINAL ANSWER (to be entered into Vicon) 
%disp('instead of :');
%disp(testRot*180/pi); % TEST ROTATION (to be compared with Vicon)


%% Rotate object after optimization

ROT = rotationMatrix(Xopt);
% Transform Object 
EdgeVec = ROT*EdgeVec;
MarkersVecFinal = ROT*MarkersVec;
unitVecX = ROT*unitVecX;
unitVecY= ROT*unitVecY;
unitVecZ = ROT*unitVecZ;


%% VISUALIZE %% final position
figure(1); %hold on; axis equal; grid on;  

plot3(EdgeVec(1,:), EdgeVec(2,:), EdgeVec(3,:),'-k')
plot3(MarkersVecFinal(1,:), MarkersVecFinal(2,:), MarkersVecFinal(3,:),'*k')
plot3(unitVecX(1,:), unitVecX(2,:), unitVecX(3,:),'-r')
plot3(unitVecY(1,:), unitVecY(2,:), unitVecY(3,:),'-g')
plot3(unitVecZ(1,:), unitVecZ(2,:), unitVecZ(3,:),'-b')

figure(2); hold on; grid on;  
plot(err); ylabel('Objective Function Value (mm)');

%% OPTIMIZATION

function obj = objective(X1)
    ROT1 = rotationMatrix(X1);
    global MarkersVec  
    global MakersFromViconRotCalc
    global err
    MarkersVec0 = ROT1*MarkersVec;
    obj = sqrt(sum(sum((MarkersVec0-MakersFromViconRotCalc).^2)));
    err = [err, obj];
end

function ROT2 = rotationMatrix(X2)
   % X2=deg2rad(X);
    steps = 1;             % X2 is divided into steps to approximate the rotation X2 is in Radians
    X2 = X2/steps;
    ROT2 = eye(3);
    for i = 1 : steps
        rotX = [ 1, 0, 0; 0, cos(X2(1)), -sin(X2(1)); 0, sin(X2(1)), cos(X2(1))]; 
        rotY = [cos(X2(2)),0 ,sin(X2(2)); 0, 1, 0 ; -sin(X2(2)), 0, cos(X2(2))]; 
        rotZ = [cos(X2(3)), -sin(X2(3)),0 ; sin(X2(3)), cos(X2(3)), 0 ; 0, 0, 1 ]; 
        ROT2 = rotZ*rotY*rotX*ROT2;
    end
end