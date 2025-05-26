clc;
vicon = ViconNexus();
Markers = vicon.GetMarkerNames('ForcePlate3Points');
n = length(Markers);
% Récupération des trajectoires moyennes des marqueurs
trajX = cell(1, n); trajY = cell(1, n); trajZ = cell(1, n);
for i = 1:n
 [trajX{i}, trajY{i}, trajZ{i}, ~] = vicon.GetTrajectory('ForcePlate3Points', Markers{i});
end
X = cellfun(@mean, trajX);
Y = cellfun(@mean, trajY);
Z = cellfun(@mean, trajZ);
% Calcul du repère local (R)
M1 = [X(1), Y(1), Z(1)];
M2 = [X(2), Y(2), Z(2)];
M3 = [X(3), Y(3), Z(3)];

v1 = (M2 - M1) / norm(M2 - M1); % X local
v_temp = (M3 - M1) / norm(M3 - M1); % direction auxiliaire                  TT: c'est pas -v2 ça? 
v3 = cross(v_temp, v1); % Z local                                           TT: et donc ici c'est -v3 ?  a verifier
%v3 = v3 / norm(v3);                                                     %  TT: ça a peu de sens. V2 et v1 sont unitaire et orthogonal 
v2 = cross(v3, v1); % Y local réorthonormé                                  TT: hmmm
v2=v2/norm(v2);                                 %                           TT: hmmm    
%v3=-v3;                                                                    TT: hmmm
R = [v1; v2; v3]'; % Matrice de rotation
% Application d'une rotation de 180° autour de Z si nécessaire
Rz180 = axang2rotm([0 0 1 pi]);                         
R_corrected = R * Rz180;                                                    % Là tu m"enbouche un coin 

% Quaternion corrigé
quat_corrected = rotm2quat(R_corrected); % [w x y z]
quat_corrected = quat_corrected/norm(quat_corrected);
R2 = quat2rotm(quat_corrected);
eul_rad = quat2eul(quat_corrected, 'ZXY');
eul_deg = rad2deg(eul_rad);
angleX = abs(eul_deg(2));
angleY = abs(eul_deg(3));
angleZ = abs(eul_deg(1));
% --- Cas 180° autour de Z : inversion X/Y
if abs(angleZ - 180) < 5
 temp = eul_deg(2);
 eul_deg(2) = eul_deg(3);
 eul_deg(3) = temp;
end
% --- Détection de l'axe de rotation principal (hors Z)
main_axis = '';
if angleX > 5 && angleX > angleY
 main_axis = 'X';
elseif angleY > 5 && angleY >= angleX
 main_axis = 'Y';
end
% --- Correction par rapport à la rotation en Z
correction = 0;
if angleZ >= 30 && angleZ < 75
 correction = -5;
elseif angleZ >= 75 && angleZ < 120
 correction = -10;
elseif angleZ >= 120 && angleZ < 165
 correction = -12;
elseif angleZ >= 165 && angleZ <= 170
 correction = -15;
end
% --- Appliquer la correction uniquement sur l'axe NON principal
if correction ~= 0
 switch main_axis
 case 'X'
 eul_deg(3) = eul_deg(3) + correction; % corriger Y
 case 'Y'
 eul_deg(2) = eul_deg(2) + correction; % corriger X
 end
end
% Dimensions et centre géométrique
d12 = norm([X(2)-X(1), Y(2)-Y(1), Z(2)-Z(1)]);
d13 = norm([X(3)-X(1), Y(3)-Y(1), Z(3)-Z(1)]);
dim_x = d12 + 2*38.25;
dim_y = d13 + 2*38.25;
% Vecteurs pour calcul du centre (même plan)
vect_x = v1;
vect_y = (M3 - M1) / norm(M3 - M1);
centre = [(d12/2)*vect_x(1) + X(1) + (d13/2)*vect_y(1), ...
 (d13/2)*vect_y(2) + Y(1) + (d12/2)*vect_x(2), ...
 mean(Z) - 9];
% Écriture du fichier
filename = 'FP_parameters.txt';
fid = fopen(filename, 'w');
fprintf(fid, 'Parameters de la FP\n-----------------------------\n\n');
fprintf(fid, 'Positions des markers (en mm):\n');
for i = 1:length(Markers)
 fprintf(fid, 'Marqueur %d (%s): X=%.2f, Y=%.2f, Z=%.2f\n', i, Markers{i}, X(i), Y(i), Z(i));
end
fprintf(fid, '\nDimensions de la plaque:\n');
fprintf(fid, '-Longueur X: %.2f mm\n-Largeur Y: %.2f mm\n\n', dim_x, dim_y);
fprintf(fid, 'Position du centre:\n');
fprintf(fid, '-Centre X: %.2f mm\n-Centre Y: %.2f mm\n-Centre Z: %.2f mm\n\n', centre(1), centre(2), centre(3));
fprintf(fid, 'Orientation (en degrees):\n');
fprintf(fid, '-Angle X: %.2f°\n-Angle Y: %.2f°\n-Angle Z: %.2f°\n\n', eul_deg(2), eul_deg(3), eul_deg(1));
fprintf(fid, 'Quaternion (x,y,z,w):\n');
fprintf(fid, '%.6f, %.6f, %.6f, %.6f\n', ...
 quat_corrected(2), quat_corrected(3), quat_corrected(4), quat_corrected(1)); % format Vicon : x y z w
fclose(fid);
% % % VISU
% % Reconstruction de la rotation depuis le quaternion exporté (comme Nexus le ferait)
% R_from_quat = quat2rotm(quat_corrected); % reconstruit les axes à partir du quaternion final
% % Ajout dans le plot :
% figure;
% hold on; grid on; axis equal;
% title('Orientation de la plaque de force : Calcul vs Nexus');
% xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
% plot3(X, Y, Z, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
% for i = 1:length(X)
% text(X(i), Y(i), Z(i), sprintf(' M%d', i), 'FontSize', 10);
% end
% plot3(centre(1), centre(2), centre(3), 'r*', 'MarkerSize', 10);
% text(centre(1), centre(2), centre(3), ' Centre', 'Color', 'r', 'FontSize', 10);
% scale = 100;
% origin = centre;
% % Axes calculés par le code
% quiver3(origin(1), origin(2), origin(3), R_corrected(1,1)*scale, R_corrected(2,1)*scale, R_corrected(3,1)*scale, 'r', 'LineWidth', 2); % X
% quiver3(origin(1), origin(2), origin(3), R_corrected(1,2)*scale, R_corrected(2,2)*scale, R_corrected(3,2)*scale, 'g', 'LineWidth', 2); % Y
% quiver3(origin(1), origin(2), origin(3), R_corrected(1,3)*scale, R_corrected(2,3)*scale, R_corrected(3,3)*scale, 'b', 'LineWidth', 2); % Z
% % Axes reconstruits à partir du quaternion (version "Nexus") en ligne pointillée
% quiver3(origin(1), origin(2), origin(3), R_from_quat(1,1)*scale, R_from_quat(2,1)*scale, R_from_quat(3,1)*scale, '--r', 'LineWidth', 2); % X'
% quiver3(origin(1), origin(2), origin(3), R_from_quat(1,2)*scale, R_from_quat(2,2)*scale, R_from_quat(3,2)*scale, '--g', 'LineWidth', 2); % Y'
% quiver3(origin(1), origin(2), origin(3), R_from_quat(1,3)*scale, R_from_quat(2,3)*scale, R_from_quat(3,3)*scale, '--b', 'LineWidth', 2); % Z'
% legend({'Marqueurs', 'Centre', ...
% 'X code', 'Y code', 'Z code', ...
% 'X Nexus', 'Y Nexus', 'Z Nexus'}, ...
% 'Location', 'bestoutside');
% view(3);
