%%  Load calibration
clc;clear;
format shortG; format compact;

load('calibrationSession_2coeff.mat')
stereoParams = stereoParameters(calibrationSession.CameraParameters.CameraParameters1,calibrationSession.CameraParameters.CameraParameters2,calibrationSession.CameraParameters.RotationOfCamera2,calibrationSession.CameraParameters.TranslationOfCamera2);

FILENAME_IMG_LEFT = '/tmp/img_left.png';
FILENAME_IMG_RIGHT = '/tmp/img_right.png';

FILENAME_IMG_LEFT = '/tmp/back/left/left-2.png';
FILENAME_IMG_RIGHT = '/tmp/back/right/right-2.png';

IMG_LEFT    = imread(FILENAME_IMG_LEFT);
IMG_RIGHT   = imread(FILENAME_IMG_RIGHT);

[IMG_LEFT] = undistortImage(IMG_LEFT, stereoParams.CameraParameters1);
[IMG_RIGHT] = undistortImage(IMG_RIGHT, stereoParams.CameraParameters2);

%   Create projection matrices
%tmp_in_l = [1358.800297, 0.000000, 507.324958; 0.000000, 1357.240671, 361.199233; 0.000000, 0.000000, 1.000000];
%tmp_in_r = [1354.587959, 0.000000, 528.883068; 0.000000, 1354.339782, 450.942605; 0.000000, 0.000000, 1.000000];
%Pl = [cams(1).intrinsic zeros(3,1)] * cams(1).transformation;
%Pr = [cams(2).intrinsic zeros(3,1)] * cams(2).transformation;
Pl_ours = [stereoParams.CameraParameters1.IntrinsicMatrix' zeros(3,1)] * eye(4);
Pr_ours = [stereoParams.CameraParameters2.IntrinsicMatrix' zeros(3,1)] * [stereoParams.RotationOfCamera2' stereoParams.TranslationOfCamera2'; zeros(1,3) 1];

Pl = Pl_ours;
Pr = Pr_ours;

%Pl = [tmp_in_l zeros(3,1)] * cams(1).transformation;
%Pr = [tmp_in_r zeros(3,1)] * cams(2).transformation;

%Pr =  [1501.257140, 0.000000, 493.606922, -120.645110; 0.000000, 1501.257140, 410.707668, 0.000000; 0.000000, 0.000000, 1.000000, 0.000000];
%Pl = [1501.257140, 0.000000, 493.606922, 0.000000; 0.000000, 1501.257140, 410.707668, 0.000000; 0.000000, 0.000000, 1.000000, 0.000000];

PXl = Pl(:,1:3);
pxl = Pl(:,4);
PXr = Pr(:,1:3);
pxr = Pr(:,4);

%   Center of cameras
Cl = [-eye(3) * zeros(3,1); 1];
Cr = [-stereoParams.RotationOfCamera2 * stereoParams.TranslationOfCamera2'; 1];

%   Compute Epipole (e = P * C)
er = Pr*Cl; 


%******************************************************************** 
% DETECT CLICKED IMAGE COORDINATE IN LEFT IMAGE
%******************************************************************** 
figure(2)
subplot(1,2,1)
hold off
imshow(IMG_LEFT);

% Getting where you click on the image.
% Left click for additional points comming
% Right click for last point
hold on
[x_left,y_left] = getpts(); % pixel in hom coordinates
hold off

% Draws colored crosses at the locations you clicked at
% (after you finished clicking)
hold on
for i = 1:size(x_left)
    plot([x_left(i)-3 x_left(i)+3],[y_left(i)   y_left(i)],'Color','r','LineWidth', 1)
    plot([x_left(i)   x_left(i)],  [y_left(i)-3 y_left(i)+3],'Color','r','LineWidth', 1)
    plot([x_left(i)-3 x_left(i)+3],[y_left(i)-3 y_left(i)+3],'Color','b','LineWidth', 1)
    plot([x_left(i)-3 x_left(i)+3],[y_left(i)+3 y_left(i)-3],'Color','b','LineWidth', 1)
end;
hold off

subplot(1,2,2)
hold off
imshow(IMG_RIGHT);
Minfl = [];
Minfr = [];
for i = 1:size(x_left)
    %   Compute infinity points
    m = [x_left(i);y_left(i);1];
    
    Minfl = [Minfl [inv(PXl)*m; 0]];
    Minfr = [Minfr [Pr*Minfl(:,i)]];

    %   Normalize stuff
    if (er(3) ~= 0.0) % Not equal
        er_norm = er/er(3);
    else
        er_norm = er/10e-50;
    end
    
    %   Compute epipolar lines (l = e_2 x m_l, inf)
    L = cross(er_norm(1:3),Minfr(1:3,i));

    h = 768;
    w = 1024;

    x1 = 0;
    y1 = - L(3)/L(2);

    x2 = w;
    y2 = -L(1)/L(2)*w-L(3)/L(2);

    %********************************************************************
    % DISPLAY LINES IN RIGHT IMAGE
    %********************************************************************
    hold on
    plot([x1 x2],[y1 y2],'Color','g','LineWidth', 1);
    hold off
end

%********************************************************************
% DETECT CLICKED IMAGE COORDINATE IN RIGHT IMAGE 
%********************************************************************
hold on
[x_right,y_right] = getpts(); % pixel in hom coordinates
hold off

M = [];
for i = 1:size(x_right)
    mr = [x_right(i);y_right(i);1]
    
    Minfr = inv(PXr)*mr;

    %   Compute plugger lines
    %   Leftclick line
    mu1 = cross(Cl(1:3),Minfl(1:3,i)) / norm(Minfl(1:3,i));
    nu1 = Minfl(1:3,i)/norm(Minfl(1:3,i));

    %   Rightclick line 
    mu2 = cross(Cr(1:3),Minfr(1:3)) / norm(Minfr(1:3));
    nu2 = Minfr(1:3)/norm(Minfr(1:3));
    
    %********************************************************************
    % COMPUTE INTERSECTION OF LINES
    %********************************************************************
    M1 = ((nu1 * cross(nu2,mu2)' - (nu1 * nu2') * nu1 * cross(nu2,mu1)') / (norm(cross(nu1,nu2))^2)) * nu1 + cross(nu1,mu1);
    M2 = ((nu2 * cross(nu1,mu1)' - (nu2 * nu1') * nu2 * cross(nu1,mu2)') / (norm(cross(nu2,nu1))^2)) * nu2 + cross(nu2,mu2);

    M = [M M1 + (M2 - M1)/2];
end

fprintf('*********************\n')
fprintf('Epipolar method')
fprintf('\n*********************\n\n')
fprintf('3D points in image:\n')
M
    %********************************************************************
    % Output distances to points in image
    %********************************************************************
M_norm = [];
fprintf('Norm distance from Camera to points:\n')
for i = 1:size(M,2)
    M_norm = [M_norm norm(M(:,i))];
end
M_norm

fprintf('Distance between point "i" and i+1:\n')
M_distance = [];
for i = 1:size(M,2)-1
    M_distance = [M_distance norm( M(:,i) - M(:,i+1) )];
    
end
M_distance

%********************************************************************
% Control test with Linear method
%********************************************************************

N = [];
for i = 1:size(x_left)
    A = []; b = []; 
    for j = 1:2 %   Increase to 4 if there are two bumble bees (4 cameras)
        if j == 1
            P = Pl;
            u = x_left(i);
            v = y_left(i);
        else
            P = Pr;
            u = x_right(i);
            v = y_right(i);
        end

        Q1 = P(1,1:3);  q14 = P(1,4);
        Q2 = P(2,1:3);  q24 = P(2,4);
        Q3 = P(3,1:3);  q34 = P(3,4);

        A = [A; Q1 - u * Q3; Q2 - v * Q3];
        b = [b; u * q34 - q14; v * q34 - q24];

    end
    N = [N pinv(A' * A) * A' *b];
end
fprintf('\n*********************\n')
fprintf('LINEAR ALG')
fprintf('\n*********************\n\n')
fprintf('3D points in image:\n')
N
    %********************************************************************
    % Output distances to points in image
    %********************************************************************
N_norm = [];
fprintf('Norm distance from Camera to points:\n')
for i = 1:size(M,2)
    N_norm = [N_norm norm(N(:,i))];
end
N_norm

fprintf('Distance between point "i" and i+1:\n')
N_distance = [];
for i = 1:size(M,2)-1
    N_distance = [N_distance norm( N(:,i) - N(:,i+1) )];
end
N_distance