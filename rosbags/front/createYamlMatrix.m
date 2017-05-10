clc; format short eng

stereoParams = calibrationSession.CameraParameters;
R = stereoParams.RotationOfCamera2;
T = stereoParams.TranslationOfCamera2;

Transformation2 = [R T'; 0 0 0 1];
Intrinsic2 = stereoParams.CameraParameters2.IntrinsicMatrix';
P2 = [Intrinsic2 [0 0 0]'] * Transformation2;
RadialDistortion2 = calibrationSession.CameraParameters.CameraParameters2.RadialDistortion;

Transformation1 = eye(4);
Intrinsic1 = stereoParams.CameraParameters1.IntrinsicMatrix';
P1 = [Intrinsic1 [0 0 0]'] * Transformation1;
RadialDistortion1 = calibrationSession.CameraParameters.CameraParameters1.RadialDistortion;


% Generate right
[height  width ] = size(Intrinsic2);
fprintf("CameraMatrix left:\n");
fprintf("data: [");
for i = 1:height
    for j = 1:width
        fprintf("%f,", Intrinsic2(i,j));
    end
end
fprintf("]\n\n");

% Generate right
[height  width ] = size(Intrinsic2);
fprintf("CameraMatrix left:\n");
fprintf("data: [");
for i = 1:height
    for j = 1:width
        fprintf("%f,", Intrinsic2(i,j));
    end
end
fprintf("]\n\n");


% Generate distortion right
fprintf("Radial distortion left:\n");
fprintf("data: [");
for i = 1:length(RadialDistortion1);
    fprintf("%f,", RadialDistortion1(i));
end
fprintf("]\n\n");


% Generate distortion left
fprintf("Radial distortion left:\n");
fprintf("data: [");
for i = 1:length(RadialDistortion1);
    fprintf("%f,", RadialDistortion1(i));
end
fprintf("]\n\n");

% Generate right
[height  width ] = size(P1);
fprintf("projection_matrix left:\n");
fprintf("data: [");
for i = 1:height
    for j = 1:width
        fprintf("%f,", P1(i,j));
    end
end
fprintf("]\n\n");

% Generate right
[height  width ] = size(P2);
fprintf("projection_matrix right:\n");
fprintf("data: [");
for i = 1:height
    for j = 1:width
        fprintf("%f,", P2(i,j));
    end
end
fprintf("]\n");
