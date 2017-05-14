clc; format short g;

% ------------- Right camera ----------------------- %
stereoParams = calibrationSession.CameraParameters;
R = stereoParams.RotationOfCamera2;
T = stereoParams.TranslationOfCamera2;
Kr = [1280.99658023138, -0, 514.932502746582; -0, 1280.99658023138, 387.48673248291; 0, 0, 1];
PR = cameraMatrix(stereoParams.CameraParameters2,R,T)';

RadialDistortionR = calibrationSession.CameraParameters.CameraParameters2.RadialDistortion;
TangentialR = calibrationSession.CameraParameters.CameraParameters2.TangentialDistortion;
DistortionR = [RadialDistortionR(1:2) TangentialR(1:2) RadialDistortionR(3)];


% Generate right CameraMatrix
[height  width ] = size(Kr);
fprintf("CameraMatrix right:\n");
fprintf("data: [");
for i = 1:height
    for j = 1:width
        fprintf("%f,", Kr(i,j));
    end
end
fprintf("]\n\n");

% Generate right distortion
fprintf("Distortion right:\n");
fprintf("data: [");
for i = 1:length(DistortionR);
    fprintf("%f,", DistortionR(i));
end
fprintf("]\n\n");


% Generate right projection matrix
[height  width ] = size(PR);
fprintf("projection_matrix right:\n");
fprintf("data: [");
for i = 1:height
    for j = 1:width
        fprintf("%f,", PR(i,j));
    end
end
fprintf("]\n\n");


% ------------- Left camera ----------------------- %
T = [0 0 0]';
R = eye(3);
IntrinsicL = [1280.99658023138, -0, 521.351615905762; -0, 1280.99658023138, 387.48673248291; 0, 0, 1];
%PL = [IntrinsicL [0 0 0]'] * TransformationL;
PL = cameraMatrix(stereoParams.CameraParameters1,R,T)';

RadialDistortionL = calibrationSession.CameraParameters.CameraParameters1.RadialDistortion;
TangentialL = calibrationSession.CameraParameters.CameraParameters1.TangentialDistortion;
DistortionL = [RadialDistortionL(1:2) TangentialL(1:2) RadialDistortionL(3)];

% Generate left CameraMatrix
[height  width ] = size(IntrinsicL);
fprintf("CameraMatrix left:\n");
fprintf("data: [");
for i = 1:height
    for j = 1:width
        fprintf("%f,", IntrinsicL(i,j));
    end
end
fprintf("]\n\n");


% Generate left distortion
fprintf("Distortion left:\n");
fprintf("data: [");
for i = 1:length(DistortionL);
    fprintf("%f,", DistortionL(i));
end
fprintf("]\n\n");


% Generate left projection matrix
[height  width ] = size(PL);
fprintf("projection_matrix left:\n");
fprintf("data: [");
for i = 1:height
    for j = 1:width
        fprintf("%f,", PL(i,j));
    end
end
fprintf("]\n");
