function [ N ] = get3dLinag( left, right, camera )
%GET3DLINAG Summary of this function goes here
%   Detailed explanation goes here

    load(sprintf('/home/mneerup/SDU/Skole/8.Semester/ROVI/code/src/rovi2_development/calibrations/%s/%s_calib.mat', camera, camera))

    stereoParams = stereoParameters(calibrationSession.CameraParameters.CameraParameters1,calibrationSession.CameraParameters.CameraParameters2,calibrationSession.CameraParameters.RotationOfCamera2,calibrationSession.CameraParameters.TranslationOfCamera2);

    Pl = [stereoParams.CameraParameters1.IntrinsicMatrix' zeros(3,1)] * eye(4);
    Pr = [stereoParams.CameraParameters2.IntrinsicMatrix' zeros(3,1)] * [stereoParams.RotationOfCamera2' stereoParams.TranslationOfCamera2'; zeros(1,3) 1];


    N = [];
    A = []; b = []; 
    for j = 1:2 %   Increase to 4 if there are two bumble bees (4 cameras)
        if j == 1
            P = Pl;
            u = left(1);
            v = left(2);
        else
            P = Pr;
            u = right(1);
            v = right(2);
        end

        Q1 = P(1,1:3);  q14 = P(1,4);
        Q2 = P(2,1:3);  q24 = P(2,4);
        Q3 = P(3,1:3);  q34 = P(3,4);

        A = [A; Q1 - u * Q3; Q2 - v * Q3];
        b = [b; u * q34 - q14; v * q34 - q24];

    end
    N = [N pinv(A' * A) * A' *b];
end

