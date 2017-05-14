clear;clc
%   Load the calibration file
load('imagesRawHack/calibrationSession.mat')
stereoParams = stereoParameters(calibrationSession.CameraParameters.CameraParameters1,calibrationSession.CameraParameters.CameraParameters2,calibrationSession.CameraParameters.RotationOfCamera2,calibrationSession.CameraParameters.TranslationOfCamera2);

%   Choose filenames
filenameLeft = '/home/mneerup/SDU/Skole/8.Semester/ROVI/code/src/rovi2_development/calibrations/TEST_TEST_left_front_xXx.yaml';
filenameRight = '/home/mneerup/SDU/Skole/8.Semester/ROVI/code/src/rovi2_development/calibrations/TEST_TEST_right_front_xXx.yaml';

%   Set width and height
width = 1024;
height = 768;

%   Print to files
for i = 1:2
   if i == 1
        %   Save left camera YAML
        %       Create the matrices
        % Parameters from ROS-yaml file.
        filename = filenameLeft;
        intrinsic = stereoParams.CameraParameters1.IntrinsicMatrix;
        cameraMatrix = reshape( intrinsic,1,9);
        distortionMatrix = [stereoParams.CameraParameters1.RadialDistortion(1:2) stereoParams.CameraParameters1.TangentialDistortion(1:2) stereoParams.CameraParameters1.RadialDistortion(3)];
        rectificationMatrix = reshape( eye(3),1,9 );
        projectionMatrix = reshape( ([intrinsic' zeros(3,1)] * eye(4))', 1, 12 );
        translationVector = [0; 0; 0];
        rotationMatrix = [1 0 0; 0 1 0; 0 0 1];

   else
        %   Save right camera YAML
        %       Create the matrices
        filename = filenameRight;
        intrinsic = stereoParams.CameraParameters2.IntrinsicMatrix;
        cameraMatrix = reshape( intrinsic,1,9);
        distortionMatrix = [stereoParams.CameraParameters2.RadialDistortion(1:2) stereoParams.CameraParameters2.TangentialDistortion(1:2) stereoParams.CameraParameters2.RadialDistortion(3)];
        rectificationMatrix = reshape( eye(3),1,9 );
        projectionMatrix = reshape( ( [intrinsic' zeros(3,1)] * [stereoParams.RotationOfCamera2' stereoParams.TranslationOfCamera2'; zeros(1,3) 1] )', 1, 12);
        translationVector = stereoParams.TranslationOfCamera2;
        rotationMatrix = reshape(stereoParams.RotationOfCamera2,1,9);
   end

    fileID = fopen(filename, 'w');

    fprintf(fileID, 'image_width: %i\n', width);
    fprintf(fileID, 'image_height: %i\n', height);
    if i == 1
        fprintf(fileID, 'camera_name: narrow_stereo/left\n');
    else
        fprintf(fileID, 'camera_name: narrow_stereo/right\n');
    end
   fprintf(fileID, 'camera_matrix:\n');
    fprintf(fileID, '  rows: 3\n');
    fprintf(fileID, '  cols: 3\n');
    fprintf(fileID, '  data: ['); 
    commaSeperatedMatrix = sprintf('%f,' , cameraMatrix);
    fprintf(fileID, '%s ',commaSeperatedMatrix(1:end-1)); 
    fprintf(fileID, ']\n');

    fprintf(fileID, 'distortion_model: plumb_bob\n');
    fprintf(fileID, 'distortion_coefficients:\n');
    fprintf(fileID, '  rows: 1\n');
    fprintf(fileID, '  cols: 5\n');
    fprintf(fileID, '  data: ['); 
    commaSeperatedMatrix = sprintf('%f,' , distortionMatrix);
    fprintf(fileID, '%s ',commaSeperatedMatrix(1:end-1)); 
    fprintf(fileID, ']\n');

    fprintf(fileID, 'rectification_matrix:\n');
    fprintf(fileID, '  rows: 3\n');
    fprintf(fileID, '  cols: 3\n');
    fprintf(fileID, '  data: ['); 
    commaSeperatedMatrix = sprintf('%f,' , rectificationMatrix);
    fprintf(fileID, '%s ',commaSeperatedMatrix(1:end-1));
    fprintf(fileID, ']\n');
    
    fprintf(fileID, 'projection_matrix:\n');
    fprintf(fileID, '  rows: 3\n');
    fprintf(fileID, '  cols: 4\n');
    fprintf(fileID, '  data: [');
    commaSeperatedMatrix = sprintf('%f,' , projectionMatrix);
    fprintf(fileID, '%s ',commaSeperatedMatrix(1:end-1));
    fprintf(fileID, ']\n');
    
    
    fprintf(fileID, 'translation_vector:\n');
    fprintf(fileID, '  rows: 3\n');
    fprintf(fileID, '  cols: 1\n');
    fprintf(fileID, '  data: [');
    commaSeperatedMatrix = sprintf('%f,' , translationVector);
    fprintf(fileID, '%s ',commaSeperatedMatrix(1:end-1));
    fprintf(fileID, ']\n');
    
        
    fprintf(fileID, 'rotation_matrix:\n');
    fprintf(fileID, '  rows: 3\n');
    fprintf(fileID, '  cols: 3\n');
    fprintf(fileID, '  data: [');
    commaSeperatedMatrix = sprintf('%f,' , rotationMatrix);
    fprintf(fileID, '%s ',commaSeperatedMatrix(1:end-1));
    fprintf(fileID, ']\n');
    
    
    fprintf(fileID, 'projection_rectified_matrix:\n');
    fprintf(fileID, '  rows: 3\n');
    fprintf(fileID, '  cols: 4\n');
    fprintf(fileID, '  data: [');
    commaSeperatedMatrix = sprintf('%f,' , projectionRecticied);
    fprintf(fileID, '%s ',commaSeperatedMatrix(1:end-1));
    fprintf(fileID, ']\n');
   

    fclose(fileID);
    
end
