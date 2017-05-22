clc; clear; close all;
warning('off','all')

camera = 'front';
load(sprintf('/home/mneerup/SDU/Skole/8.Semester/ROVI/code/src/rovi2_development/calibrations/%s/%s_calib.mat', camera, camera))
stereoParams = stereoParameters(calibrationSession.CameraParameters.CameraParameters1,calibrationSession.CameraParameters.CameraParameters2,calibrationSession.CameraParameters.RotationOfCamera2,calibrationSession.CameraParameters.TranslationOfCamera2);

format longG; format compact;

images = [60 80 100];
for k = 1:length(images)

    imageLeft = imread(sprintf('~/Billeder/%s/left-%d.png', camera,images(k)));
    imageRight = imread(sprintf('~/Billeder/%s/right-%d.png', camera,images(k)));
    imageLeft = undistortImage(imageLeft, stereoParams.CameraParameters1);
    imageRight = undistortImage(imageRight, stereoParams.CameraParameters2);

    [imagePoints,boardSize,pairsUsed] = detectCheckerboardPoints(imageLeft,imageRight);
    
    %figure
    %imshow(imageLeft);
    %hold on
    %plot(imagePoints(:,1,1),imagePoints(:,2,1),'y.','MarkerSize',20);
    %plot(imagePoints(:,1,1),imagePoints(:,2,1),'yx','MarkerSize',20);
    %set(gca,'position',[0 0 1 1],'units','normalized')

    Points = [];
    for i = 1:length(imagePoints)
        % ACHTUNG ACHTUNG, use the following two lines if using
        %back/{left,right}-100.png
        %leftPoint = [imagePoints(49 - i,1); imagePoints(49 - i,2)];
        %rightPoint = [imagePoints(i,3); imagePoints(i,4)];
        leftPoint = [imagePoints(i,1); imagePoints(i,2)];
        rightPoint = [imagePoints(i,3); imagePoints(i,4)];
        Point = get3dLinag(leftPoint, rightPoint,camera);
        Points = [Points Point];
    end

    x = Points(1,:);
    y = Points(2,:);
    z = Points(3,:);
    %figure
    %scatter3(x,y,z,'rx');

    %figure;

    dist = [];
    for i = 1:6
        i1 = i*1 + 42;
        i2 = i*1;
        %scatter3(x(i1),y(i1),z(i1),'rx');
        %hold on;
        %scatter3(x(i2),y(i2),z(i2),'bx');
        %x(i2);
        %x(i1);
        
        
        disttmp = norm([x(i2) - x(i1); y(i2) - y(i1); z(i2) - z(i1)]);
        dist = [dist disttmp];
    end
    normDist = norm([x(1), y(1), z(1)]);

    
    my = mean(dist);
    R = var(dist);
    dist;
    fprintf('Image: %d, mean: %f, var: %f, normDist: %f\n\n', images(k), my, R, normDist);
end