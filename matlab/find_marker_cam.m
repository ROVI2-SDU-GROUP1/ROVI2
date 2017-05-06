clc; clear;
robot_points = [];
camera_points = [];

assert( length(robot_points) == length(camera_points) );

% Calculate centroids
 mu_base = sum(robot_points)/length(robot_points);
 mu_cam = sum(camera_points)/length(camera_points);

 H = [];
 
 for i = 1:length(robot_points)
    H = H + (robot_points(i) - mu_base)*(camera_points(i) - my_camera)';
 end
 [U,S,V] = svd(H);
 
 R = V*U';
 
 if det(R) < 0
     R(:,3) = R(:,3)*-1;
 end
 P = - R*mu_base + mu_cam