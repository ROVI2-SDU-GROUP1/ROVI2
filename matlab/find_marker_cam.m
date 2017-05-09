clc; clear;

data = csvread('~/TEST_hand_to_eye.csv');



robot_points = data(:,1:3);
camera_points = data(:,4:6);
%%
scatter3(camera_points(:,1),camera_points(:,2), camera_points(:,3));
%%
clc; 
assert( length(robot_points) == length(camera_points) );

% Calculate centroids
 mu_base = sum(robot_points)/length(robot_points);
 mu_camera = sum(camera_points)/length(camera_points);

 H = zeros(3);
 
 for i = 1:length(robot_points)
    H = H + (robot_points(i,:) - mu_base)'*(camera_points(i,:) - mu_camera);
 end
 
 [U,S,V] = svd(H);
 
 R = V*U';
 
 if det(R) < 0
     R(:,3) = R(:,3)*-1;
 end
 
 P = - R*mu_base' + mu_camera';
 norm(P)