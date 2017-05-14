clc; clear;

data = csvread('~/hte_back.csv');

robot_points = data(:,4:6);
camera_points = data(:,1:3)/1000;

%
%scatter3(camera_points(:,1),camera_points(:,2),camera_points(:,3));
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

T=[R P; 0 0 0 1]

norm(P)
 
%   Test with the point at the edge of the table
%p1=[-0.317222072079;-0.487509135996;-1.85257493885; 1];
p1 = [0.181131055917;0.37173324503;2.15828502449; 1];
p2 = [-0.111626661713;0.366865480386;2.147308153; 1];
 
norm(T*p1)