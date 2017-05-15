clc; clear;

data = csvread('~/hte_front.csv')

robot_points = data(:,4:6);
camera_points = data(:,1:3);

tmpData = csvread('~/20170515-103406.csv')

%
scatter3(camera_points(:,1),camera_points(:,2),camera_points(:,3));
hold on
scatter3(tmpData(:,1),tmpData(:,2),tmpData(:,3),'r');
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

tmp = norm(P);
 
%   Test with the point at the edge of the table
p1=[-0.317222072079;-0.487509135996;-1.85257493885; 1];
%p1 = [0.181131055917;0.37173324503;2.15828502449; 1];
%p2 = [-0.111626661713;0.366865480386;2.147308153; 1];

base1 = [-0.108598508529;0.362635345725;2.15571716553;1]
base2 = [0.170876508554;0.357103984272;2.07343476951;1]
base3 = [0.170339748384;0.206139561715;2.22899039759;1]

norm(T*p1) - tmp

norm(T*base1) - tmp
norm(T*base2) - tmp
norm(T*base3) - tmp






















