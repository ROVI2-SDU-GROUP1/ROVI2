clear; clc
data = csvread('data.csv',1);
x = data(:,5);
y = data(:,6);
z = data(:,7);

points = [x y z];

scatter3(x,y,z);

dist = eye(6,1);
for i = 1:6
   dist(i) = norm(points(i*8-7) - points(i*8)); 
end
dist