back_data = csvread('3dpose_back.log');
back_x = back_data(:,1);
back_y = back_data(:,2);
back_z = back_data(:,3);

front_data = csvread('3dpose_front.log');
front_x = front_data(:,1);
front_y = front_data(:,2);
front_z = front_data(:,3);


figure
scatter3(back_x, back_y, back_z, 'ro');
hold on
scatter3(front_x, front_y, front_z,'bo');
title('Front and back positions of same tracked object');
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');


subplot(2,2,1)
scatter3(back_x, back_y, back_z, 'ro');
hold on
scatter3(front_x, front_y, front_z,'bo');
grid on
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');



subplot(2,2,2)
scatter(back_x, back_y, 'ro');
hold on
scatter(front_x, front_y, 'bo')
grid on
xlabel('X [m]');
ylabel('Y [m]');


subplot(2,2,4)
scatter(back_x, back_z, 'ro');
hold on
scatter(front_x, front_z, 'bo')
grid on
xlabel('X [m]');
ylabel('Z [m]');


subplot(2,2,3)
scatter(back_y, back_z, 'ro');
hold on
scatter(front_y, front_z, 'bo')
grid on
xlabel('Y [m]');
ylabel('Z [m]');

