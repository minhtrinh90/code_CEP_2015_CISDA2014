%% Program to import positions of agents and plot
clear all;
clc;
% Import data from excel file
excelName = 'result_00091714_5834.xlsx';
excelSheet = 'Sheet1';
p1_x = xlsread(excelName,excelSheet,'A:A')/100;
p1_y = xlsread(excelName,excelSheet,'B:B')/100;
p2_x = xlsread(excelName,excelSheet,'C:C')/100;
p2_y = xlsread(excelName,excelSheet,'D:D')/100;
p3_x = xlsread(excelName,excelSheet,'E:E')/100;
p3_y = xlsread(excelName,excelSheet,'F:F')/100;
p4_x = xlsread(excelName,excelSheet,'G:G')/100;
p4_y = xlsread(excelName,excelSheet,'H:H')/100;
% p5_x = xlsread('result_00090114_1854_filter.xlsx','Sheet1','I:I');
% p5_y = xlsread('result_00090114_1854_filter.xlsx','Sheet1','J:J');
% p6_x = xlsread('result_00090114_1854_filter.xlsx','Sheet1','K:K');
% p6_y = xlsread('result_00090114_1854_filter.xlsx','Sheet1','L:L');
% p7_x = xlsread('result_00090114_1854_filter.xlsx','Sheet1','M:M');
% p7_y = xlsread('result_00090114_1854_filter.xlsx','Sheet1','N:N');
min_x = min([p1_x(1) p2_x(1) p3_x(1) p4_x(1)]);
min_y = min([p1_y(1) p2_y(1) p3_y(1) p4_y(1)]);
max_y = max([p1_y(1),p2_y(1),p3_y(1),p4_y(1)]);
max_x = max([p1_x(1),p2_x(1),p3_x(1),p4_x(1)]);

% Auxiliary variables
t = length(p1_x);
min = [ones(t,1)*min_x ones(t,1)*min_y];
% p1_x = p1_x - min_x;
% p2_x = p2_x - min_x;
% p3_x = p3_x - min_x;
% p4_x = p4_x - min_x;
% p1_y = p1_y - min_y;
% p2_y = p2_y - min_y;
% p3_y = p3_y - min_y;
% p4_y = p4_y - min_y;

p1 = [p1_x p1_y];
p2 = [p2_x p2_y];
p3 = [p3_x p3_y];
p4 = [p4_x p4_y];
errAngle = zeros(3, t);

%% Plot trajectory
figure(1)
hold on;

plot(p1_x(1), p1_y(1), 'ok', p2_x(1), p2_y(1), 'og', p3_x(1), p3_y(1), 'ob');
plot(p1_x(t), p1_y(t), 'sk', p2_x(t), p2_y(t), 'sg', p3_x(t), p3_y(t), 'sb');
plot(p2_x(1:t), p2_y(1:t), '-g', 'LineWidth', 2);
plot(p3_x(1:t), p3_y(1:t), '-b', 'LineWidth', 2);
plot(p1_x(1:t), p1_y(1:t), '-k', 'LineWidth', 2);

m1 = plot(p4_x(1), p4_y(1), 'or', 'LineWidth', 1);
m2 = plot(p4_x(t), p4_y(t), 'sr', 'LineWidth', 1);
m3 = plot(p4_x(1:t), p4_y(1:t), '-r', 'LineWidth', 2);
line([p1(t,1), p2(t,1)],[p1(t,2), p2(t,2)],'LineStyle','--','Color','k');
line([p3(t,1), p2(t,1)],[p3(t,2), p2(t,2)],'LineStyle','--','Color','k');
line([p1(t,1), p3(t,1)],[p1(t,2), p3(t,2)],'LineStyle','--','Color','k');

line([p1(t,1), p4(t,1)],[p1(t,2), p4(t,2)],'LineStyle','--','Color','k');
line([p4(t,1), p2(t,1)],[p4(t,2), p2(t,2)],'LineStyle','--','Color','k');
line([p4(t,1), p3(t,1)],[p4(t,2), p3(t,2)],'LineStyle','--','Color','k');
title('Trajectory of the autonomous agent');
xlabel('x [m]');
ylabel('y [m]');
m = legend([m1 m2 m3], 'Init. position', 'Final position', 'Trajectory');
grid on
 axis 'equal'
%% Plot angle dynamics
a1_d=[0;1];b1_d=[-.5;sqrt(3)/2];c1_d=[.0;sqrt(3)/2]; J = [0 -1;1 0];
% Calculate the angles based on data
for i = 1:t
    
a1 = (p1(i,:) - p4(i,:))'/norm(p1(i,:) - p4(i,:));
b1 = (p2(i,:) - p4(i,:))'/norm(p2(i,:) - p4(i,:));
c1 = (p3(i,:) - p4(i,:))'/norm(p3(i,:) - p4(i,:));

angle1C = findAngle1(a1(1,1),a1(2,1),b1(1,1),b1(2,1));
angle1A = findAngle1(b1(1,1),b1(2,1),c1(1,1),c1(2,1));
angle1B = findAngle1(c1(1,1),c1(2,1),a1(1,1),a1(2,1));

u4 = -((a1_d'*(J*a1))*(J*a1) + (b1_d'*(J*b1))*(J*b1) + (c1_d'*(J*c1))*(J*c1));

errAngle(:,i) = rad2deg([angle1A, angle1B, angle1C])';

end

% Plot
figure(2)
hold on
k1=plot([0:t-1]*0.04, errAngle(1,1:t), '-b', 'LineWidth', 2 );
k2=plot([0:t-1]*0.04, errAngle(2,1:t), '--k', 'LineWidth', 2 );
k3=plot([0:t-1]*0.04, errAngle(3,1:t), '-.m', 'LineWidth', 2 );
h = legend([k1 k2 k3],'$e_1$','$e_2$','$e_3$');
title('Angle');
xlabel('time [s]'); 
ylabel('angle [deg]');
grid on;
set(h,'Interpreter','latex');
rad2deg([angle1A angle1B angle1C])