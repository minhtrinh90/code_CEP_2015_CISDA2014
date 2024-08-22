%% Navigation main mfile
clear all;
clc;
% Setup parameters
init;
% simulation without noises
 for ii = 1:t_end-1
    
% Calculate new parameters 
    zz(:,ii) = z;
    p_t = p1(:,ii);
    slow_loop1;
    p1(:,ii+1) = [x;y];
 end

% Trajectories
figure(1)
hold on
text(10.3,0,'B_1','FontSize', 14,'Color','b');
text(-0.3,-0.3,'B_2','FontSize', 14,'Color','b');
text(0,10.3,'B_3','FontSize', 14,'Color','b');

% plot(pA(1,1:t_end), pA(2,1:t_end), 'm', pB(1,1:t_end), pB(2,1:t_end), 'b', pC(1,1:t_end), pC(2,1:t_end), 'g', pA(1,t_end), pA(2,t_end), 'om', pB(1,t_end), pB(2,t_end), 'ob', pC(1,t_end), pC(2,t_end), 'og',pA(1,1), pA(2,1), '*m', pB(1,1), pB(2,1), '*b', pC(1,1), pC(2,1), '*g')
plot(pA(1,1), pA(2,1), 'ok', pB(1,1), pB(2,1), 'ok', pC(1,1), pC(2,1), 'ok')

m2 = plot(p1(1,1), p1(2,1), 'og');
m1 = plot(p1(1,t_end), p1(2,t_end), 'sr');
m3 = plot(p1(1,1:t_end), p1(2,1:t_end), '-r', 'LineWidth', 2);
grid on

line([pA(1,1), pB(1,1)],[pA(2,1), pB(2,1)],'LineStyle','--','Color','k', 'LineWidth', 2);
line([pC(1,1), pB(1,1)],[pC(2,1), pB(2,1)],'LineStyle','--','Color','k', 'LineWidth', 2);
line([pA(1,1), pC(1,1)],[pA(2,1), pC(2,1)],'LineStyle','--','Color','k', 'LineWidth', 2);

% line([pA(1,1), p1(1,t_end)],[pA(2,1), p1(2,t_end)],'LineStyle','--','Color','m', 'LineWidth', 1);
% line([p1(1,t_end), pB(1,1)],[p1(2,t_end), pB(2,1)],'LineStyle','--','Color','m', 'LineWidth', 1);
% line([p1(1,t_end), pC(1,1)],[p1(2,t_end), pC(2,1)],'LineStyle','--','Color','m', 'LineWidth', 1);

xlabel('x [m]','FontSize', 14);ylabel('y [m]','FontSize', 14);
title('Trajectory of the quadcopter','FontSize', 14)
% m = legend([m2 m1 m3], 'Init. position', 'Final position', 'Noised Trajectory');
axis equal
disp('Final bearing angles:');
% rad2deg([angleA angleB angleC])
rad2deg([angle1A angle1B angle1C])

figure(2)
hold on
l1 = plot(0.3*[1:t_end-1],zz(1,1:t_end-1),'LineWidth', 2);
title( 'Altitude of the quadrotor','FontSize', 14)
grid on
xlabel( 'time [s]','FontSize', 14)
ylabel( 'altitude [m]','FontSize', 14)

%%%%%%%%%%%%% simulation with noise
init;
 for ii = 1:t_end-1
    
% Calculate new parameters 
    zz(:,ii) = z;
    p_t = p1(:,ii);
    slow_loop;
    p1(:,ii+1) = [x;y];
 end
 
figure(1)
hold on
m4 = plot(p1(1,1:t_end), p1(2,1:t_end), '-g', 'LineWidth', 2);
m1 = plot(p1(1,t_end), p1(2,t_end), 'sg');
line([pA(1,1), p1(1,t_end)],[pA(2,1), p1(2,t_end)],'LineStyle','--','Color','b', 'LineWidth', 1);
line([p1(1,t_end), pB(1,1)],[p1(2,t_end), pB(2,1)],'LineStyle','--','Color','b', 'LineWidth', 1);
line([p1(1,t_end), pC(1,1)],[p1(2,t_end), pC(2,1)],'LineStyle','--','Color','b', 'LineWidth', 1);
m = legend([m2 m1 m4 m3], 'Init. position', 'Final position', 'Traj.', 'Traj. w/ noises,wind');

grid on

figure(2)
hold on
l2 = plot(0.3*[1:t_end-1],zz(1,1:t_end-1),'-g','LineWidth', 2);
l = legend([l1 l2], 'Altitude w/ noises,wind','Altitude');
grid on
