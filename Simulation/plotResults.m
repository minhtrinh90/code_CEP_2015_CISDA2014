function plotResults(p1,p2,p3,p4,errAngle,t)
pA = [ 10;  0]; 
pB = [ 0;  0]; 
pC = [ 0;  10];

figure(1)
t_end = length(t);

delete(subplot(1,2,1))
subplot(1,2,1)
hold on
text(-1.2,-0.4,'B','FontSize', 14');
text(10.5,-0.5,'A','FontSize', 14');
text(-1,11,'C','FontSize', 14');
plot(pA(1,1), pA(2,1), 'ok', pB(1,1), pB(2,1), 'ok', pC(1,1), pC(2,1), 'ok')
line([pA(1,1), pB(1,1)],[pA(2,1), pB(2,1)],'LineStyle','-','Color','k', 'LineWidth', 2);
line([pC(1,1), pB(1,1)],[pC(2,1), pB(2,1)],'LineStyle','-','Color','k', 'LineWidth', 2);
line([pA(1,1), pC(1,1)],[pA(2,1), pC(2,1)],'LineStyle','-','Color','k', 'LineWidth', 2);

% First trajectory
plot(p1(1,1), p1(2,1), 'xg');
plot(p1(1,t_end), p1(2,t_end), 'og');
plot(p1(1,1:t_end), p1(2,1:t_end), 'g', 'LineWidth', 1);
line([pA(1,1), p1(1,t_end)],[pA(2,1), p1(2,t_end)],'LineStyle','--','Color','k', 'LineWidth', 0.5);
line([p1(1,t_end), pB(1,1)],[p1(2,t_end), pB(2,1)],'LineStyle','--','Color','k', 'LineWidth', 0.5);
line([p1(1,t_end), pC(1,1)],[p1(2,t_end), pC(2,1)],'LineStyle','--','Color','k', 'LineWidth', 0.5);

% Second trajectory
plot(p2(1,1), p2(2,1), 'xr');
plot(p2(1,t_end), p2(2,t_end), 'or');
plot(p2(1,1:t_end), p2(2,1:t_end), 'r', 'LineWidth', 1);
line([pA(1,1), p2(1,t_end)],[pA(2,1), p2(2,t_end)],'LineStyle','--','Color','k', 'LineWidth', 0.5);
line([p2(1,t_end), pB(1,1)],[p2(2,t_end), pB(2,1)],'LineStyle','--','Color','k', 'LineWidth', 0.5);
line([p2(1,t_end), pC(1,1)],[p2(2,t_end), pC(2,1)],'LineStyle','--','Color','k', 'LineWidth', 0.5);

% Third trajectory
plot(p3(1,1), p3(2,1), 'xb');
plot(p3(1,t_end), p3(2,t_end), 'ob');
plot(p3(1,1:t_end), p3(2,1:t_end), 'b', 'LineWidth', 1);
line([pA(1,1), p3(1,t_end)],[pA(2,1), p3(2,t_end)],'LineStyle','--','Color','k', 'LineWidth', 0.5);
line([p3(1,t_end), pB(1,1)],[p3(2,t_end), pB(2,1)],'LineStyle','--','Color','k', 'LineWidth', 0.5);
line([p3(1,t_end), pC(1,1)],[p3(2,t_end), pC(2,1)],'LineStyle','--','Color','k', 'LineWidth', 0.5);

% Fourth trajectory
plot(p4(1,1), p4(2,1), 'xm');
plot(p4(1,t_end), p4(2,t_end), 'om');
plot(p4(1,1:t_end), p4(2,1:t_end), 'm', 'LineWidth', 1);
line([pA(1,1), p4(1,t_end)],[pA(2,1), p4(2,t_end)],'LineStyle','--','Color','k', 'LineWidth', 0.5);
line([p4(1,t_end), pB(1,1)],[p4(2,t_end), pB(2,1)],'LineStyle','--','Color','k', 'LineWidth', 0.5);
line([p4(1,t_end), pC(1,1)],[p4(2,t_end), pC(2,1)],'LineStyle','--','Color','k', 'LineWidth', 0.5);
box on;
xlabel('x');
ylabel('y');
axis equal
xlim([-5,18])
ylim([-5,18])
grid on

delete(subplot(1,2,2))
subplot(1,2,2)
hold on
plot(t, errAngle(1,:), '-g', 'LineWidth', 1.25);
plot(t, errAngle(2,:), '-r', 'LineWidth', 1.25);
plot(t, errAngle(3,:), '-b', 'LineWidth', 1.25);
plot(t, errAngle(4,:), '-m', 'LineWidth', 1.25);
legend('$e(t)=\sum_{i=1}^3|\alpha_i - \alpha_i^*|$', 'FontSize', 18, 'Interpreter', 'latex')
ylim([0,5]);
xlabel('Time t [s]'); 
ylabel('Total angle errors e [rad]');
grid on;
box on;
end