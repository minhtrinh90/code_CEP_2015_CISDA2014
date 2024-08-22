%% Navigation in the 3D space using only bearing angle information
% 2015.10.20
% There are 4 beacons located at 4 noncoplanar points forming a tetrahedron
% in the space
% There is one agent which desired trihedral angles are relatively defined
% with regard to beacons.
    clear all
    clc

%% Problem setup
%

% Simulation Parameters
    T_step = 0.01;                                % simulation step
    k = 1;
    tf = 4;
    t_end = 2000;
% Calculate the desired relative positions of agents in the desired formation

    
    % Initial conditions
    p1 = zeros(3, t_end); p2 = zeros(3, t_end); p3 = zeros(3, t_end); p4 = zeros(3, t_end);

    p3(:,1) = [-5; 0;0];
    p2(:,1) = [0 ; 5;0];
    p4(:,1) = [3 ;-3;0];
    p1(:,1) = [0 ; 0;4];
    
    p = zeros(3, t_end);
    %p0 = [1; 0; 1.5];
    %p0 = [-4; 0.2; 0];
    %p0 = [-.5; 1.5; 1];
    %p0 = [0; -0.3; 2.5];
    %p0 = [0; 0; 0]; 
    %p0 = [1; 2; 0]; 
    %p0 = [-2; -1; 0]; 
    %p0 = [0; 0; 3]; 
    %p0 = [-2.5; 0; 1]; 
    %p0 = [-2; 1; 0.1]; 
    %p0 = [2; -2; 0.3]; 
    p0 = [-.7; .5; 2.7];
    p(:,1) = p0;

    % Desired values
    p_d = [0; 0; 2];
    
    g1d = (p1(:,1) - p_d)/norm(p1(:,1) - p_d, 2);
    g2d = (p2(:,1) - p_d)/norm(p2(:,1) - p_d, 2);
    g3d = (p3(:,1) - p_d)/norm(p3(:,1) - p_d, 2);
    g4d = (p4(:,1) - p_d)/norm(p4(:,1) - p_d, 2);
    det([g1d' 1;g2d' 1;g3d' 1;g4d' 1])
    omega1d = findTrihedralAngle(g2d, g3d, g4d)
    omega2d = findTrihedralAngle(g1d, g4d, g3d)
    omega3d = findTrihedralAngle(g1d, g2d, g4d)
    omega4d = findTrihedralAngle(g1d, g2d, g3d)
    omega1d+omega2d+omega3d+omega4d
    % Display error
    errAngle = zeros(4, t_end);
    omega = zeros(4, t_end);

%
%% System Dynamics
%

 for i = 1:t_end
    
% Calculate new parameters 
    % k1 = 1;
    p_t = p(:,i);

    g1 = (p1(:,1) - p_t)/norm(p1(:,1) - p_t, 2);
    g2 = (p2(:,1) - p_t)/norm(p2(:,1) - p_t, 2);
    g3 = (p3(:,1) - p_t)/norm(p3(:,1) - p_t, 2);
    g4 = (p4(:,1) - p_t)/norm(p4(:,1) - p_t, 2);

%     omega1 = findTrihedralAngle(g2, g3, g4);
%     omega2 = findTrihedralAngle(g3, g4, g1);
%     omega3 = findTrihedralAngle(g4, g1, g2);
%     omega4 = findTrihedralAngle(g1, g2, g3);
    omega1 = findTrihedralAngle(g2, g3, g4);
    omega2 = findTrihedralAngle(g1, g4, g3);
    omega3 = findTrihedralAngle(g1, g2, g4);
    omega4 = findTrihedralAngle(g1, g2, g3);
    errAngle(:,i) = [omega1 - omega1d, omega2 - omega2d, omega3 - omega3d, omega4 - omega4d]';
    omega(:,i) = [omega1, omega2, omega3, omega4]';
%     if abs(angle1C + angle1B + angle1A - 2*pi) < 0.00001
%         p1Cu = -k1 * (angle1Cd - angle1C) * g3;
%         p1Au = -k1 * (angle1Ad - angle1A) * g1;
%         p1Bu = -k1 * (angle1Bd - angle1B) * g2;
%         p1u = p1Cu + p1Au + p1Bu;
%     else
%         p1u = g1 + g2 + g3;
%     end

    u1 = (omega1 - omega1d) * g1;
    u2 = (omega2 - omega2d) * g2;
    u3 = (omega3 - omega3d) * g3;
    u4 = (omega4 - omega4d) * g4;
    u = u1 + u2 + u3 + u4;
    
    p(:,i+1) = p(:,i) + T_step * u;
    
 end

%
%% Simulation Results
%

% Trajectories
figure(1)
hold on
% text(-1.2,-0.4,'B_1','FontSize', 14');
% text(0,5.2,'B_3','FontSize', 14');
% text(4.1,-0.4,'B_2','FontSize', 14');
% plot(pA(1,1:t_end), pA(2,1:t_end), 'm', pB(1,1:t_end), pB(2,1:t_end), 'b', pC(1,1:t_end), pC(2,1:t_end), 'g', pA(1,t_end), pA(2,t_end), 'om', pB(1,t_end), pB(2,t_end), 'ob', pC(1,t_end), pC(2,t_end), 'og',pA(1,1), pA(2,1), '*m', pB(1,1), pB(2,1), '*b', pC(1,1), pC(2,1), '*g')
plot3(p1(1,1), p1(2,1), p1(3,1), 'ob', p2(1,1), p2(2,1), p2(3,1), 'oy', p3(1,1), p3(2,1), p3(3,1), 'og', p4(1,1), p4(2,1), p4(3,1), 'om')

m1 = plot3(p(1,t_end), p(2,t_end), p(3,t_end),'sr');
m2 = plot3(p(1,1), p(2,1), p(3,1), 'or');
m3 = plot3(p(1,1:t_end), p(2,1:t_end), p(3,1:t_end), 'r', 'LineWidth', 2);
grid on

line([p1(1,1), p2(1,1)],[p1(2,1), p2(2,1)],[p1(3,1), p2(3,1)],'LineStyle','--','Color','k', 'LineWidth', 2);
line([p3(1,1), p2(1,1)],[p3(2,1), p2(2,1)],[p3(3,1), p2(3,1)],'LineStyle','--','Color','k', 'LineWidth', 2);
line([p1(1,1), p3(1,1)],[p1(2,1), p3(2,1)],[p1(3,1), p3(3,1)],'LineStyle','--','Color','k', 'LineWidth', 2);
line([p1(1,1), p4(1,1)],[p1(2,1), p4(2,1)],[p1(3,1), p4(3,1)],'LineStyle','--','Color','k', 'LineWidth', 2);
line([p4(1,1), p3(1,1)],[p4(2,1), p3(2,1)],[p4(3,1), p3(3,1)],'LineStyle','--','Color','k', 'LineWidth', 2);
line([p4(1,1), p2(1,1)],[p4(2,1), p2(2,1)],[p4(3,1), p2(3,1)],'LineStyle','--','Color','k', 'LineWidth', 2);


line([p1(1,1), p(1,t_end)],[p1(2,1), p(2,t_end)],[p1(3,1), p(3,t_end)],'LineStyle','--','Color','b', 'LineWidth', 2);
line([p(1,t_end), p2(1,1)],[p(2,t_end), p2(2,1)],[p(3,t_end), p2(3,1)],'LineStyle','--','Color','b', 'LineWidth', 2);
line([p(1,t_end), p3(1,1)],[p(2,t_end), p3(2,1)],[p(3,t_end), p3(3,1)],'LineStyle','--','Color','b', 'LineWidth', 2);
line([p(1,t_end), p4(1,1)],[p(2,t_end), p4(2,1)],[p(3,t_end), p4(3,1)],'LineStyle','--','Color','b', 'LineWidth', 2);

xlabel('x');ylabel('y');zlabel('z');

m = legend([m2 m1 m3], 'Init. position', 'Final position', 'Trajectory');

% Final bearing angles
figure(2)
hold on

k1=plot([0:t_end-2]*0.01, errAngle(1,1:t_end-1), '-b', 'LineWidth', 2 );
k2=plot([0:t_end-2]*0.01, errAngle(2,1:t_end-1), '-c', 'LineWidth', 2 );
k3=plot([0:t_end-2]*0.01, errAngle(3,1:t_end-1), '-m', 'LineWidth', 2 );
k4=plot([0:t_end-2]*0.01, errAngle(4,1:t_end-1), '-g', 'LineWidth', 2 );
h = legend([k1 k2 k3 k4],'$e_1$','$e_2$','$e_3$','$e_4$');
xlabel('Time [s]'); ylabel('angle errors [rad]');
grid on;
set(h,'Interpreter','latex');

errAngle(:,t_end)
omega(:,t_end)
sum(omega(:,t_end))











