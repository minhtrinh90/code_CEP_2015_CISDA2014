%% Moving in the plane using only bearing angle information
% 2014.05.02
% There are 3 reference agents located at 3 points forming a triangle
% There is one follower which desired angles are relatively defined
% with regard to the landmarks.
    clear all
    clc

%% Problem setup
%

% Simulation Parameters
 %   t_end  = 3000;                               % simulation time
    T_step = 0.01;                                % simulation step
    k = 1;
    tf = 4;
    t_end = 10000;
% Calculate the desired relative positions of agents in the desired formation


  % Three leader's A, B, C's desired bearing angles
    pAd = [-1;0];                                % we can assume that first leader position is stationary
    pBd = [4;0];                                 % two second leaders' relative positions forming a triangle
    pCd = [0;5];                                 % with angles: 90, 45, 45 degree
    
% Initial position of agents
    pA = zeros(2, t_end); pB = zeros(2, t_end); pC = zeros(2, t_end);
%     pA(:,1) = [ 0;  3]; 
%     pB(:,1) = [ 2; -4]; 
%     pC(:,1) = [-1;  1];
    pA(:,1) = [ -1;  0]; 
    pB(:,1) = [ 4;  0]; 
    pC(:,1) = [ 0;  5];
    p1 = zeros(2, t_end);                       
    p10 = [1.5;3]; %(2.5;1.5)   (0;0) (2;0) (-.4; 2.5) (0.5;4) (2;2.5) (-0.5;0.5) (1.5;3)
    % [-2;-3]   [6;-3]     [-1;7]   [5;6]   [-3;3]
    errAngle = zeros(3, t_end);
    p1(:,1) = p10;    
    angle1Ad = 2*pi/3; angle1Bd = 2*pi/3; angle1Cd = 2*pi/3; 

%
%% System Dynamics
%

 for i = 1:t_end
    
% Calculate new parameters 
    k1 = 1;
    p_t = p1(:,i);

    a1 = (pA(:,1) - p_t)/norm(pA(:,1) - p_t, 2);
    b1 = (pB(:,1) - p_t)/norm(pB(:,1) - p_t, 2);
    c1 = (pC(:,1) - p_t)/norm(pC(:,1) - p_t, 2);

    angle1C = findAngle1(a1(1,1),a1(2,1),b1(1,1),b1(2,1));
    angle1A = findAngle1(b1(1,1),b1(2,1),c1(1,1),c1(2,1));
    angle1B = findAngle1(c1(1,1),c1(2,1),a1(1,1),a1(2,1));

    errAngle(:,i) = -[angle1Ad - angle1A, angle1Bd - angle1B, angle1Cd - angle1C]';
        
    if abs(angle1C + angle1B + angle1A - 2*pi) < 0.00001
        p1Cu = -k1 * (angle1Cd - angle1C) * c1;
        p1Au = -k1 * (angle1Ad - angle1A) * a1;
        p1Bu = -k1 * (angle1Bd - angle1B) * b1;
        p1u = p1Cu + p1Au + p1Bu;
    else
        p1u = a1 + b1 + c1;
    end
%     p1u = a1+b1+c1;
    p1(:,i+1) = p1(:,i) + T_step * p1u;
    
 end

%
%% Simulation Results
%

% Trajectories
figure(1)
hold on
text(-1.2,-0.4,'B_1','FontSize', 14');
text(0,5.2,'B_3','FontSize', 14');
text(4.1,-0.4,'B_2','FontSize', 14');
% plot(pA(1,1:t_end), pA(2,1:t_end), 'm', pB(1,1:t_end), pB(2,1:t_end), 'b', pC(1,1:t_end), pC(2,1:t_end), 'g', pA(1,t_end), pA(2,t_end), 'om', pB(1,t_end), pB(2,t_end), 'ob', pC(1,t_end), pC(2,t_end), 'og',pA(1,1), pA(2,1), '*m', pB(1,1), pB(2,1), '*b', pC(1,1), pC(2,1), '*g')
plot(pA(1,1), pA(2,1), 'ob', pB(1,1), pB(2,1), 'oy', pC(1,1), pC(2,1), 'og')

m2 = plot(p1(1,1), p1(2,1), 'or');
m1 = plot(p1(1,t_end), p1(2,t_end), 'sr');
m3 = plot(p1(1,1:t_end), p1(2,1:t_end), 'r', 'LineWidth', 2);
grid on

line([pA(1,1), pB(1,1)],[pA(2,1), pB(2,1)],'LineStyle','--','Color','k', 'LineWidth', 2);
line([pC(1,1), pB(1,1)],[pC(2,1), pB(2,1)],'LineStyle','--','Color','k', 'LineWidth', 2);
line([pA(1,1), pC(1,1)],[pA(2,1), pC(2,1)],'LineStyle','--','Color','k', 'LineWidth', 2);

line([pA(1,1), p1(1,t_end)],[pA(2,1), p1(2,t_end)],'LineStyle','--','Color','b', 'LineWidth', 2);
line([p1(1,t_end), pB(1,1)],[p1(2,t_end), pB(2,1)],'LineStyle','--','Color','b', 'LineWidth', 2);
line([p1(1,t_end), pC(1,1)],[p1(2,t_end), pC(2,1)],'LineStyle','--','Color','b', 'LineWidth', 2);
xlabel('x');ylabel('y');
m = legend([m2 m1 m3], 'init. position', 'desired position', 'trajectory');

% Final bearing angles
disp('Final bearing angles:')
% rad2deg([angleA angleB angleC])
% rad2deg([angle1A angle1B angle1C])
% sum(rad2deg([angle1A angle1B angle1C]))

figure(2)
k1=plot( [0:t_end-2]*0.01, errAngle(1,1:t_end-1), '-b', 'LineWidth', 2 );
hold on
k2=plot( [0:t_end-2]*0.01, errAngle(2,1:t_end-1), '--k', 'LineWidth', 2 );
k3=plot( [0:t_end-2]*0.01, errAngle(3,1:t_end-1), '-.m', 'LineWidth', 2 );
h = legend([k1 k2 k3],'$e_1$','$e_2$','$e_3$','FontSize',14);
xlabel('time [s]'); ylabel('angle errors [rad]');
grid on;
set(h,'Interpreter','latex');















