    clear all;
    clc;

%% Problem setup
%

% Simulation Parameters
global mem region_d region_d1
pA = [10;  0]; 
pB = [ 0;  0]; 
pC = [ 0;  10];

angle1Ad = pi/6; angle1Bd = pi/3; angle1Cd = pi/6; 
region_d = 3;  % Region d is used to check whether the desired position is inside R0, or R_ia, or R_ib
               % for i = 1, 2, 3
region_d1 = 2; % Region d_1 is used to check whether the desired position 
               % is inside R_0 or (R_1a & R_1b) or (R_2a & R_2b) or (R_3a & R_3b)

tsim=0:0.01:50;
%% System Dynamics 1
p0 = [5;-3];
mem = 0;
[t,p1] = ode45(@control_lawNav,tsim,p0);
p1 = p1';
t_end = length(t);
errAngle = zeros(4,t_end);

% Calculate error angles
for i = 1:t_end
    
    % Calculate new parameters 
    p_t = p1(:,i);

    a1 = (pA - p_t)/norm(pA - p_t, 2);
    b1 = (pB - p_t)/norm(pB - p_t, 2);
    c1 = (pC - p_t)/norm(pC - p_t, 2);

    angle1C = findAngle1(a1(1,1),a1(2,1),b1(1,1),b1(2,1));
    angle1A = findAngle1(b1(1,1),b1(2,1),c1(1,1),c1(2,1));
    angle1B = findAngle1(c1(1,1),c1(2,1),a1(1,1),a1(2,1));

    errAngle(1,i) = abs(angle1Ad - angle1A) + abs(angle1Bd - angle1B) +abs(angle1Cd - angle1C);
end


%% System Dynamics 2
p0 = [-3;-3];
global mem
mem = 0;
[t,p2] = ode45(@control_lawNav,tsim,p0);
p2 = p2';
t_end = length(t);

% Calculate error angles
for i = 1:t_end
    
    % Calculate new parameters 
    p_t = p2(:,i);

    a1 = (pA - p_t)/norm(pA - p_t, 2);
    b1 = (pB - p_t)/norm(pB - p_t, 2);
    c1 = (pC - p_t)/norm(pC - p_t, 2);

    angle1C = findAngle1(a1(1,1),a1(2,1),b1(1,1),b1(2,1));
    angle1A = findAngle1(b1(1,1),b1(2,1),c1(1,1),c1(2,1));
    angle1B = findAngle1(c1(1,1),c1(2,1),a1(1,1),a1(2,1));

    errAngle(2,i) = abs(angle1Ad - angle1A) + abs(angle1Bd - angle1B) +abs(angle1Cd - angle1C);
end

%
%% System Dynamics 3
p0 = [-3;3]; 
global mem
mem = 0;
[t,p3] = ode45(@control_lawNav,tsim,p0);
p3 = p3';
t_end = length(t);

% Calculate error angles
for i = 1:t_end
    
    % Calculate new parameters 
    p_t = p3(:,i);

    a1 = (pA - p_t)/norm(pA - p_t, 2);
    b1 = (pB - p_t)/norm(pB - p_t, 2);
    c1 = (pC - p_t)/norm(pC - p_t, 2);

    angle1C = findAngle1(a1(1,1),a1(2,1),b1(1,1),b1(2,1));
    angle1A = findAngle1(b1(1,1),b1(2,1),c1(1,1),c1(2,1));
    angle1B = findAngle1(c1(1,1),c1(2,1),a1(1,1),a1(2,1));

    errAngle(3,i) = abs(angle1Ad - angle1A) + abs(angle1Bd - angle1B) +abs(angle1Cd - angle1C);
end

%% System Dynamics 4
p0 = [6;13]; 
global mem
mem = 0;
[t,p4] = ode45(@control_lawNav,tsim,p0);
p4 = p4';
t_end = length(t);

% Calculate error angles
for i = 1:t_end
    
    % Calculate new parameters 
    p_t = p4(:,i);

    a1 = (pA - p_t)/norm(pA - p_t, 2);
    b1 = (pB - p_t)/norm(pB - p_t, 2);
    c1 = (pC - p_t)/norm(pC - p_t, 2);

    angle1C = findAngle1(a1(1,1),a1(2,1),b1(1,1),b1(2,1));
    angle1A = findAngle1(b1(1,1),b1(2,1),c1(1,1),c1(2,1));
    angle1B = findAngle1(c1(1,1),c1(2,1),a1(1,1),a1(2,1));

    errAngle(4,i) = abs(angle1Ad - angle1A) + abs(angle1Bd - angle1B) + abs(angle1Cd - angle1C);
end

%% Simulation Animation
F(t_end) = struct('cdata',[],'colormap',[]);
v = VideoWriter('simNavBOM.avi');
v.FrameRate = 20;
open(v);

figure(1); clf
hold on

for i=1:1:t_end
%     if ((mod(i,5)==0)&&(t(i)<10))
%         plotResults(p1(:,1:i),p2(:,1:i),p3(:,1:i),p4(:,1:i),errAngle(:,1:i),t(1:i))
%         drawnow %limitrate
%         F = getframe(gcf);
%         writeVideo(v,F);
%     else
        if (mod(i,20)==0)
            plotResults(p1(:,1:i),p2(:,1:i),p3(:,1:i),p4(:,1:i),errAngle(:,1:i),t(1:i))
            drawnow %limitrate
            F = getframe(gcf);
            writeVideo(v,F);
        end
%     end

end

F = getframe(gcf);
writeVideo(v,F);
close(v);





