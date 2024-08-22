clear all;
clc;

Max = 10;
dt = 1e-2;

% Quadrotor parameter
m = 1.336;        % mass
g = 9.80665;       % gravity
l = 0.285;       % length btw motor and center

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% quadrotor(real)
b = 0.000122641;
d = 0.00000648447;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Ct, Cp down -> b, d down -> RPM up
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Ix = 0.0259;
Iy = 0.0260;
Iz = 0.0397;

% Motor inertia
Jr = 0.000021;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Control parameter (redesign)
% alt
K1 = 2.2;         % alt P
K2 = 0.5;         % alt D

% attitude
K3 = 2.5;         % phi P
K4 = 2.5;         % phi D
K5 = 2.5;         % theta P
K6 = 2.5;         % theta D
K7 = 2;         % psi P
K8 = 2;         % psi D

% virtual (position)
K9 = 1.0;             % Phi_d P
% K10 = 2.5;            % Phi_d D
K11 = 1.0;            % Theta_d P
% K12 = 2.5;            % Theta_d P

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
xi_1 = 1;
xi_2 = 1;
xi_3 = 1;
xi_4 = 1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

epsilon = 0.7071;
beta = 2.5;
bd = 0.7071;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Data save 
dot_Phi = zeros(1,Max/dt);
dot_Theta = zeros(1,Max/dt);
dot_Psi = zeros(1,Max/dt);

Phi = zeros(1,Max/dt);
Theta = zeros(1,Max/dt);
Psi = zeros(1,Max/dt);

Phi_d = zeros(1,Max/dt);
Theta_d = zeros(1,Max/dt);

dot_x = zeros(1,Max/dt);
dot_y = zeros(1,Max/dt);
dot_z = zeros(1,Max/dt);

X = zeros(1,Max/dt);
Y = zeros(1,Max/dt);
Z = zeros(1,Max/dt);
Z_dt = zeros(1,Max/dt);

U1 = zeros(1,Max/dt);
U2 = zeros(1,Max/dt);
U3 = zeros(1,Max/dt);
U4 = zeros(1,Max/dt);
U_x = zeros(1,Max/dt);
U_y = zeros(1,Max/dt);

W = zeros(4,Max/dt);
RPM = zeros(4,Max/dt);

dot_RPM =zeros(4, Max/dt);


C1 = zeros(1,Max/dt);
C2 = zeros(1,Max/dt);

Position = zeros(3,Max/dt);
Position_e = zeros(3,Max/dt);
Euler_e = zeros(3,Max/dt);
Euler = zeros(3,Max/dt);
Euler_D = zeros(3,Max/dt);

Eta1_s = zeros(1,Max/dt);
Eta2_s = zeros(1,Max/dt);

V3 = zeros(1,Max/dt);
V3_dot = zeros(1,Max/dt);

E10 = zeros(1,Max/dt);
E12 = zeros(1,Max/dt);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Nonlinear Disturbance observer
hat_D = zeros(6,Max/dt);
D = zeros(6,Max/dt);

Z_D = zeros(6,Max/dt);
dot_Z_D = zeros(6,Max/dt);
P = zeros(6,Max/dt);

F = zeros(6,Max/dt);
GU = zeros(6,Max/dt);

U1_x = zeros(1,Max/dt);
U1_y = zeros(1,Max/dt);
U1_z = zeros(1,Max/dt);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Disturbance
% dphi = 0.0875;      % 5도
% dphi = 0.175;      % 10도
% dtheta = 0.0525;    % 3도
% dpsi = 0.0175;      % 1도
 
dx = 0.5;
dy = 0.5;
dz = 0.5;
dphi = 10*0.0175;
dtheta = 10*0.0175;
dpsi = 10*0.0175;

% dx = 0;
% dy = 0;
% dz = 0;
% dphi = 0;
% dtheta = 0;
% dpsi = 0;

% Estimated disturbance
dx_hat = 0;
dy_hat = 0;
dz_hat = 0;
dphi_hat = 0;
dtheta_hat = 0;
dpsi_hat = 0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Position initial
x = 0;
y = 0;
z = 0;

% Desired position
x_d = 3;
y_d = 3;
z_d = 2;

% Euler angle initial 
phi = 0.0873;
theta = -0.0873;
psi = 0.1745;
% phi = 0.1745;            % rad (10 deg)
% theta = 0.1745;          % rad (10 deg)
% psi = 0.1745;



% Desired Euler angle 
phi_d = 0;
theta_d = 0;
psi_d = 0;

% Initial angular velocity(w1 ~ w4) 
w1 = 0;
w2 = 0;
w3 = 0;
w4 = 0;
Sigma = 0;

n = 0;

for(t=1:1:Max/dt)
    n = n + 1;
   
   
    % Boundary of Euler angle
    % -2/pi < phi < 2/pi
    phi = rem(phi, 2*pi);    
    if((phi > pi/2) && (phi < (3*pi)/2))
        phi = pi - phi;
    elseif(phi > (3*pi)/2)
        phi = phi - 2*pi;
    elseif((phi >= -(3*pi)/2) && (phi < -pi/2))
        phi = -pi - phi;
    elseif(phi < -(3*pi)/2) 
        phi = phi + 2*pi;
    else
        phi;
    end
    
    % -2/pi < theta < 2/pi
    theta = rem(theta, 2*pi);    
    if((theta > pi/2) && (theta < (3*pi)/2))
        theta = pi - theta;
    elseif(theta > (3*pi)/2)
        theta = theta - 2*pi;
    elseif((theta > -(3*pi)/2) && (theta < -pi/2))
        theta = -pi - theta;
    elseif(theta < -(3*pi)/2) 
        theta = theta + 2*pi;
    else
        theta;
    end  
    
    % -pi <= psi <= pi
    psi = rem(psi,2*pi);
    if(psi > pi)
        psi = psi - 2*pi;
    elseif(psi < -pi)
        psi = psi + 2*pi;
    else
        psi;
    end
    
     
    % Position error
    e_x = x - x_d;
    e_y = y - y_d;
    e_z = z - z_d;
    
    Position(:,t) = [x;y;z];
    Position_e(:,t) = [e_x;e_y;e_z];
    
    
    e2 = dot_z(:,t) + K1*(e_z);
 

    % Altitude control (redesign)
%     U1(:,t) = (m/(cos(phi_d)*cos(theta_d)))*((K1^2-1)*e_z + g - (K1 + K2)*e2);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Nonlinear disturbance observer controller 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    U1(:,t) = (m/(cos(phi_d)*cos(theta_d)))*((K1^2-1)*e_z + g - (K1 + K2)*e2) + (-m/(cos(phi_d)*cos(theta_d)))*hat_D(3,t);

%     U1_z(:,t) = (m/(cos(phi_d)*cos(theta_d)))*((K1^2-1)*e_z + g - (K1 + K2)*e2) + (-m/(cos(phi_d)*cos(theta_d)))*hat_D(3,t);
%     U1_x(:,t) = U1(:,t) + (-m/(cos(phi_d)*sin(theta_d)*cos(psi)+sin(phi_d)*sin(psi)))*hat_D(1,t);
%     U1_y(:,t) = U1(:,t) + (-m/(cos(phi_d)*sin(theta_d)*sin(psi)-sin(phi_d)*cos(psi)))*hat_D(2,t); 
%     U1_x(:,t) = (m/(cos(phi_d)*cos(theta_d)))*((K1^2-1)*e_z + g - (K1 + K2)*e2) + (-m/(cos(phi_d)*sin(theta_d)*cos(psi)+sin(phi_d)*sin(psi)))*hat_D(1,t);
%     U1_y(:,t) = (m/(cos(phi_d)*cos(theta_d)))*((K1^2-1)*e_z + g - (K1 + K2)*e2) + (-m/(cos(phi_d)*sin(theta_d)*sin(psi)-sin(phi_d)*cos(psi)))*hat_D(2,t);

 
    % Virtual control input for Phi_d & Theta_d
    e10 = dot_x(:,t) + K9*(e_x);
    e12 = dot_y(:,t) + K11*(e_y);
    
    E10(:,t) = e10;
    E12(:,t) = e12;

    V3(:,t) = 1/2*(xi_1*e_x^2 + xi_2*e10^2 + xi_3*e_y^2 + xi_4*e12^2);
    V3_dot(:,t) = xi_1*e_x*(e10-K9*e_x) + xi_3*e_y*(e12-K11*e_y) + xi_2*e10*((cos(phi_d)*sin(theta_d)*cos(psi)+sin(phi_d)*sin(psi))*U1(:,t)/m+K9*(e10-K9*e_x)) + xi_4*e12*((cos(phi_d)*sin(theta_d)*sin(psi)-sin(phi_d)*cos(psi))*U1(:,t)/m+K11*(e12-K11*e_y));
    
    if (abs(e10/epsilon) <= bd)
        Sat1 = e10/epsilon;
    elseif(e10/epsilon > bd)
        Sat1 = bd;
    else
        Sat1 = -bd;
    end
   
    if (abs(e12/epsilon) <= bd)
        Sat2 = e12/epsilon;
    elseif(e12/epsilon > bd)
        Sat2 = bd;
    else
        Sat2 = -bd;
    end
    
    u_x = m/U1(:,t)*(-e10-K9*e_x-beta*Sat1);
    u_y = m/U1(:,t)*(-e12-K11*e_y-beta*Sat2);

    eta1_sol = u_x;
    eta2_sol = u_y;
    
    Eta1_s(:,t) = u_x;
    Eta2_s(:,t) = u_y;
     
    % Phi_d
    C1(:,t) = eta1_sol*sin(psi) - eta2_sol*cos(psi);

    Phi_d(:,t) = asin(C1(:,t));
       
    phi_d = Phi_d(:,t);  
    
    if(t == 1)
        dot_Phi_d(:,1) = Phi_d(:,1);
    elseif(t > 2)
        dot_Phi_d(:,t) = (Phi_d(:,t) - Phi_d(:,t-1))/dt;
    end
    
        
   
    % Theta_d
    
    C2(:,t) = eta2_sol/(cos(phi_d)*sin(psi)) + tan(phi_d)/tan(psi);
    
    Theta_d(:,t) = asin(C2(:,t));    
        
    theta_d = Theta_d(:,t);
      
    if(t == 1)
        dot_Theta_d(:,1) = Theta_d(:,1);
    elseif(t > 2)
        dot_Theta_d(:,t) = (Theta_d(:,t) - Theta_d(:,t-1))/dt;
    end
      
    % Euler angle error
    Euler(:,t) = [phi;theta;psi];
    Euler_D(:,t) = [phi*180/pi;theta*180/pi;psi*180/pi];
    
    e_phi = phi - phi_d;
    e_theta = theta - theta_d;
    e_psi = psi - psi_d;
    
    Euler_e(:,t) = [e_phi*180/pi;e_theta*180/pi;e_psi*180/pi];   
    
    e4 = dot_Phi(:,t) + K3*(e_phi);
    e6 = dot_Theta(:,t) + K5*(e_theta);
    e8 = dot_Psi(:,t) + K7*(e_psi);
    
    % Euler control (redesign)
%     U2(:,t) = (Ix/l)*(-dot_Theta(:,t)*dot_Psi(:,t)*((Iy-Iz)/Ix) + (Jr/Ix)*dot_Theta(:,t)*Sigma + (K3^2-1)*e_phi - (K3+K4)*e4);
%     U3(:,t) = (Iy/l)*(-dot_Phi(:,t)*dot_Psi(:,t)*((Iz-Ix)/Iy) - (Jr/Ix)*dot_Phi(:,t)*Sigma + (K5^2-1)*e_theta - (K5+K6)*e6);
%     U4(:,t) = Iz*(-dot_Phi(:,t)*dot_Theta(:,t)*((Ix-Iy)/Iz) + (K7^2-1)*e_psi - (K7+K8)*e8);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Nonlinear disturbance observer controller 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Euler control (NDOBC)
    U2(:,t) = (Ix/l)*(-dot_Theta(:,t)*dot_Psi(:,t)*((Iy-Iz)/Ix) + (Jr/Ix)*dot_Theta(:,t)*Sigma + (K3^2-1)*e_phi - (K3+K4)*e4) + (-Ix/l)*hat_D(4,t);
    U3(:,t) = (Iy/l)*(-dot_Phi(:,t)*dot_Psi(:,t)*((Iz-Ix)/Iy) - (Jr/Ix)*dot_Phi(:,t)*Sigma + (K5^2-1)*e_theta - (K5+K6)*e6) + (-Iy/l)*hat_D(5,t);
    U4(:,t) = Iz*(-dot_Phi(:,t)*dot_Theta(:,t)*((Ix-Iy)/Iz) + (K7^2-1)*e_psi - (K7+K8)*e8) + (-Iz)*hat_D(6,t);  
    
    u1 = U1(:,t);
    u2 = U2(:,t);
    u3 = U3(:,t);
    u4 = U4(:,t);
    
   
    % Motor angular velocity
    w1 = 1/2*sqrt(U1(:,t)/b-2/b*U3(:,t)+U4(:,t)/d);
    w2 = 1/2*sqrt(U1(:,t)/b-2/b*U2(:,t)-U4(:,t)/d);
    w3 = 1/2*sqrt(U1(:,t)/b+2/b*U3(:,t)+U4(:,t)/d);
    w4 = 1/2*sqrt(U1(:,t)/b+2/b*U2(:,t)-U4(:,t)/d);
    
    W(:,t) = [w1;w2;w3;w4];
%     RPM(:,t) = W(:,t)*60/(2*pi);

    RPM(:,t) = W(:,t)*60;
    
    rpm1 = w1*60;
    rpm2 = w2*60;
    rpm3 = w3*60;
    rpm4 = w4*60;
        
  
    Sigma = w1+w3-w2-w4;
       
    
    % Attitude dynamics
    Phi(:,1) = phi;
    Theta(:,1) = theta;
    Psi(:,1) = psi;
    
    dot_Phi(:,t+1) = dot_Phi(:,t) + dt*(dot_Theta(:,t)*dot_Psi(:,t)*(Iy-Iz)/Ix - Jr/Ix*dot_Theta(:,t)*Sigma + l/Ix*U2(:,t) + dphi);
    dot_Theta(:,t+1) = dot_Theta(:,t) + dt*(dot_Phi(:,t)*dot_Psi(:,t)*(Iz-Ix)/Iy + Jr/Iy*dot_Phi(:,t)*Sigma + l/Iy*U3(:,t) + dtheta);
    dot_Psi(:,t+1) = dot_Psi(:,t) + dt*(dot_Phi(:,t)*dot_Theta(:,t)*(Ix-Iy)/Iz + 1/Iz*U4(:,t) + dpsi);
    
    Phi(:,t+1) = Phi(:,t) + dt*dot_Phi(:,t);
    Theta(:,t+1) = Theta(:,t) + dt*dot_Theta(:,t);
    Psi(:,t+1) = Psi(:,t) + dt*dot_Psi(:,t);
    
    phi = Phi(:,t);
    theta = Theta(:,t);
    psi = Psi(:,t);
    
    % Convert to degree
    PHI = phi*180/pi;
    THETA = theta*180/pi;
    PSI = psi*180/pi;
    
    % Position dynamics 
    dot_x(:,t+1) = dot_x(:,t) + dt*((cos(phi_d)*sin(theta_d)*cos(psi)+sin(phi_d)*sin(psi))*U1(:,t)/m+dx);
    dot_y(:,t+1) = dot_y(:,t) + dt*((cos(phi_d)*sin(theta_d)*sin(psi)-sin(phi_d)*cos(psi))*U1(:,t)/m+dy);
    dot_z(:,t+1) = dot_z(:,t) + dt*((cos(phi_d)*cos(theta_d))*U1(:,t)/m - g+dz);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Nonlinear disturbance observer controller 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
%     % Position dynamics (Disturbance case)
%     dot_x(:,t+1) = dot_x(:,t) + dt*((cos(phi_d)*sin(theta_d)*cos(psi)+sin(phi_d)*sin(psi))*U1_x(:,t)/m + dx);
%     dot_y(:,t+1) = dot_y(:,t) + dt*((cos(phi_d)*sin(theta_d)*sin(psi)-sin(phi_d)*cos(psi))*U1_y(:,t)/m + dy);
%     dot_z(:,t+1) = dot_z(:,t) + dt*((cos(phi_d)*cos(theta_d))*U1_z(:,t)/m - g + dz);
    

    X(:,t+1) = X(:,t) + dt*dot_x(:,t);
    Y(:,t+1) = Y(:,t) + dt*dot_y(:,t);
    Z(:,t+1) = Z(:,t) + dt*dot_z(:,t);
    
    x = X(:,t);
    y = Y(:,t);
    z = Z(:,t);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Nonlinear disturbance observer 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    P(:,t) = [dot_x(:,t);dot_y(:,t);dot_z(:,t);dot_Phi(:,t);dot_Theta(:,t);dot_Psi(:,t)];
    
    c = 1;
    L = c*eye(6);
    
    F(:,t) = [0;0;-g;dot_Theta(:,t)*dot_Psi(:,t)*(Iy-Iz)/Ix - Jr/Ix*dot_Theta(:,t)*Sigma;dot_Phi(:,t)*dot_Psi(:,t)*(Iz-Ix)/Iy + Jr/Iy*dot_Phi(:,t)*Sigma;dot_Phi(:,t)*dot_Theta(:,t)*(Ix-Iy)/Iz];
    GU(:,t) = [(cos(phi_d)*sin(theta_d)*cos(psi)+sin(phi_d)*sin(psi))*U1(:,t)/m;(cos(phi_d)*sin(theta_d)*sin(psi)-sin(phi_d)*cos(psi))*U1(:,t)/m;(cos(phi_d)*cos(theta_d))*U1(:,t)/m;l/Ix*U2(:,t);l/Iy*U3(:,t);1/Iz*U4(:,t)];
   
    
    hat_D(:,1) = [dx_hat;dy_hat;dz_hat;dphi_hat;dtheta_hat;dpsi_hat];
    
  
    Z_D(:,t+1) = Z_D(:,t) + dt*(-L*(Z_D(:,t)+P(:,t)+F(:,t)+GU(:,t)));

    
    hat_D(:,t+1) = Z_D(:,t) + P(:,t);
    
    
    dx_hat_D = hat_D(1,t);
    dy_hat_D = hat_D(2,t);
    dz_hat_D = hat_D(3,t);
    
    dphi_hat_D = hat_D(4,t)*180/pi;
    dtheta_hat_D = hat_D(5,t)*180/pi;
    dpsi_hat_D = hat_D(6,t)*180/pi;
    
    n
       
    
end


t = 1:Max/dt;

figure(1);
subplot(311);
grid on;
plot(t,hat_D(1,t),'LineWidth',1.5);
title('x');
set(gca,'XTickLabel',{'0','1','2','3','4','5','6','7','8','9','10'})
% axis([0,1000,0,12]);
hold on;
subplot(312);
grid on;
plot(t,hat_D(2,t),'LineWidth',1.5);
title('y');
set(gca,'XTickLabel',{'0','1','2','3','4','5','6','7','8','9','10'})        
ylabel('m');
% axis([0,1000,0,4]);
hold on;
subplot(313);
grid on;
plot(t,hat_D(3,t),'LineWidth',1.5);
title('z');
xlabel('time(sec)');
set(gca,'XTickLabel',{'0','1','2','3','4','5','6','7','8','9','10'})        
% axis([0,1000,0,2]);
hold on;

figure(2);
subplot(311);
grid on;
plot(t,hat_D(4,t)*180/pi,'LineWidth',1.5);
title('Roll');
set(gca,'XTickLabel',{'0','1','2','3','4','5','6','7','8','9','10'})
% axis([0,1000,0,12]);
hold on;
subplot(312);
grid on;
plot(t,hat_D(5,t)*180/pi,'LineWidth',1.5);
title('Pitch');
set(gca,'XTickLabel',{'0','1','2','3','4','5','6','7','8','9','10'})
% axis([0,1000,0,12]);
hold on;
subplot(313);
grid on;
plot(t,hat_D(6,t)*180/pi,'LineWidth',1.5);
title('Yaw');
xlabel('time(sec)');
set(gca,'XTickLabel',{'0','1','2','3','4','5','6','7','8','9','10'})
% axis([0,1000,0,12]);
hold on;

% figure(3);
% subplot(311);
% grid on;
% plot(t,Position(1,t), 'LineWidth',1.5);
% title('x');
% set(gca,'XTickLabel',{'0','1','2','3','4','5','6','7','8','9','10'})
% % axis([0,1000,0,8]);
% hold on;
% subplot(312);
% grid on;
% plot(t,Position(2,t), 'LineWidth',1.5);
% title('y');
% set(gca,'XTickLabel',{'0','1','2','3','4','5','6','7','8','9','10'})        
% ylabel('m');
% % axis([0,1000,0,8]);
% hold on;
% subplot(313);
% grid on;
% plot(t,Position(3,t), 'LineWidth',1.5);
% title('z');
% set(gca,'XTickLabel',{'0','1','2','3','4','5','6','7','8','9','10'})        
% xlabel('time(sec)');
% axis([0,1000,0,3]);
% hold on;
% 
% figure(4);
% subplot(311);
% grid on;
% plot(t,Euler_D(1,t), 'LineWidth',1.5);
% title('Roll');
% set(gca,'XTickLabel',{'0','1','2','3','4','5','6','7','8','9','10'})
% % axis([0,1000,0,10]);
% hold on;
% subplot(312);
% grid on;
% plot(t,Euler_D(2,t), 'LineWidth',1.5);
% title('Pitch');
% set(gca,'XTickLabel',{'0','1','2','3','4','5','6','7','8','9','10'})        
% ylabel('degree');
% % axis([0,1000,0,10]);
% hold on;
% subplot(313);
% grid on;
% plot(t,Euler_D(3,t), 'LineWidth',1.5);
% title('Yaw');
% set(gca,'XTickLabel',{'0','1','2','3','4','5','6','7','8','9','10'})        
% xlabel('time(sec)');
% axis([0,1000,0,10]);
% hold on;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(3);
subplot(311);
grid on;
plot(t,Position(1,t),'r--', 'LineWidth',1.5);
title('x');
set(gca,'XTickLabel',{'0','1','2','3','4','5','6','7','8','9','10'})
% axis([0,1000,0,8]);
hold on;
subplot(312);
grid on;
plot(t,Position(2,t),'r--', 'LineWidth',1.5);
title('y');
set(gca,'XTickLabel',{'0','1','2','3','4','5','6','7','8','9','10'})        
ylabel('m');
% axis([0,1000,0,8]);
hold on;
subplot(313);
grid on;
plot(t,Position(3,t),'r--', 'LineWidth',1.5);
title('z');
set(gca,'XTickLabel',{'0','1','2','3','4','5','6','7','8','9','10'})        
xlabel('time(sec)');
axis([0,1000,0,3]);
hold on;

figure(4);
subplot(311);
grid on;
plot(t,Euler_D(1,t),'r--', 'LineWidth',1.5);
title('Roll');
set(gca,'XTickLabel',{'0','1','2','3','4','5','6','7','8','9','10'})
% axis([0,1000,0,10]);
hold on;
subplot(312);
grid on;
plot(t,Euler_D(2,t),'r--', 'LineWidth',1.5);
title('Pitch');
set(gca,'XTickLabel',{'0','1','2','3','4','5','6','7','8','9','10'})        
ylabel('degree');
% axis([0,1000,0,10]);
hold on;
subplot(313);
grid on;
plot(t,Euler_D(3,t),'r--', 'LineWidth',1.5);
title('Yaw');
set(gca,'XTickLabel',{'0','1','2','3','4','5','6','7','8','9','10'})        
xlabel('time(sec)');
axis([0,1000,0,10]);
hold on;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% figure(3);
% subplot(311);
% grid on;
% plot(t,Position(1,t),'k--', 'LineWidth',1.5);
% title('x');
% set(gca,'XTickLabel',{'0','1','2','3','4','5','6','7','8','9','10'})
% % axis([0,1000,0,8]);
% hold on;
% subplot(312);
% grid on;
% plot(t,Position(2,t),'k--', 'LineWidth',1.5);
% title('y');
% set(gca,'XTickLabel',{'0','1','2','3','4','5','6','7','8','9','10'})        
% ylabel('m');
% % axis([0,1000,0,8]);
% hold on;
% subplot(313);
% grid on;
% plot(t,Position(3,t),'k--', 'LineWidth',1.5);
% title('z');
% set(gca,'XTickLabel',{'0','1','2','3','4','5','6','7','8','9','10'})        
% xlabel('time(sec)');
% axis([0,1000,0,3]);
% hold on;
% 
% figure(4);
% subplot(311);
% grid on;
% plot(t,Euler_D(1,t),'k--', 'LineWidth',1.5);
% title('Roll');
% set(gca,'XTickLabel',{'0','1','2','3','4','5','6','7','8','9','10'})
% % axis([0,1000,0,10]);
% hold on;
% subplot(312);
% grid on;
% plot(t,Euler_D(2,t),'k--', 'LineWidth',1.5);
% title('Pitch');
% set(gca,'XTickLabel',{'0','1','2','3','4','5','6','7','8','9','10'})        
% ylabel('degree');
% % axis([0,1000,0,10]);
% hold on;
% subplot(313);
% grid on;
% plot(t,Euler_D(3,t),'k--', 'LineWidth',1.5);
% title('Yaw');
% set(gca,'XTickLabel',{'0','1','2','3','4','5','6','7','8','9','10'})        
% xlabel('time(sec)');
% % axis([0,1000,0,10]);
% hold on;