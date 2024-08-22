

Max = 10;
dt = 0.01;

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
K9 = 1;             % Phi_d P
% K10 = 2.5;            % Phi_d D
K11 = 1;            % Theta_d P
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
 
dx = 2;
dy = 2;
dz = 1;
dphi = 0.175;
dtheta = 0.175;
dpsi = 0.175;

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

%t = 1:Max/dt;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulation Parameters
t_end  = 100;                              % simulation time
T_step = 0.01;                                % simulation step
k = 1;
    
pA = [ 10;  0]; 
pB = [ 0;  0]; 
pC = [ 0;  10];
p1 = zeros(2, t_end);                       
p10 = [-6;13]; %[0;7]  [5;4]   [2;-2]   [-3;-0.5]   [-3;-0.5]   [6;-.2]

errAngle = zeros(3, t_end);
p1(:,1) = p10;    
angle1Ad = pi/6; angle1Bd =pi/3; angle1Cd = pi/6; 

region_d = 3;
region_d1 = 2;
mem = 0;

% Initial position of agents
% Position initial
x = p10(1,1);
y = p10(2,1);
z = 0;

% Euler angle initial 
phi = 0.0673;
theta = -0.0873;
psi = 0.1745;

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

zz = zeros(1,t_end);
