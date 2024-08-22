%% Quadrotor fastloop control
for(t=1:1:600)
   % n = n + 1;
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
    U1_x(:,t) = U1(:,t) + (-m/(cos(phi_d)*sin(theta_d)*cos(psi)+sin(phi_d)*sin(psi)))*hat_D(1,t);
    U1_y(:,t) = U1(:,t) + (-m/(cos(phi_d)*sin(theta_d)*sin(psi)-sin(phi_d)*cos(psi)))*hat_D(2,t); 
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
    
%     % Euler control (redesign)
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
    
    Phi(:,t+1) = Phi(:,t) + real(dt*dot_Phi(:,t));
    Theta(:,t+1) = Theta(:,t) + real(dt*dot_Theta(:,t));
    Psi(:,t+1) = Psi(:,t) + real(dt*dot_Psi(:,t));
    
    phi = Phi(:,t);
    theta = Theta(:,t);
    psi = Psi(:,t);
    
    % Convert to degree
    PHI = phi*180/pi;
    THETA = theta*180/pi;
    PSI = psi*180/pi;
    
    % Position dynamics 
    dot_x(:,t+1) = dot_x(:,t) + dt*real((cos(phi_d)*sin(theta_d)*cos(psi)+sin(phi_d)*sin(psi))*U1(:,t)/m+dx);
    dot_y(:,t+1) = dot_y(:,t) + dt*real((cos(phi_d)*sin(theta_d)*sin(psi)-sin(phi_d)*cos(psi))*U1(:,t)/m+dy);
    dot_z(:,t+1) = dot_z(:,t) + dt*real((cos(phi_d)*cos(theta_d))*U1(:,t)/m - g+dz);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Nonlinear disturbance observer controller 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
%     % Position dynamics (Disturbance case)
%     dot_x(:,t+1) = dot_x(:,t) + dt*((cos(phi_d)*sin(theta_d)*cos(psi)+sin(phi_d)*sin(psi))*U1_z(:,t)/m + dx);
%     dot_y(:,t+1) = dot_y(:,t) + dt*((cos(phi_d)*sin(theta_d)*sin(psi)-sin(phi_d)*cos(psi))*U1_z(:,t)/m + dy);
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
   % n
end