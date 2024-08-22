p_t = [x;y];% + 0.7*rand(2,1);

a1 = (pA - p_t)/norm(pA - p_t, 2);
b1 = (pB - p_t)/norm(pB - p_t, 2);
c1 = (pC - p_t)/norm(pC - p_t, 2);

angle1C = findAngle1(a1(1,1),a1(2,1),b1(1,1),b1(2,1));
angle1A = findAngle1(b1(1,1),b1(2,1),c1(1,1),c1(2,1));
angle1B = findAngle1(c1(1,1),c1(2,1),a1(1,1),a1(2,1));

% errAngle(:,i) = -[angle1Ad - angle1A, angle1Bd - angle1B, angle1Cd - angle1C]';

region = check_region(angle1A, angle1B, angle1C)
p1u = [0;0];
if (mem==0)
    if (region==0)
        mem = 1;
    else
        p1u = a1 + b1 + c1;
    end
else
    if (region_d1~=region)
        switch(region_d)
            case 1
                p1u = -a1 + b1 + c1;
            case 2
                p1u = a1 - b1 - c1;
            case 3
                p1u = a1 - b1 + c1;
            case 4
                p1u = -a1 + b1 - c1;
            case 5
                p1u = a1 + b1 - c1;
            case 6
                p1u = -a1 - b1 + c1;
        end
    else
        p1Cu = - (angle1Cd - angle1C) * c1;
        p1Au = - (angle1Ad - angle1A) * a1;
        p1Bu = - (angle1Bd - angle1B) * b1;
        switch(region_d)
            case 0
                p1u = p1Cu + p1Au + p1Bu;
            case 1
                p1u =-(p1Cu + p1Au + p1Bu); 
            case 3
                p1u =-(p1Cu + p1Au + p1Bu); 
            case 5
                p1u =(p1Cu + p1Au + p1Bu);       
            case 2
                p1u = p1Cu - p1Au + p1Bu;       
            case 4
                p1u = p1Cu + p1Au - p1Bu;
            case 6
                p1u = -p1Cu + p1Au + p1Bu;          
        end
     end
end

% Desired position
x_d = x + p1u(1,1)*3 + 0.5*(rand(1)-0.5);
y_d = y + p1u(2,1)*3 + 0.5*(rand(1)-0.5);
z_d = 3.15 + 0.05*(rand(1)-0.5);
phi_d = 0;
theta_d = 0;
psi_d = 0;

fast_loop;
