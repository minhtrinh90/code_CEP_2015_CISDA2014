function dpdt = control_lawNav(t,p)
global mem region_d1 region_d
% 
pA = [ 10;  0]; 
pB = [ 0;  0]; 
pC = [ 0;  10];
angle1Ad = pi/6; angle1Bd =pi/3; angle1Cd = pi/6; 
k = 5;
k1 = 1;
if (norm(pA - p,2)<=0.00001)
    a1 = [0;0];
else
a1 = (pA - p)/norm(pA - p, 2);
end
if (norm(pB - p,2)<=0.00001)
    b1 = [0;0];
else
b1 = (pB - p)/norm(pB - p, 2);
end
if (norm(pC - p,2)<=0.00001)
    c1 = [0;0];
else
c1 = (pC - p)/norm(pC - p, 2);
end
angle1C = findAngle1(a1(1,1),a1(2,1),b1(1,1),b1(2,1));
angle1A = findAngle1(b1(1,1),b1(2,1),c1(1,1),c1(2,1));
angle1B = findAngle1(c1(1,1),c1(2,1),a1(1,1),a1(2,1));

region = check_region(angle1A, angle1B, angle1C);
p1u = 0;
if (mem==0)
    if (region==0)
        mem = 1;
    else
        p1u = k1*(a1 + b1 + c1);  % Try to get to R_0
    end
else
    if (region_d1~=region)
        switch(region_d)
            case 1
                p1u = -a1 + b1 + c1; % Try to get to R_1a from R0
            case 2
                p1u = a1 - b1 - c1;  % Try to get to R_1b from R0
            case 3
                p1u = a1 - b1 + c1;  % Try to get to R_2a from R0
            case 4
                p1u = -a1 + b1 - c1; % Try to get to R_2b from R0
            case 5
                p1u = a1 + b1 - c1;  % Try to get to R_1c from R0
            case 6
                p1u = -a1 - b1 + c1; % Try to get to R_2c from R0
        end
    else
        % Calculate control components
        p1Cu = -k * (angle1Cd - angle1C) * c1; 
        p1Au = -k * (angle1Ad - angle1A) * a1;
        p1Bu = -k * (angle1Bd - angle1B) * b1;
        switch(region_d)
            case 0
                p1u = p1Cu + p1Au + p1Bu;   % If region_d = 0:
            case 1
                p1u =-(p1Cu + p1Au + p1Bu); % If region_d = 1 & the agent is in R_1a
            case 2
                p1u = p1Cu - p1Au + p1Bu;  % If region_d = 2 & the agent is in R_1b
            case 3
                p1u =-(p1Cu + p1Au + p1Bu);    
            case 4
                p1u = p1Cu + p1Au - p1Bu;
            case 5
                p1u =(-p1Cu + p1Au + p1Bu);   
            case 6
                p1u = -p1Cu + p1Au + p1Bu;          
        end
        if norm(p1u,2)<0.1
            p1u = 6*p1u;
        else
            if norm(p1u,2)<0.2
                p1u = 3*p1u;
            end
        end
     end
end

dpdt = p1u;
end