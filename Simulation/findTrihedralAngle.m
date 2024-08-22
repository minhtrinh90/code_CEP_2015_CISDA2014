function angle = findTrihedralAngle(m1,m2,m3)

% eta = mod(atan2(v1x*v2y-v1y*v2x , v1x*v2x+v1y*v2y),2*pi);
%eta = atan2(v1x*v2y-v1y*v2x , v1x*v2x+v1y*v2y);
% if eta < 0
%     eta = 2*pi + eta;
% end
% if eta < pi
%     angle = eta;
% else
%     angle = 2 * pi - eta;
% end
% g1 = [g1x;g1y;g1z];
% g2 = [g2x;g2y;g2z];
% g3 = [g3x;g3y;g3z];
% N = m1'*cross(m2,m3);
% D = 1 + m1'*m2+m2'*m3+m3'*m1;
% angle = 2 * atan(abs(N/D));
l1 = acos(m2'*m3);
l2 = acos(m1'*m3);
l3 = acos(m1'*m2);
%s = (l1+l2+l3)/2;
%t = sqrt(tan(s/2)*tan((s-l1)/2)*tan(s-l2)/2*tan((s-l3)/2));
%angle = 4*atan(sqrt(tan(s/2)*tan((s-l1)/2)*tan(s-l2)/2*tan((s-l3)/2)));
angle = 2* acos((1+cos(l1)+cos(l2)+cos(l3))/(4*cos(l1/2)*cos(l2/2)*cos(l3/2)));
end
