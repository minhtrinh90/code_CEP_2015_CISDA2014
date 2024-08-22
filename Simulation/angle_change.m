a = [1;0;0];
b = [0;1;0];

dt = 0.0001;
% v = [0;0;1];
c = zeros(3,10000);
angle = zeros(1,10000);
for i=1:10000
    g1 = (a - c(:,i))/norm(a - c(:,i));
    g2 = (b - c(:,i))/norm(b - c(:,i));
    angle(i) = atan2(norm(cross(g1,g2)), dot(g1,g2));
    v = cross(g1,g2);
    c(:,i+1) = c(:,i) + v * dt;
end
figure(1)
plot3(c(1,1:10000),c(2,1:10000),c(3,1:10000))
grid on
axis square
figure(2)
plot(angle)
grid on