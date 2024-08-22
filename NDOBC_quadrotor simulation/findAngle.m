function angle = findAngle(v1x,v1y,v2x,v2y)

% eta = mod(atan2(v1x*v2y-v1y*v2x , v1x*v2x+v1y*v2y),2*pi);
eta = atan2(v1x*v2y-v1y*v2x , v1x*v2x+v1y*v2y);
if eta < 0
    eta = 2*pi + eta;
end
if eta < pi
    angle = eta;
else
    angle = 2 * pi - eta;
end

end
