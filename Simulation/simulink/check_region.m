function r = check_region(angle_1, angle_2, angle_3)
if (abs(angle_1 - angle_2 - angle_3)<eps)&&(angle_1 < pi)
    r = 1;
else
    if (abs(angle_2 - angle_1 - angle_3)<eps)&&(angle_2 < pi)
        r = 2;
    else
        if (abs(angle_3 - angle_1 - angle_2)<eps)&&(angle_3 < pi)
            r = 3;
        else
            %if abs(angle_1 + angle_2 + angle_3 - 2*pi)<=0.000001
                r = 0;
            %end
        end
    end
end