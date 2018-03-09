function [Z1, Y, Z2] = rot2ZYZ(R)

    Z1 = atan2(R(2,3), R(1,3));
    Y  = atan2(sqrt(R(1,3)^2 + R(2,3)^2), R(3,3));
    Z2 = atan2(R(3,2), -R(3,1));

end