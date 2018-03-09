function [R] = zyz2Rot(z1, y, z2)

    R = [cos(z1)*cos(y)*cos(z2)-sin(z1)*sin(z2) -cos(z1)*cos(y)*sin(z2)-sin(z1)*cos(z2) cos(z1)*sin(y); ...
        sin(z1)*cos(y)*cos(z2)+cos(z1)*sin(z2) -sin(z1)*cos(y)*sin(z2)+cos(z1)*cos(z2) sin(z1)*sin(y); ...
        -sin(y)*cos(z2) sin(y)*sin(z2) cos(y)];

end