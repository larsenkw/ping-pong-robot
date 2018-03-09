syms theta1 theta2 theta3;
syms L1 L2 L3

T1 = dhTransform(linkListP3(1).a, L1, linkListP3(1).alpha, theta1);
T2 = dhTransform(linkListP3(1).a, L1, linkListP3(1).alpha, theta1) * ...
    dhTransform(L2, linkListP3(2).d, linkListP3(2).alpha, theta2);
T3 = dhTransform(linkListP3(1).a, L1, linkListP3(1).alpha, theta1) * ...
    dhTransform(L2, linkListP3(2).d, linkListP3(2).alpha, theta2) * ...
    dhTransform(L3, linkListP3(3).d, linkListP3(3).alpha, theta3);

vpa(T1,4)
vpa(T2,4)
vpa(T3,4)