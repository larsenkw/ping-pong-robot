syms tau_be tau_bs p1 p2 p3 ce se qdot_e qdot_s;

% Simplify the elbow d2 equation
disp('qdotdot_e');
simplify((((tau_bs + 2*se*qdot_e*qdot_s*p2 + se*qdot_e^2*p2)/(p1)) - ((tau_be - se*qdot_s^2*p2)/(p1 + ce*p2)))*(1 + ((ce*p2)/(p1)) - ((p1 + ce*p2 + p3)/(p1 + ce*p2)))^-1)

% Simplify the shoulder d2 equation
disp('qdotdot_s');
simplify((tau_be*(p1 + ce*p2) - tau_bs*(p1 + ce*p2 + p3) - se*qdot_s^2*p2*(p1 + ce*p2) - 2*se*qdot_e*qdot_s*p2*(p1 + ce*p2 + p3) - se*qdot_e^2*p2*(p1 + ce*p2 + p3))/((p1 + ce*p2)^2 - p1*(p1 + ce*p2 + p3)))