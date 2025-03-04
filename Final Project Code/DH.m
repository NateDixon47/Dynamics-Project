function T = DH(alpha, a, d, q)
T = [cos(q) -sin(q)*cos(alpha) sin(q)*sin(alpha)  a*cos(q);
     sin(q) cos(q)*cos(alpha)  -cos(q)*sin(alpha) a*sin(q);
     0          sin(alpha)             cos(alpha)             d;
     0          0                      0                      1];
end