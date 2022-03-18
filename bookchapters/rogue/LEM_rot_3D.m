function T = LEM_rot_3D(x,theta)
%

ux = x(1);
uy = x(2);
uz = x(3);
c = cos(theta);
s = sin(theta);
T = eye(4,4);
T(1,1) = c + ux^2*(1-c);
T(1,2) = ux*uy*(1-c) - uz*s;
T(1,3) = ux*uz*(1-c) + uy*s;
T(2,1) = uy*ux*(1-c) + uz*s;
T(2,2) = c + uy^2*(1-c);
T(2,3) = uy*uz*(1-c) - ux*s;
T(3,1) = uz*ux*(1-c) - uy*s;
T(3,2) = uz*uy*(1-c) + ux*s;
T(3,3) = c + uz^2*(1-c);
