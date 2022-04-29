function [radius,center] = LEM_3pts2circle(p1,p2,p3)
%

radius = (1/2)*(norm(p1-p2)*norm(p2-p3)*norm(p3-p1))...
    /norm(cross((p1-p2),(p2-p3)));
A = [p1(1) p1(2) 1; p2(1) p2(2) 1; p3(1) p3(2) 1];
b = [-(p1(1)^2+p1(2)^2);-(p2(1)^2+p2(2)^2);-(p3(1)^2+p3(2)^2)];
center = [p1(1), p1(2)];

if(det(A) == 0 || det(A) < 20*eps)
    return;
end
x = A\b;
D = x(1);
E = x(2);
F = x(3);
h = -D/2;
k = -E/2;
radius = sqrt(h^2+k^2-F);
center = [h,k];
tch = 0;
