function [p,s] = CV_total_LS(x,y)
%CV_total_LS - total least squares method to fit best line to points
%    (Forsyth and Ponce page 335)
%On input:
%     x (nx1 vector): x coordinates of points
%     y (nx1 vector): y coordinates of points
%On output:
%     p (1x3 vector): coefficients of best fit line  ax + by + c = 0
%     s (float): error measure (sum of squares of distances
%                of points to line)
%Call:
%     [p1,s1] =  CV_total_LS([1,2,3],[1,2,3]);
%Author:
%     Tom Henderson
%     UU
%     Fall 2004
%

lenx = length(x);
leny = length(y);

xm = mean(x);
ym = mean(y);
x2m = sum(x.*x)/lenx;
y2m = sum(y.*y)/leny;
xym = sum(x.*y)/lenx;
A = [x2m-xm^2 xym-xm*ym; xym-xm*ym y2m-ym^2];

[V,D] = eigs(A);
if D(1,1)<D(2,2)
    a = V(1,1);
    b = V(2,1);
    c = -a*xm-b*ym;
else
    a = V(1,2);
    b = V(2,2);
    c = -a*xm-b*ym;
end

p = [a,b,c];
s1 = (a*x+b*y+c);
s = sum(s1.^2);