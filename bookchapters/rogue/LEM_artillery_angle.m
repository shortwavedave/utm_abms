function theta = LEM_artillery_angle(x0,y0,z0,v0)
%

theta = NaN;

g = 9.81;

x = norm([x0,y0]);
z = z0;
tan_theta1 = (v0^2+sqrt(v0^4-g*(g*x^2+2*z*v0^2)))/(g*x);
theta1 = atan(tan_theta1);
tan_theta2 = (v0^2-sqrt(v0^4-g*(g*x^2+2*z*v0^2)))/(g*x);
theta2 = atan(tan_theta2);

if imag(theta1)==0
    theta = theta1;
elseif imag(theta2)==0
    theta = theta2;
end

tch = 0;
