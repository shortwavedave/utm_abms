function traj = LEM_artillery_traj(x0,y0,z0,x1,y1,z1,v0,theta,del_t)
%

g = 9.81;
viscosity = 1 ; % 0.3544;

t_max = 2*v0*sin(theta)/g;
t_vals = [0:del_t:t_max];
if t_vals(end)~=t_max
    t_vals = [t_vals,t_max];
end
x_vals = v0*t_vals*cos(theta);
z_vals = v0*t_vals*sin(theta) - (1/2)*g*t_vals.^2;
x_vals = x_vals + x0;
z_vals = z_vals + z0;
traj = [x_vals',0*x_vals',z_vals'];
traj = viscosity*traj;

alpha = atan2(y1-y0,x1-x0);
T = [cos(alpha), -sin(alpha), 0; sin(alpha), cos(alpha), 0; 0 0 1];
traj = (T*traj')';
traj = [traj,t_vals'];
