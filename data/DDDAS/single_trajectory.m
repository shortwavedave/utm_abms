function single_trajectory()
%SINGLE_TRAJECTORY Summary of this function goes here
%   Detailed explanation goes here

f_hz = 1;

leg_len = 100;
vert_displace = 30;
toa_s = [0 2.2 9];
[wps, ground_speeds, climb_rates] = calc_wps(leg_len, vert_displace, toa_s);

figure;
plot3(wps(:,1),wps(:,2),wps(:,3), '-*r')


traj = TrajectoryNoClimbRate(f_hz, wps, toa_s);

hold on
traj.plot_all(false);
hold off

disp(['Calculated linear ground_speed', mat2str(ground_speeds)]);
disp(['Calculated linear climb_rate', mat2str(climb_rates)]);

end

function [wps, ground_speeds, climb_rates] = calc_wps(leg_len, vert_displace, toa_s)
    x1 = [0 0 0];
    x2 = [1 0 0]*leg_len;
    x3 = x2 + [sqrt(leg_len^2 - vert_displace^2), 0, vert_displace];
    
    
    wps = [x1; x2; x3];
    
    v1 = (x2 - x1)/toa_s(2);
    v2 = (x3 - x2)/toa_s(3);
    
    ground_speeds = [ norm(v1(1),v1(2)), norm(v2(1),v2(2)) ];
    climb_rates = [ v1(3), v2(3) ];
end