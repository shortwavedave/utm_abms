function traj = traj_plot(plot_traj)
%TRAJ_PLOT Summary of this function goes here
%   Detailed explanation goes here
mps2fpm = 196.9; % fpm per mps
m2ft = 3.281; % ft per m
fpm2mps = 1/mps2fpm;
ft2m = 1/m2ft;
knots2mps = 1/1.944;

if nargin < 1
    plot_traj = true;
end
workingpath = erase(mfilename('fullpath'), mfilename());
M = readmatrix(strcat(workingpath, "traj.csv"));
traj = M;
traj(:,2) = traj(:,2) - min(traj(:,2)) - 800;
traj(:,3) = traj(:,3) - min(traj(:,3)) - 800;
traj(:,4) = traj(:,4) - 4100;

if plot_traj
    f = figure;
    subplot(1,2,1);
    plot(traj(:,2),traj(:,3), "k", "Linewidth", 2);
    xlabel("x(m)");
    ylabel("y(m)");
    title("Mission Path");

    subplot(1,2,2);
    plot(traj(:,1),traj(:,4)*ft2m, "k", "Linewidth", 2);
    xlabel("t(s)");
    ylabel("z(m)");
    title("Altitude Above Ground");

    % subplot(2,2,3);
    % title("X");
    % plot(traj(:,1),traj(:,2));
    % 
    % subplot(2,2,4);
    % title("Y");
    % plot(traj(:,1),traj(:,3));


    folder = strcat(workingpath, "plots");

    exportgraphics(f,strcat(folder, '/adsb_traj.png'),'Resolution',300);
    exportgraphics(f,strcat(folder, '/adsb_traj.eps'),'Resolution',300);
end
end

