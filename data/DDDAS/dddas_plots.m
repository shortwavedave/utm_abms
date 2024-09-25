function dddas_plots()
%DDDAS_PLOTS Summary of this function goes here
%   Detailed explanation goes here
workingpath = erase(mfilename('fullpath'), mfilename());

folder = strcat(workingpath, "plots");

disp(strcat("Saving plots to ", folder));

lbsd = LBSD.genSampleLanes(10, 15);
uas = UAS();
uas.lbsd = lbsd;
[traj, ~, ~, ~] = uas.createTrajectory([-20,-20],[20,20]);

h = lbsd.plot();
hold on;
traj.plot();
hold off;

axes1 = gca;

view(axes1,[-167.791981923665 33.4028540515761]);
box(axes1,'on');
axis(axes1,'tight');
hold(axes1,'off');
% Set the remaining axes properties
set(axes1,'DataAspectRatio',[1 1 1]);
legend("lane system","trajectory", 'Location', 'north', 'NumColumns', 2);
h.LineWidth = 4;

exportgraphics(axes1,strcat(folder, '/traj.png'),'Resolution',300);
exportgraphics(axes1,strcat(folder, '/traj.eps'),'Resolution',300);



tch = 0;
end

