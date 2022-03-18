function M = LEM_traj2movie(traj)
%

[num_frames,~] = size(traj);
x_min = min(traj(:,1)) - 5;
x_max = max(traj(:,1)) + 5;
y_min = min(traj(:,2)) - 5;
y_max = max(traj(:,2)) + 5;
z_min = min(traj(:,3)) - 5;
z_max = max(traj(:,3)) + 5;

for f = 1:num_frames
    clf
    plot3(x_min,y_min,0,'w.');
    hold on
    axis equal
    plot3(x_max,y_max,0,'w.');
    plot3(x_max,y_max,z_max,'w.');
    plot3(traj(f,1),traj(f,2),traj(f,3),'ko');
    drawnow
    M(f) = getframe(gcf);
end
