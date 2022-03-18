function [M,model,traj] = LEM_traj_analysis_driver_rogue
%

load  MFI_airways_EB_GIS.mat
speed = 10;
altitude = 500;
del_t = 0.1;

V = airways.vertexes;
[num_vertexes,~] = size(V);
index1 = randi(num_vertexes);
index2 = randi(num_vertexes);
route = zeros(3,8);
height1 = V(index1,3) + altitude;
height2 = V(index2,3) + altitude;
t11 = 0;
t12 = altitude/speed;
t1 = t12 - t11;
num_pts1 = floor(t1/del_t);
t21 = t12;
t22 = t21 + norm(V(index2,1:2)-V(index1,1:2))/speed;
t2 = t22 - t21;
num_pts2 = floor(t2/del_t);
t31 = t22;
t32 = t31 + altitude/speed;
t3 = t12 - t11;
num_pts3 = floor(t3/del_t);
pt1 = V(index1,1:3);
pt2 = [V(index1,1:2),height1];
pt3 = [V(index2,1:2),height2];
pt4 = V(index2,:);
x1 = linspace(pt1(1),pt2(1),num_pts1);
y1 = linspace(pt1(2),pt2(2),num_pts1);
z1 = linspace(pt1(3),pt2(3),num_pts1);
x2 = linspace(pt2(1),pt3(1),num_pts2);
y2 = linspace(pt2(2),pt3(2),num_pts2);
z2 = linspace(pt2(3),pt3(3),num_pts2);
x3 = linspace(pt3(1),pt4(1),num_pts3);
y3 = linspace(pt3(2),pt4(2),num_pts3);
z3 = linspace(pt3(3),pt4(3),num_pts3);
traj = [[x1,x2,x3]',[y1,y2,y3]',[z1,z2,z3]'];
tt = [0:0.1:t32]';
tt = tt(1:length(traj(:,1)));
traj = [tt,traj];

model3 = LEM_lanes2model(airways.lanes,2);

model.xyzuvw = model3;
kdt = createns(model3(:,1:3));
model.kdt = kdt;

M = LEM_traj_measures(model,traj);
figure(1);
clf
plot(M(:,1));
figure(2);
clf
plot(M(:,2));
