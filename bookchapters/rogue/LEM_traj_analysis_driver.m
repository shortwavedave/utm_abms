function [M,model,traj] = LEM_traj_analysis_driver
%

load  MFI_airways_EB_GIS.mat

roads.vertexes = airways.vertexes;
roads.edges = airways.edges;
traj = LEM_amateur_traj(roads.vertexes,100,1000,0.1,0,0);
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
