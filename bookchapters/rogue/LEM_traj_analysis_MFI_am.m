function [model,res] = LEM_traj_analysis_MFI_am
%

load  MFI_airways_EB_GIS.mat

roads.vertexes = airways.vertexes;
roads.edges = airways.edges;
traj = LEM_amateur_traj(roads.vertexes,100,1000,0.1,0,0);
model3 = LEM_lanes2model(airways.lanes,2);

model.xyzuvw = model3;
tic
kdt = createns(model3(:,1:3));
tt = toc
model.kdt = kdt;

for k = 1:100
    k
    dx = (0.5-rand)*2000;
    dy = (0.5*rand)*2000;
    trajk = traj;
    n = sqrt(rand*10);
    trajk(:,2) = trajk(:,2) + dx + n*randn(length(trajk(:,2)),1);
    trajk(:,3) = trajk(:,3) + dy + n*randn(length(trajk(:,2)),1);
    M = LEM_traj_measures(model,trajk);
    res(k).traj = trajk;
    res(k).dist = M(:,1);
    res(k).dir = M(:,2);
end
