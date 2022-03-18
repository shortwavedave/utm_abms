function [model,M] = LEM_traj_analysis_IAS(roads,airways,traj,speed,del_t)
%

model = LEM_lanes2model(airways.lanes,2);

kdt = createns(model.xyzuvw(:,1:3));
model.kdt = kdt;
M = LEM_traj_measures(airways,model,traj,speed,del_t);
