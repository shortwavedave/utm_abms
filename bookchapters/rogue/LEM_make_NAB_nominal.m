function LEM_make_NAB_nominal
%

rng('default');

roads = LEM_gen_grid_roads(0,50,0,50,10,10);
airways = LEM_gen_airways(roads,[1:10],[21:25],2,50,60);
[path,vpath] = LEM_get_path(airways,275,285,[]);
t = LEM_path2traj(airways,path,1,0.1);
tn = [t(:,1:3) + 0.1*randn(length(t(:,1)),3),t(:,4)];
tns = tn;
tns(:,1) = imfilter(tns(:,1),ones(20,1)/20);
[mn,Mn] = LEM_traj_analysis_IAS(tns);
clf
subplot(3,1,1);
plot(tn(11:210,1),'b');
hold on
plot(tns(11:210,1),'r');
legend('Flight X Values','Smoothed Values','location','Southwest');
ylabel('Flight X Value');
subplot(3,1,2);
plot(Mn(:,1));
ylabel('Distance to Lanes');
subplot(3,1,3);
plot(Mn(:,2));
ylabel('Cosine of Angle');
xlabel('Time Step');
