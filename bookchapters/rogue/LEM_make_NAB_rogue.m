function LEM_NAB_sim
%

rng('default');

speed = 1;
del_t = 0.1;
ratio = 1;

roads = LEM_gen_grid_roads(0,50,0,50,10,10);
airways = LEM_gen_airways(roads,[1:10],[21:25],2,50,60);
[path,vpath] = LEM_get_path(airways,275,285,[]);
t = LEM_path2traj(airways,path,1,0.1);
t = LEM_noisy_traj(t,ratio,speed,del_t);
t1 = LEM_hobby_type1([15;45;0],50,5,40,speed,del_t,ratio);
t2 = LEM_hobby_type2([0;40;0],50,5,30,speed,del_t,10,20,ratio);
t3 = LEM_hobby_type3([30;10;0],[35;10;0],60,speed,del_t,ratio);
tr1 = LEM_rogue_type1([0;0;0],[50;50;0],55,speed,del_t,ratio);
launch = airways.lane_vertexes(275,1:3)';
land = airways.lane_vertexes(285,1:3)';
tr2 = LEM_rogue_type2(airways,launch,land,5,3,speed,del_t,ratio);

[m,M] = LEM_traj_analysis_IAS(roads,airways,t,speed,del_t);
[m1,M1] = LEM_traj_analysis_IAS(roads,airways,t1,speed,del_t);
[m2,M2] = LEM_traj_analysis_IAS(roads,airways,t2,speed,del_t);
[m3,M3] = LEM_traj_analysis_IAS(roads,airways,t3,speed,del_t);
[mr1,Mr1] = LEM_traj_analysis_IAS(roads,airways,tr1,speed,del_t);
[mr2,Mr2] = LEM_traj_analysis_IAS(roads,airways,tr2,speed,del_t);
model = LEM_lanes2model(airways.lanes,1);

figure(1);
clf
%LEM_show_airways3D(airways,[]);
pts = model.xyzuvw;
hold on
axis equal
plot3(pts(:,1),pts(:,2),pts(:,3),'k.');
plot3(t1(:,1),t1(:,2),t1(:,3),'b*');
plot3(t2(:,1),t2(:,2),t2(:,3),'m*');
plot3(t3(:,1),t3(:,2),t3(:,3),'g*');
plot3(tr1(:,1),tr1(:,2),tr1(:,3),'c*');
plot3(tr2(:,1),tr2(:,2),tr2(:,3),'r*');
legend('Lanes','Hobby Type I','Hobby Type II','Hobby Type III', 'Rogue Type I',...
    'Rogue Type II','location','Southeast');

figure(2);
clf
subplot(6,2,1);
plot(M1(:,1));
ylabel('Distance to Lanes');
subplot(6,2,2);
plot(M1(:,2));
ylabel('Cosine of Angle');
subplot(6,2,3);
plot(M2(:,1));
ylabel('Distance to Lanes');
subplot(6,2,4);
plot(M2(:,2));
ylabel('Cosine of Angle');
subplot(6,2,5);
plot(M3(:,1));
ylabel('Distance to Lanes');
subplot(6,2,6);
plot(M3(:,2));
ylabel('Cosine of Angle');
subplot(6,2,7);
plot(Mr1(:,1));
ylabel('Distance to Lanes');
subplot(6,2,8);
plot(Mr1(:,2));
ylabel('Cosine of Angle');
subplot(6,2,9);
plot(Mr2(:,1));
ylabel('Distance to Lanes');
subplot(6,2,10);
plot(Mr2(:,2));
ylabel('Cosine of Angle');
subplot(6,2,11);
plot(M(:,1));
ylabel('Distance to Lanes');
subplot(6,2,12);
plot(M(:,2));
ylabel('Cosine of Angle');
xlabel('Time Step');

tch = 0;
