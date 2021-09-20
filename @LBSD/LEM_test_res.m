function LEM_test_res
%

reservations(1).flights = [];
reservations(1).hd = 1;
reservations(2).flights = [1,2,3,1; 2,7,8,1];
reservations(2).hd = 1;
%reservations(2).flights = [2,5,6,1];
%reservations(2).hd = 1;
reservations(3).flights = [3,5,6,1];
reservations(3).hd = 1;

path = [1,1;2,1;3,1];
t1 = 1;
t2 = 6;
lane_lengths = [1,1,1];

t = LBSD.LEM_launch_time_nc(reservations,path,t1,t2,lane_lengths,1);

tch = 0;
