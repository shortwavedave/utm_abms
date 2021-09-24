function [t, lbsd] = LEM_test_res(use_class)
lbsd = [];

if nargin < 1
    use_class = false;
end
%

%first res: flight 1, in at time 2, out at time 3, speed 1
%flight 2; in at time 7, out at time 8, speed 1
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

% Select to either test using the LBSD class or the LEM functions
if use_class
    % Create a simple lane structure, given lane lengths
    lbsd = LBSD.genSimpleLanes(lane_lengths);
    % Convert the lane reservations to the LBSD format
    for i = 1:length(reservations)
        lane_id = string(i);
        hd = reservations(i).hd;
        flts = reservations(i).flights;
        for j = 1:size(flts,1)
            entry_time = flts(j,2);
            exit_time = flts(j,3);
            speed = flts(j,4);
            [ok, res_id] = lbsd.makeReservation(lane_id, ...
                entry_time, exit_time, speed, hd);
            if ~ok
                error(["Problem scheduling reservation " res_id])
            end
        end
    end
    
    % Now schedule the test trajectory
    % Convert the LEM path desctiption to time-of-arrival format
    toa_s = [0];
    lane_ids = [];
    for i = 1:size(path,1)
        lane_ids = [lane_ids; string(path(i,1))];
        s = path(i,2);
        l = lane_lengths(path(i,1));
        toa_s = [toa_s toa_s(i)+l/s];
    end
    h_d = reservations(1).hd;
    toa_s = toa_s + t1;
    r_e = t1;
    r_l = t2;
    [ok, res_ids, res_toa_s] = ...
                lbsd.reserveLBSDTrajectory(lane_ids, toa_s, h_d, r_e, r_l);
    t = res_toa_s(1);
else
    t = LBSD.LEM_launch_time_nc(reservations,path,t1,t2,lane_lengths,1);
end
tch = 0;
