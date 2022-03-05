function [s] = trajDump(sim, filename)
%TRAJDUMP Summary of this function goes here
%   Detailed explanation goes here
    res = sim.lbsd.getReservations();
    minTime = min(res.entry_time_s);
    maxTime = max(res.exit_time_s);
    num_steps = floor((maxTime - minTime)/sim.step_rate_hz);
    num_uas = length(sim.uas_list);
    s = [];
    for i = 1:num_uas
       u.start_time = -1;
       u.end_time = -1;
       u.pos = [];
       s = [s u];
    end
    
    for i = 1:num_steps
        for j = 1:num_uas
            uas = sim.uas_list(j);
            uas_step = uas.stepTrajectory();
            if uas.active
                if s(j).start_time < 0
                    s(j).start_time = i;
                end
                pos = uas.exec_traj;
                if ~isempty(pos)
                    t = i;
                    x = pos(uas_step, 1);
                    y = pos(uas_step, 2);
                    z = pos(uas_step, 3);
                    s(j).pos = [s(j).pos; t x y z];
                end
            else
                if s(j).start_time > -1 && s(j).end_time < 0
                    s(j).end_time = i;
                end
            end
        end
    end
    uas_j = jsonencode(s);
    fid = fopen(filename,'w');
    fprintf(fid, uas_j);
    fclose("all")
end

