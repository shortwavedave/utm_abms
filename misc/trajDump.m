function [s] = trajDump(sim, filename)
%TRAJDUMP Summary of this function goes here
%   Detailed explanation goes here
    res = sim.lbsd.getReservations();
    minTime = min(res.entry_time_s);
    maxTime = max(res.exit_time_s);
    num_steps = floor((maxTime - minTime)/sim.step_rate_hz);
    num_uas = length(sim.uas_list);
%     num_uas = 10;
    s(num_uas).start_time = -1;
    s(num_uas).end_time = -1;
    s(num_uas).pos = [];
    total = num_steps*num_uas;
    for i = 1:num_uas
       s(i).start_time = -1;
       s(i).end_time = -1;
       s(i).pos = -1*ones(num_uas, 4);
       s(i).ind = 1;
%        s = [s u];
    end
    f = waitbar(0,'Please wait...');
    count = 0;
    for i = 1:num_steps
        for j = 1:num_uas
            if mod(count, 10000) == 0
                waitbar(count/total,f,['Loading your data: ', num2str(count),'/',num2str(total)]);
            end
            count = count + 1;
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
%                     s(j).pos = [s(j).pos; t x y z];
                    s(j).pos(s(j).ind,:) = [t x y z];
                    s(j).ind = s(j).ind + 1;
                end
            else
                if s(j).start_time > -1 && s(j).end_time < 0
                    s(j).end_time = i;
                end
            end
        end
    end
    
    % Clean up positions
    for i = 1:num_uas
        s(i).pos(s(i).ind:end,:) = [];
    end
    close(f);
    uas_j = jsonencode(s);
    fid = fopen(filename,'w');
    fprintf(fid, uas_j);
    fclose("all")
end

