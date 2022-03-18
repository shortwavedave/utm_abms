function atraj_out = LEM_schedule_artillery(model_ISR,model_manned,...
    model_UAS,airways_ISR,airways_manned,airways_UAS,atraj,theta,hd,t_max,...
    reservations_ISR,reservations_manned,reservations_UAS)
%

g = 9.81;
DELAY = 1;
v0 = 827;

atraj_out = atraj;

ISR_ALT1 = 18000;
ISR_ALT2 = 18200;
MANNED_ALT1 = 4000;
MANNED_ALT2 = 4200;
UAS_ALT1 = 600;
UAS_ALT2 = 620;

[len_traj,~] = size(atraj);
t = atraj(1,4);
done = 0;
while done==0   % check this delay
    done = 1;
    % check ISR
    for k = 1:len_traj  % check every trajectory point against 3 models
        lanes_ISR = [];
        Idx_ISR = cell2mat(rangesearch(model_ISR.kdt,atraj(k,1:3),hd));
        if ~isempty(Idx_ISR)
            lanes_ISR = unique(model_ISR.lane(Idx_ISR));
        end
        Idx_manned = cell2mat(rangesearch(model_manned.kdt,atraj(k,1:3),hd));
        lanes_manned = [];
        if ~isempty(Idx_manned)
            lanes_manned = unique(model_manned.lane(Idx_manned));
        end
        Idx_UAS = cell2mat(rangesearch(model_UAS.kdt,atraj(k,1:3),hd));
        lanes_UAS = [];
        if ~isempty(Idx_UAS)
            lanes_UAS = unique(model_UAS.lane(Idx_UAS));
        end
        OK = 1;
        if ~isempty(lanes_ISR)  % if lane near enough
            num_lanes = length(lanes_ISR);
            OK = 1;
            for p = 1:num_lanes  % check flights through lane
                OK = LEM_lane_OK(reservations_ISR,airways_ISR,lanes_ISR(p),...
                    atraj(k,:),hd);
                if OK==0;
                    done = 0;
                    break
                end
            end
        end
        if ~isempty(lanes_manned)&OK==1  % if lane near enough
            num_lanes = length(lanes_manned);
            OK = 1;
            for p = 1:num_lanes  % check flights through lane
                OK = LEM_lane_OK(reservations_manned,airways_manned.lanes,...
                    lanes_manned(p),atraj(k,:),hd);
                if OK==0;
                    done = 0;
                    break
                end
            end
        end
        if ~isempty(lanes_UAS)&OK==1  % if lane near enough
            num_lanes = length(lanes_UAS);
            OK = 1;
            for p = 1:num_lanes  % check flights through lane
                OK = LEM_lane_OK(reservations_UAS,airways_UAS.lanes,...
                    lanes_UAS(p),atraj(k,:),hd);
                if OK==0;
                    done = 0;
                    break
                end
            end
        end
    end
    if done==0
        t = t + DELAY;
    end
end
atraj_out(:,4) = atraj_out(:,4) + t;
