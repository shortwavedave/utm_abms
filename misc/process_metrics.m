function tbl = process_metrics(folder, max_trials)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
metric_files = dir(sprintf("%s/%s",folder,"*.mat"));

num_metrics = length(metric_files);
num_types = 16;

if nargin < 2
    max_trials = inf;
end

s_metrics = [];
for i = 1:num_metrics
    f_i = metric_files(i);
    f = fullfile(f_i.folder, f_i.name);
    m = load(f);
    s.density = m.metric.test_config.density;
    s.speed = m.metric.test_config.speed;
    s.hd = m.metric.test_config.h_d;
    s.flex = m.metric.test_config.flex;
    s.sim_time = m.metric.test_config.sim_time;
    s.struct = m.metric.test_config.net_struct;
    s.num_lanes = length(m.metric.lane_occs);
    s.num_success = m.metric.num_success_flights;
    s.num_failed = m.metric.num_failed_flights;
    s.total_time = m.metric.init_time_traj_s;
    s.delay_mean = m.metric.delay_time.mean;
    s.delay_median = m.metric.delay_time.median;
    s.delay_max = m.metric.delay_time.max;
    s.delay_min = m.metric.delay_time.min;
    s.delay_var = m.metric.delay_time.var;
    s.res_mean = m.metric.reservation_time.mean;
    s.res_median = m.metric.reservation_time.median;
    s.res_max = m.metric.reservation_time.max;
    s.res_min = m.metric.reservation_time.min;
    s.res_var = m.metric.reservation_time.var;
    s.occ_mean = mean([m.metric.lane_occs.occ]);
    s.occ_min = min([m.metric.lane_occs.occ]);
    s.occ_max = max([m.metric.lane_occs.occ]);
    s.occ_median = median([m.metric.lane_occs.occ]);
    s_metrics = [s_metrics s];
end

tbl = struct2table(s_metrics);

densities = unique(tbl.density);
speeds = unique(tbl.speed);
hds = unique(tbl.hd);
flexes = unique(tbl.flex);
sim_times = unique(tbl.sim_time);
structs = unique(tbl.struct);

s_metrics = [];
for density = densities'
    for speed = speeds'
        for hd = hds'
            for flex = flexes'
                for sim_time = sim_times'
                    for net_struct = structs'
                        t = tbl(tbl.density == density & tbl.speed == speed &...
                            tbl.hd == hd & tbl.flex == flex & ...
                            tbl.sim_time == sim_time & ...
                            tbl.struct == net_struct, :);
                        if ~isempty(t)
                            num_trials = height(t);
                            if num_trials > max_trials
                                t = t(1:max_trials,:);
                            end
                            s2.struct = net_struct;
                            s2.num_lanes = t.num_lanes(1);
                            s2.num_trials = height(t);
                            s2.density = mean(t.density);
                            s2.speed = mean(t.speed);
                            s2.hd = mean(t.hd);
                            s2.flex = mean(t.flex);
                            s2.sim_time = mean(t.sim_time);
                            s2.num_success = mean(t.num_success);
                            s2.num_failed = mean(t.num_failed);
                            s2.total_time = mean(t.total_time);
                            s2.delay_mean = mean(t.delay_mean);
                            s2.delay_median = mean(t.delay_median);
                            s2.delay_max = mean(t.delay_max);
                            s2.delay_min = mean(t.delay_min);
                            s2.delay_var = mean(t.delay_var);
                            s2.res_mean = mean(t.res_mean);
                            s2.res_median = mean(t.res_median);
                            s2.res_max = mean(t.res_max);
                            s2.res_min = mean(t.res_min);
                            s2.res_var = mean(t.res_var);
                            s2.occ_mean = mean(t.occ_mean);
                            s2.occ_min = mean(t.occ_min);
                            s2.occ_max = mean(t.occ_max);
                            s2.occ_median = mean(t.occ_median);
                            s_metrics = [s_metrics s2];
                        end
                    end
                end
            end
        end
    end
end
tbl = struct2table(s_metrics);
end

