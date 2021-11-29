function tbl = process_metrics(folder)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
metric_files = dir(sprintf("%s/%s",folder,"*.mat"));

num_metrics = length(metric_files);
num_types = 16;
% var_types(num_types) = "";
% var_types = arrayfun(@(x) "double", var_types);

% tbl = table('Size',[num_metrics num_types],'VariableNames',[...
%         "density",...
%         "speed",...
%         "hd",...
%         "num_success",...
%         "num_failed",...
%         "total_time",...
%         "delay_mean",...
%         "delay_median",...
%         "delay_max",...
%         "delay_min",...
%         "delay_var",...
%         "res_mean",...
%         "res_median",...
%         "res_max",...
%         "res_min",...
%         "res_var",...
%         ],"VariableTypes", var_types);
    
%       test_conf.net_struct= "grid";
%       test_conf.h_d = headway;
%       test_conf.speed = speed;
%       test_conf.graph_density = 50;
%       test_conf.flex = 5*60;
%       test_conf.sim_time = 4*60*60;
%       test_conf.density = density;
%       test_conf.launch_rate = density/60/60;

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
                            s.struct = net_struct;
                            s.density = mean(t.density);
                            s.speed = mean(t.speed);
                            s.hd = mean(t.hd);
                            s.flex = mean(t.flex);
                            s.sim_time = mean(t.sim_time);
                            s.num_success = mean(t.num_success);
                            s.num_failed = mean(t.num_failed);
                            s.total_time = mean(t.total_time);
                            s.delay_mean = mean(t.delay_mean);
                            s.delay_median = mean(t.delay_median);
                            s.delay_max = mean(t.delay_max);
                            s.delay_min = mean(t.delay_min);
                            s.delay_var = mean(t.delay_var);
                            s.res_mean = mean(t.res_mean);
                            s.res_median = mean(t.res_median);
                            s.res_max = mean(t.res_max);
                            s.res_min = mean(t.res_min);
                            s.res_var = mean(t.res_var);
                            s.occ_mean = mean(t.occ_mean);
                            s.occ_min = mean(t.occ_min);
                            s.occ_max = mean(t.occ_max);
                            s.occ_median = mean(t.occ_median);
                            s_metrics = [s_metrics s];
                        end
                    end
                end
            end
        end
    end
end
tbl = struct2table(s_metrics);
end

