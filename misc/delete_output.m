function [num_before,num_deleted,num_after] = delete_output(struct_name)
%DELETE_OUTPUT Summary of this function goes here
%   Detailed explanation goes here
metric_files = dir(sprintf("%s/%s","output","*.mat"));
num_metrics = length(metric_files);
num_before = num_metrics;

deleted = 0;
for i = 1:num_metrics
    f_i = metric_files(i);
    f = fullfile(f_i.folder, f_i.name);
    m = load(f);
    if m.metric.test_config.net_struct == struct_name
        delete(f);
        deleted = deleted + 1;
    end
end
num_deleted = deleted;
metric_files = dir(sprintf("%s/%s","output","*.mat"));
num_metrics = length(metric_files);
num_after = num_metrics;
end

