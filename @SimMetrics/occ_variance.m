function [occ_var, occ] = occ_variance(metrics)
%OCC_VARIANCE Calculate the variance in occupancy over total simulation
%trials
% On Input:
%   metrics: Array of SimMetrics

    trials = length(metrics);
    occ_var = zeros(trials,1);
    occ = zeros(trials,1);
    for i = 1:trials
        m = [metrics(1:i).lane_occs];
        occs = [m.occ];
        occ(i) = mean(occs);
        occ_var(i) = var(occs);
    end
end

