function [tbl_out] = add_tch_stats(tbl_in)
%ADD_TCH_STATS Summary of this function goes here
%   Detailed explanation goes here
    res1000 = load('res1000.mat');
    res1000 = res1000.res1000;
    opts = detectImportOptions("res100.txt");
    opts.VariableNames = ["delay_mean","delay_max","res_mean",...
        "res_max","num_failed_mean","num_failed_max"];
    res100 = readmatrix("res100.txt", opts);
%     rs = table2struct(tbl_in);
    tbl_out = tbl_in;
    s = [];
    for density = [100, 1000]
        if density == 100
            r = res100;
        else
            r = res1000;
        end
        i = 0;
        for speed = [5 10 15]
            for hd = [5 10 30]
                for flex = [0 300 1800]
                    i = i + 1;
%                     rs(end+1) = rs(end);
                    rs.struct = "FAA";
                    rs.num_lanes = -1;
                    rs.num_trials = 10;
                    rs.density = density;
                    rs.speed = speed;
                    rs.hd = hd;
                    rs.flex = flex;
                    rs.sim_time = -1;
                    rs.num_success = -1;
                    rs.num_failed = r(i,5);
                    rs.total_time = -1;
                    rs.delay_mean = r(i,1);
                    rs.delay_median = -1;
                    rs.delay_max = r(i,2);
                    rs.delay_min = -1;
                    rs.delay_var = -1;
                    rs.res_mean = r(i,3);
                    rs.res_median = -1;
                    rs.res_max = r(i,4);
                    rs.res_min = -1;
                    rs.res_var = -1;
                    rs.occ_mean = -1;
                    rs.occ_min = -1;
                    rs.occ_max = -1;
                    rs.occ_median = -1;
                    s = [s;rs];
                end
            end
        end
    end
    tbl_out = [tbl_out;struct2table(s)];
%     tbl_out = struct2table(rs);
end

