function [tbl_out] = add_tch_stats2(tbl_in)
%ADD_TCH_STATS Summary of this function goes here
%   Detailed explanation goes here
    res1000 = load('res1000a.mat');
    res1000 = res1000.res1000a;
    res100 = load('res100SF.mat');
    res100 = res100.res100n;
%     rs = table2struct(tbl_in);
    tbl_out = tbl_in;
    
    res100 = get_stats(res100);
    res1000 = get_stats1000(res1000);
    
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
                    rs.struct = "sf3";
                    rs.num_lanes = -1;
                    rs.num_trials = 10;
                    rs.density = density;
                    rs.speed = speed;
                    rs.hd = hd;
                    rs.flex = flex;
                    rs.sim_time = -1;
                    rs.num_success = -1;
                    rs.num_failed = r.num_failed(i);
                    rs.total_time = -1;
                    rs.delay_mean = r.delay_mean(i);
                    rs.delay_median = -1;
                    rs.delay_max = r.delay_max(i);
                    rs.delay_min = -1;
                    rs.delay_var = -1;
                    rs.res_mean = r.res_mean(i);
                    rs.res_median = -1;
                    rs.res_max = r.res_max(i);
                    rs.res_min = -1;
                    rs.res_var = -1;
                    rs.occ_mean = -1;
                    rs.occ_min = -1;
                    rs.occ_max = -1;
                    rs.occ_median = -1;
                    rs.mission_time_mean = -1;
                    s = [s;rs];
                end
            end
        end
    end
    tbl_out = [tbl_out;struct2table(s)];
%     tbl_out = struct2table(rs);
end

function t_data = get_stats1000(stats)
    sf_data = [];
    for i = 1:size(stats,1)
        d.num_failed = stats(i,10);
        d.delay_mean = stats(i,6);
        d.delay_max = stats(i,7);
        d.res_mean = stats(i,8);
        d.res_max = stats(i,9);
        sf_data = [sf_data,d];
    end
    t_data = struct2table(sf_data);
end

function t_data = get_stats(data)
    stats = reshape([data.stats],6,[])';
    sf_data = [];
    for i = 1:size(stats,1)
        d.num_failed = stats(i,5);
        d.delay_mean = stats(i,1);
        d.delay_max = stats(i,2);
        d.res_mean = stats(i,3);
        d.res_max = stats(i,4);
        sf_data = [sf_data,d];
    end
    t_data = struct2table(sf_data);
end