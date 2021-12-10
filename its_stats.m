function its_stats()
%ITS_STATS Summary of this function goes here
%   Detailed explanation goes here
    % flex
    reload = false;
    if reload
        tbl = process_metrics("output", 10);
        tbl = tbl(tbl.density < 4000,:);
        tbl_out = add_tch_stats(tbl);
        save("its_stats","tbl_out");
        tbl = tbl_out;
    else
        tbl = load("its_stats.mat");
        tbl = tbl.tbl_out;
    end

    structs = unique(tbl.struct)';
    flexes = unique(tbl.flex)';
    speeds = unique(tbl.speed)';
    densities = unique(tbl.density)';
    
    tbl_structs = [];
    for struct = structs
        flex_means = [];
        f_tbl = tbl(tbl.struct == struct,:);
        for flex = flexes
            m = mean(f_tbl{f_tbl.flex == flex, "delay_mean"});
            flex_means = [flex_means; flex m];
        end
        flex_diffs = flex_means(2:end,:)-flex_means(1:end-1,:);
        flex_diffs = flex_diffs(:,2)./flex_diffs(:,1);
        flex_sense = mean(flex_diffs);
        tbl_struct.struct = struct;
        tbl_struct.flex_sense = flex_sense;
        
        speed_means = [];
        for speed = speeds
            m = mean(f_tbl{f_tbl.speed == speed, "delay_mean"});
            speed_means = [speed_means; speed m];
        end
        speed_diffs = speed_means(2:end,:)-speed_means(1:end-1,:);
        speed_diffs = speed_diffs(:,2)./speed_diffs(:,1);
        speed_sense = mean(speed_diffs);
        tbl_struct.speed_sense = speed_sense;
        
        density_means = [];
        for density = densities
            m = mean(f_tbl{f_tbl.density == density, "delay_mean"});
            density_means = [density_means; density m];
        end
        density_diffs = density_means(2:end,:)-density_means(1:end-1,:);
        density_diffs = density_diffs(:,2)./density_diffs(:,1);
        density_sense = mean(density_diffs);
        tbl_struct.density_sense = density_sense;
        
        tbl_structs = [tbl_structs tbl_struct];
    end

    t = struct2table(tbl_structs);
    
    t.density_sense = t.density_sense / max(abs(t.density_sense));
    t.speed_sense = t.speed_sense / max(abs(t.speed_sense));
    t.flex_sense = t.flex_sense / max(abs(t.flex_sense));
%     scatter(categorical(t.struct), t.flex_sense);
    stackedplot(t,["Flex","Speed","Density"]);
end

