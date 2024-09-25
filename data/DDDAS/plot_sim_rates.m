function [max_rate, min_rate, f] = plot_sim_rates(sim)
%PLOT_SIM_RATES Summary of this function goes here
%   Detailed explanation goes here
mps2fpm = 196.9; % fpm per mps
f = figure;
plot(sim.uas_list(1).exec_times, sim.uas_list(1).exec_vel(:,3)*mps2fpm, "Linewidth",2);
title("Vehicle Climb Rates");
xlabel("time(s)");
ylabel("climb/descend rate (ft/min)");

max_rate = max(sim.uas_list(1).exec_vel(:,3));
min_rate = min(sim.uas_list(1).exec_vel(:,3));
hold on
for i = 2:length(sim.uas_list)
    plot(sim.uas_list(i).exec_times, sim.uas_list(i).exec_vel(:,3), "Linewidth",2);
    t_max_rate = max(sim.uas_list(i).exec_vel(:,3));
    t_min_rate = min(sim.uas_list(i).exec_vel(:,3));
    
    if t_max_rate > max_rate
        max_rate = t_max_rate;
    end
    
    if t_min_rate < min_rate
        min_rate = t_min_rate;
    end
end
hold off

text(50,350, strcat("max rate: ", num2str(max_rate*mps2fpm,6), " ft/min"));
text(50,330, strcat("min rate: ", num2str(min_rate*mps2fpm,6), " ft/min"));

end

