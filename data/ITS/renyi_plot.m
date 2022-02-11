function renyi_plot(metrics)
%RENYI_PLOT Summary of this function goes here
%   Detailed explanation goes here
line_width = 2;
[occ_var, occ] = SimMetrics.occ_variance(metrics);
num_trials = length(metrics);
renyi=0.7475979202;
figure;
subplot(2,1,1);
h = plot(occ);
h.LineWidth = line_width;
hold on
h2 = plot(1:num_trials,renyi*ones(1,num_trials),'r');
h2.LineWidth = line_width;
legend('Occupancy',"Renyi's Constant");
xlabel('Trials');
ylabel('Occupancy');
hold off

subplot(2,1,2);
h = plot(sqrt(occ_var));
h.LineWidth = line_width;
xlabel('Trials');
ylabel('Occ. Standard Deviation');
end

