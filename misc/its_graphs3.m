function [p1,p2] = its_graphs2()
%ITS_GRAPHS Summary of this function goes here
%   Detailed explanation goes here
reload = false;
if reload
    tbl = process_metrics("output", 10);
    tbl = tbl(tbl.density < 4000,:);
    tbl_out = add_tch_stats2(tbl);
    save("its_stats2","tbl_out");
    tbl = tbl_out;
else
    tbl = load("its_stats2.mat");
    tbl = tbl.tbl_out;
end
% Accessible color pallette ...
% https://xdgov.github.io/data-design-standards/components/colors
colors = [...
   0, 149, 168;...
   17, 46, 81;...
   255, 112, 67;...
   120, 144, 156;...
   46, 120, 210;...
   0, 108, 122;...
   255, 151, 118;...
]/255;

% struct_names = unique(tbl.struct)';
struct_names = ["FAA","slc3","sf3","delaunay","g_delaunay","grid"];
for i = 1:length(struct_names) 
    struct_name = struct_names(i);
%     t = tbl(tbl.struct == struct_name,["density","speed","hd","flex","num_failed","delay_mean","delay_max","res_mean","res_max"]);
    t = tbl(tbl.struct == struct_name,["density","speed","hd","flex","num_failed","delay_mean","res_mean"]);
    v = string(t.Properties.VariableNames);
    v(v=="num_failed") = sprintf("%s failed", struct_name);
    v(v=="delay_mean") = sprintf("%s delay_mean", struct_name);
%     v(v=="delay_max") = sprintf("%s delay_max", struct_name);
    v(v=="res_mean") = sprintf("%s res_mean", struct_name);
%     v(v=="res_max") = sprintf("%s res_max", struct_name);
    t.Properties.VariableNames = cellstr(v);
    t = sortrows(t);
    if i == 1
        tg = t;
    else
        tg = join(tg,t);
    end
end

% Offsets
% types = ["failed","delay_mean","res_mean"];
% offsets = [3, .1, 0];
% densities = [100 1000];
% density_m = [.5, 10];
% for i = 1:length(types)
%     offset = offsets(i);
%     type = types(i);
%     for k = 1:length(densities)
%         density = densities(k);
%         m = density_m(k);
%         for j = 1:length(struct_names)
%             struct_name = struct_names(j);
%             str = sprintf("%s %s", struct_name, type);
%             tg.(str)(tg.density == density) = ...
%                 tg.(str)(tg.density == density)+offset*m*(j-1);
%         end
%     end
% end

% tg.("grid failed")(tg.density == 1000) = tg.("grid failed")(tg.density == 1000);
% tg.("delaunay failed")(tg.density == 1000) = tg.("delaunay failed")(tg.density == 1000)+80;
% tg.("g_delaunay failed")(tg.density == 1000) = tg.("g_delaunay failed")(tg.density == 1000)+40;
% tg.("delaunay delay_mean")(tg.density == 1000) = tg.("delaunay delay_mean")(tg.density == 1000)+.2;
% tg.("grid delay_mean")(tg.density == 1000) = tg.("grid delay_mean")(tg.density == 1000)+.1;
% 
% tg.("delaunay delay_mean")(tg.density == 100) = tg.("delaunay delay_mean")(tg.density == 100)+.02;
% tg.("grid delay_mean")(tg.density == 100) = tg.("grid delay_mean")(tg.density == 100)+.01;
% tg.("delaunay failed")(tg.density == 100) = tg.("delaunay failed")(tg.density == 100)+2;
% tg.("grid failed")(tg.density == 100) = tg.("grid failed")(tg.density == 100)+1;
tg.speed = round(3*(tg.speed-min(tg.speed))/max(tg.speed))+1;
tg.hd = ceil(2*(tg.hd-min(tg.hd))/max(tg.hd))+1;
tg.flex = ceil(2*(tg.flex-min(tg.flex))/max(tg.flex))+1;
tga = tg(tg.density == 100,:);
tgb = tg(tg.density == 1000, :);

vars = {};
i = 1;
% for m_type = ["failed","delay_mean","delay_max","res_mean","res_max"]
for m_type = ["failed","delay_mean","res_mean"]
    strs = [];
    for s_n = struct_names
        strs = [strs sprintf("%s %s", s_n, m_type)];
    end
    vars{i} = strs;
    i = i + 1;
end

vars = {["speed","hd","flex"], vars{:}};

f1 = figure;
f1.Position = [1469 23 1092 1334];
p1 = stackedplot(tga,vars);
p1.Title = ["Simulation Statistics (100 Flights per Hour)"];
f2 = figure;
f2.Position = [1469 23 1092 1334];
p2 = stackedplot(tgb,vars);
p2.Title = ["Simulation Statistics (1000 Flights per Hour)"];

["FAA","slc3","sf3","delaunay","g_delaunay","grid"];
for p = [p1 p2]
    p.LineWidth = 2;
    p.AxesProperties(1).YLimits = [.91 3.8];
    p.AxesProperties(1).LegendLocation = "northwest";
    p.AxesProperties(2).LegendLocation = "northwest";
    p.GridVisible = true;
    p.LineProperties(1).Marker = ["*","o","+"];
    p.LineProperties(2).Marker = ["+",'x','.','^','v','>'];
    p.LineProperties(3).Marker = ["+",'x','.','^','v','>'];
    p.LineProperties(4).Marker = ["+",'x','.','^','v','>'];
    p.AxesProperties(2).LegendLabels = ["FAA", "SLC GIS", "SF GIS", "Delaunay", "Grid-Delaunay", "Grid"];
    p.FontSize = 10;
    for c_i = 1:length(p.LineProperties)
        sz = size(p.LineProperties(c_i).Color);
        c = colors(1:sz(1),:);
        p.LineProperties(c_i).Color = c;
        p.LineProperties(c_i).MarkerEdgeColor = c;
    end
    
    for l_s_i = 2:length(p.LineProperties)
        p.LineProperties(l_s_i).LineStyle = ["-.","-","-","-","-","-"];
    end
%     p.LineProperties(1).PlotType = "stairs";
%     p.LineProperties(2).PlotType = "stairs";
%     p.LineProperties(3).PlotType = "stairs";
%     p.LineProperties(4).PlotType = "stairs";
%     p.DisplayLabels{1} = ["Parameter"; "Index"];
%     p.DisplayLabels{2} = ["Failed"; "Flights"; "(count)"];
%     p.DisplayLabels{3} = ["Mean Delay"; "(seconds)"];
%     p.DisplayLabels{4} = ["Mean"; "Deconfliction"; "(seconds)"];
    p.DisplayLabels{1} = ["Individual Parameter Index (i,j,k)"];
    p.DisplayLabels{2} = ["Failed Flights (count)"];
    p.DisplayLabels{3} = ["Mean Delay (seconds)"];
    p.DisplayLabels{4} = ["Mean Deconfliction (seconds)"];
    p.XLabel = "Parameter Combination Enumeration";
    
    p.AxesProperties(3).LegendVisible = false;
    p.AxesProperties(4).LegendVisible = false;
    ax = findobj(p.NodeChildren, 'Type','Axes');
    text(ax(4),7,3.4,'${(speed_i, hd_j, flex_k) \mid speed_i \in (5,10,15), hd_j \in (5,10,30), flex_k \in (0,300,1800)}$','BackgroundColor','w','Interpreter','latex');
    set(ax,'XTick',1:27);
    set(ax,'GridAlpha',.3);
    for a_i = 1:length(ax)
        ax(a_i).YLabel.Rotation = 90;
        ax(a_i).YLabel.VerticalAlignment = 'bottom';
        ax(a_i).YLabel.HorizontalAlignment = 'center';
    end
    ax(4).YTick = 1:3;
end

annotation(f1,'textarrow',[0.503663003663003 0.489010989010989],...
    [0.761832061068702 0.743511450381679],'String',{'flex = 0'});
annotation(f1,'textarrow',[0.787545787545787 0.805860805860806],...
    [0.768702290076335 0.800763358778626],'String',{'hd = 10'});
annotation(f1,'textarrow',[0.8003663003663 0.785714285714286],...
    [0.89236641221374 0.874045801526718],'String',{'speed = 15'});




exportgraphics(p1,'res100_new.png','Resolution',300);
exportgraphics(p1,'res100_new.eps','Resolution',300);
exportgraphics(p2,'res1000_new.png','Resolution',300);
exportgraphics(p2,'res1000_new.eps','Resolution',300);
% print(f1,'-dpng','-color','res100_new.png')
% print(f2,'-dpng','-color','res1000_new.png')
% print(f1,'-depsc2', 'res100_new.eps')
% print(f2,'-depsc2', 'res1000_new.eps')
end

