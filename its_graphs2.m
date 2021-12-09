function [p1,p2] = its_graphs2()
%ITS_GRAPHS Summary of this function goes here
%   Detailed explanation goes here
tbl = load("its_stats.mat");
tbl = tbl.tbl_out;
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

struct_names = unique(tbl.struct)';
for i = 1:length(struct_names) 
    struct_name = struct_names(i);
%     t = tbl(tbl.struct == struct_name,["density","speed","hd","flex","num_failed","delay_mean","delay_max","res_mean","res_max"]);
    t = tbl(tbl.struct == struct_name,["density","speed","hd","flex","delay_max","res_max"]);
    v = string(t.Properties.VariableNames);
    v(v=="num_failed") = sprintf("%s failed", struct_name);
%     v(v=="delay_mean") = sprintf("%s delay_mean", struct_name);
    v(v=="delay_max") = sprintf("%s delay_max", struct_name);
%     v(v=="res_mean") = sprintf("%s res_mean", struct_name);
    v(v=="res_max") = sprintf("%s res_max", struct_name);
    t.Properties.VariableNames = cellstr(v);
    t = sortrows(t);
    if i == 1
        tg = t;
    else
        tg = join(tg,t);
    end
end
tg.speed = round(3*(tg.speed-min(tg.speed))/max(tg.speed))+1;
tg.hd = ceil(2*(tg.hd-min(tg.hd))/max(tg.hd))+1;
tg.flex = ceil(2*(tg.flex-min(tg.flex))/max(tg.flex))+1;
tga = tg(tg.density == 100,:);
tgb = tg(tg.density == 1000, :);

vars = {};
i = 1;
% for m_type = ["failed","delay_mean","delay_max","res_mean","res_max"]
for m_type = ["delay_max","res_max"]
    strs = [];
    for s_n = struct_names
        strs = [strs sprintf("%s %s", s_n, m_type)];
    end
    vars{i} = strs;
    i = i + 1;
end

vars = {["speed","hd","flex"], vars{:}};

f1 = figure;
% f1.Position = [1469 23 1092 1334];
f1.Position = [1469 23 1092 982];
% 508,344,1099,982
p1 = stackedplot(tga,vars);
p1.Title = ["Simulation Statistics (100 Flights per Hour)"];
f2 = figure;
% f2.Position = [1469 23 1092 1334];
f2.Position = [1469 23 1092 982];
p2 = stackedplot(tgb,vars);
p2.Title = ["Simulation Statistics (1000 Flights per Hour)"];

for p = [p1 p2]
    p.LineWidth = 2;
    p.AxesProperties(1).YLimits = [.93 3.8];
%     p.AxesProperties(1).YTick = [1:3];
    p.AxesProperties(1).LegendLocation = "northwest";
    p.AxesProperties(2).LegendLocation = "northwest";
    p.GridVisible = true;
    p.LineProperties(1).Marker = ["*","o","+"];
    p.LineProperties(2).Marker = ["+",'x','.','^','v'];
    p.LineProperties(3).Marker = ["+",'x','.','^','v'];
%     p.LineProperties(4).Marker = "/+";
    p.AxesProperties(2).LegendLabels = ["FAA", "Delaunay", "Grid-Delaunay", "GIS", "Grid"];
    p.FontSize = 10;
    for c_i = 1:length(p.LineProperties)
        sz = size(p.LineProperties(c_i).Color);
        c = colors(1:sz(1),:);
        p.LineProperties(c_i).Color = c;
        p.LineProperties(c_i).MarkerEdgeColor = c;
    end
    
    for l_s_i = 2:length(p.LineProperties)
        p.LineProperties(l_s_i).LineStyle = ["-.","-","-","-","-"];
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
%     p.DisplayLabels{2} = ["Failed Flights (count)"];
    p.DisplayLabels{2} = ["Max Delay (seconds)"];
    p.DisplayLabels{3} = ["Max Deconfliction (seconds)"];
    p.XLabel = "Parameter Combination Enumeration";
    
    p.AxesProperties(3).LegendVisible = false;
%     p.AxesProperties(4).LegendVisible = false;
    ax = findobj(p.NodeChildren, 'Type','Axes');   
    text(ax(3),7,3.4,'${(speed_i, hd_j, flex_k) \mid speed_i \in (5,10,15), hd_j \in (5,10,30), flex_k \in (0,300,1800)}$','BackgroundColor','w','Interpreter','latex');
    set(ax,'XTick',1:27);
    set(ax,'GridAlpha',.3);
    ax(3).YTick = 1:3;
    for a_i = 1:length(ax)
        ax(a_i).YLabel.Rotation = 90;
        ax(a_i).YLabel.VerticalAlignment = 'bottom';
        ax(a_i).YLabel.HorizontalAlignment = 'center';
    end
%     ax(4).YTick = 0:2;
end

% annotation(f1,'textarrow',[0.368131868131868 0.344322344322344],...
%     [0.886259541984733 0.861832061068702],'String',{'hd = 30'});
% annotation(f1,'textarrow',[0.482600732600733 0.458791208791209],...
%     [0.885496183206107 0.861068702290076],'String',{'flex = 1800'});
% annotation(f1,'textarrow',[0.69047619047619 0.666666666666666],...
%     [0.883969465648855 0.859541984732824],'String',{'speed = 15'});

exportgraphics(p1,'res100_max.png','Resolution',300);
exportgraphics(p1,'res100_max.eps','Resolution',300);
% exportgraphics(p1,'res100_new.eps');
exportgraphics(p2,'res1000_max.png','Resolution',300);
exportgraphics(p2,'res1000_max.eps','Resolution',300);
% exportgraphics(p2,'res1000_new.eps');
% print(f1,'-dpng','-color','res100_new.png')
%       
% print(f2,'-dpng','-color','res1000_new.png')
% print(f1,'-depsc2', 'res100_new.eps')
% print(f2,'-depsc2', 'res1000_new.eps')
end

