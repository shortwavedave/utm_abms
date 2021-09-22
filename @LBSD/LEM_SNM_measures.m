function measures = LEM_SNM_measures(G,bits)
% LEM_SNM_measures - Spatial Network Measure: compute all measures
% On input:
%     G (Matlab graph struct): undirected graph info
%       .Nodes (nx3 array): vertex locations in cols 1 and 2
%       .Edges (mx2 array): vertex indexes for edges
%     bits (15x1 vector): if bit  is 1, compute kth measure
% On output:
%     measures (measures struct): all measures
%      .degree (nx1 vector): node degree
%      .density (float): |E|/|V|
%      .total_length (float): 1.51*N^0.49
%      .clusering_coef (float): # edges in nei / (ki(k1-1)/2)
%      .min_path_step (nxn array): min cost steps between node pairs
%      .min_path_dist (nxn array): min cost dist between node pairs
%      .graph_diameter (float): max path dist
%      .cyclomatic_num (float): Tau = |E| - |N|+ 1
%      .alpha_index (float): Tau/(2|N|-5)
%      .r_n ((float): (N(1)+N(3)/sum_{k~=2} N(k)
%      .route_factor (nxn array): Q(i,j) = steps/dist
%      .accessibility (nx1 vector): <Q(i)> average access per node
%      .avg_accessibility (float): <Q>
%      .cost_L_T (float): total edge length
%      .cost_L_MST (float): total edge length of MST
%      .cost (float): ratio: cost_L_T / cost_L_MST
%      .efficiency (float): sum(1/L(i,j)) / (N(N-1))
%      .detour_index (mx1 vector): dist / path length for each edge
% Call:
%     m = LEM_SNM_measures(G);
% Author:
%     T. Henderson
%     UU
%     Spring 2021
%

if isempty(G)|max(bits)<1
    measures = [];
    return
end

if bits(1)==1   % degree
    measures.degree = LEM_SNM_degree(G);
end

if bits(2)==1   % density
    measures.density = LEM_SNM_density(G);
end

if bits(3)==1   % total_length
    measures.total_length = LEM_SNM_total_length(G);
end

if bits(4)==1   % clustering coefficient
    measures.clustering_coef = LEM_SNM_clustering_coefs(G);
end

if bits(5)==1   % min path length in steps
    measures.min_path_step = LEM_SNM_min_path_step(G);
end

if bits(6)==1   % min path length in dist
    measures.min_path_dist = LEM_SNM_min_path_dist(G);
end

if bits(7)==1   % graph diameter
    if bits(5)==1
        measures.graph_diameter = max(measures.min_path_step(:));
    else
        measures.graph_diameter = LEM_SNM_graph_diameter(G);
    end
end

if bits(8)==1   % cyclomatic number (Tau = |E| - |N| + 1
    measures.cyclomatic = LEM_SNM_cyclomatic_num(G);
end

if bits(9)==1   % alpha index (Tau/(2|N|-5)
    measures.alpha_index = LEM_SNM_alpha_index(G);
end

if bits(10)==1   % r_n
    measures.r_n = LEM_SNM_r_n(G);
end

if bits(11)==1   % route factor
    measures.route_factor = LEM_SNM_route_factor(G);
end

if bits(12)==1   % accessibility
    [a,aa] = LEM_SNM_accessibility(G);
    measures.accessibility = a;
    measures.avg_accessibility = aa;
end

if bits(13)==1   % cost_L_T
    [L_T,L_MST,L_C] = LEM_SNM_cost_L_T(G);
    measures.L_T = L_T;
    measures.L_MST = L_MST;
    measures.cost = L_C;
end

if bits(14)==1   % efficiency
    efficiency = LEM_SNM_efficiency(G);
    measures.efficiency = efficiency;
end

if bits(15)==1   % detour index
    detour_index = LEM_SNM_detour_index(G);
    measures.detour_index = detour_index;
end

tch = 0;
