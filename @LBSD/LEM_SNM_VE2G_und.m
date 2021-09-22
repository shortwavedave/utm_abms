function G = LEM_SNM_VE2G_und(V,E)
% LEM_SNM_VE2G_und - Spatial Network Measure: create undirected graph
% On input:
%     V (nx2 array): vertex locations in plane
%     E (mx2 array): vertex indexes for edges
% On output:
%     G (Matlab undirected graph): graph of V and E
%      .Nodes (nx3 table): vertex locations
%      .Edges (mx3 table): vertex indexes for edges; weight
% Call:
%     G = LEM_SNM_VE2G_und(V,E);
% Author:
%     T. Henderson
%     UU
%     Spring 2021
%

if isempty(E)
    G = [];
    return
end

[num_V,dummy] = size(V);
[num_E,dummy] = size(E);
A = zeros(num_V,num_V);
for e = 1:num_E
    A(E(e,1),E(e,2)) = norm(V(E(e,1),1:2)-V(E(e,2),1:2));
    A(E(e,2),E(e,1)) = A(E(e,1),E(e,2));
end
T = array2table(V);
G = graph(A,T);

