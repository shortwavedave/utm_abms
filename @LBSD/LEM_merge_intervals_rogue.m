function new_int = LEM_merge_intervals_rogue(obj, int1,int2)
% LEM_merge_intervals - merge two intervals
% On input:
%     int1 (interval): first interval
%     int2 (interval): interval 2
% On output:
%     new_int (interval): merged interval
% Call:
%     ni = LEM_merge_intervals(i1,i2);
% Author:
%     T. Henderson
%     UU
%     Fall 2020
%

if isempty(int1)&isempty(int2)
    new_int = [];
    return
end

new_int = [int1;int2];
[vals,indexes] = sort(new_int(:,1));
new_int = new_int(indexes,:);
change = 1;
while change==1
    change = 0;
    len_new_int = length(new_int(:,1));
    for k = 1:len_new_int-1
        if new_int(k,1)==new_int(k+1,1)
            v_min = min(new_int(k,2),new_int(k+1,2));
            new_int(k+1,2) = v_min;
            new_int(k,:) = [];
            change = 1;
            break
        end
    end
end
