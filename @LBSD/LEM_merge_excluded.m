function new_int = LEM_merge_excluded(t1,t2,int1,int2)
% LEM_merge_excluded - merge two intervals inside [t1,t2]
% On input:
%     t1 (float): min value
%     t2 (float): max value
%     int1 (interval): first interval
%     int2 (interval): interval 2
% On output:
%     new_int (interval): merged interval
% Call:
%     ni = LEM_merge_excluded(t1,t2,i1,i2);
% Author:
%     T. Henderson
%     UU
%     Fall 2021
%

if isempty(int1)&isempty(int2)
    new_int = [];
    return
end

if int1(1)<t1
    int1(1) = t1;
end
if int1(2)>t2
    int1(2) = t2;
end
if int2(1)<t1
    int2(1) = t1;
end
if int2(2)>t2
    int2(2) = t2;
end

if int1(2)<int2(1)
    new_int = [int1;int2];
elseif int2(1)>=int1(1)&int2(1)<=int1(2)&int1(2)<=int2(2)
    new_int = [int1(1),int2(2)];
elseif int2(1)>=int1(1)&int2(1)<int1(2)&int2(2)<int1(2)
    new_int = int1;
elseif int2(1)<int1(1)&int1(2)<int2(2)
    new_int = int2;
elseif int2(1)<int1(1)&int2(2)>=int1(1)&int2(2)<int1(2)
    new_int = [int2(1),int1(2)];
else
    new_int = [int2;int1];
end
