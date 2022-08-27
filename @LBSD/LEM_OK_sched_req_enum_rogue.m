function intervals = LEM_OK_sched_req_enum_rogue(obj, ts1,ts2,s_s,tr1,tr2,s_r,d,ht)
% LEM_OK_sched_req_enum - enumeration of possible intervals
% On input:
%     ts1 (float): scheduled flight start
%     ts2 (float): scheduled flight exit
%     s_s (float): speed of scheduled flight to deconflict with
%     tr1 (float): max of min start times
%     tr2 (float): max of max start times
%     s_r (float): speed of current flight
%     d (float): lane length
%     ht (float): headway time
% On output:
%     intervals (): possible intervals
% Call:
%     int1 = LEM_OK_sched_req_enum(0,50,1,10,20,2,1,20);
% Author:
%     T. Henderson
%     UU
%     Fall 2020
%

ZERO_THRESH = 0.001;

persistent first itable

if isempty(first)
    first = 0;
    itable = [...
        1 1 1 1;...  % Case   1
        1 1 1 2;...  % Case   2
        1 1 1 3;...  % Case   3
        1 1 1 4;...  % Case   4
        1 1 1 5;...  % Case   5
        1 1 2 1;...  % Case   6
        1 1 2 2;...  % Case   7
        1 1 2 3;...  % Case   8
        1 1 2 4;...  % Case   9
        1 1 2 5;...  % Case  10
        1 1 3 1;...  % Case  11
        1 1 3 2;...  % Case  12
        1 1 3 3;...  % Case  13
        1 1 3 4;...  % Case  14
        1 1 3 5;...  % Case  15
        1 1 4 1;...  % Case  16
        1 1 4 2;...  % Case  17
        1 1 4 3;...  % Case  18
        1 1 4 4;...  % Case  19
        1 1 4 5;...  % Case  20
        1 1 5 1;...  % Case  21
        1 1 5 2;...  % Case  22
        1 1 5 3;...  % Case  23
        1 1 5 4;...  % Case  24
        1 1 5 5;...  % Case  25
        1 2 1 3;...  % Case  26
        1 2 1 4;...  % Case  27
        1 2 1 5;...  % Case  28
        1 2 2 3;...  % Case  29
        1 2 2 4;...  % Case  30
        1 2 2 5;...  % Case  31
        1 2 3 3;...  % Case  32
        1 2 3 4;...  % Case  33
        1 2 3 5;...  % Case  34
        1 2 4 5;...  % Case  35
        1 2 5 5;...  % Case  36
        1 3 1 3;...  % Case  37
        1 3 1 4;...  % Case  38
        1 3 1 5;...  % Case  39
        1 3 2 3;...  % Case  40
        1 3 2 4;...  % Case  41
        1 3 2 5;...  % Case  42
        1 3 3 3;...  % Case  43
        1 3 3 4;...  % Case  44
        1 3 3 5;...  % Case  45
        1 3 4 5;...  % Case  46
        1 3 5 5;...  % Case  47
        1 4 1 5;...  % Case  48
        1 4 2 5;...  % Case  49
        1 4 3 5;...  % Case  50
        1 4 4 5;...  % Case  51
        1 4 5 5;...  % Case  52
        1 5 1 5;...  % Case  53
        1 5 2 5;...  % Case  54
        1 5 3 5;...  % Case  55
        1 5 4 5;...  % Case  56
        1 5 5 5;...  % Case  57
        2 1 3 1;...  % Case  58
        2 1 3 2;...  % Case  59
        2 1 3 3;...  % Case  60
        2 1 4 1;...  % Case  61
        2 1 4 2;...  % Case  62
        2 1 4 3;...  % Case  63
        2 1 5 1;...  % Case  64
        2 1 5 2;...  % Case  65
        2 1 5 3;...  % Case  66
        2 1 5 4;...  % Case  67
        2 1 5 5;...  % Case  68
        2 2 3 3;...  % Case  69
        2 2 4 4;...  % Case  70
        2 2 5 5;...  % Case  71
        2 3 3 3;...  % Case  72
        2 3 3 4;...  % Case  73
        2 3 3 5;...  % Case  74
        2 3 4 5;...  % Case  75
        2 3 5 5;...  % Case  76
        2 4 3 5;...  % Case  77
        2 4 4 5;...  % Case  78
        2 4 5 5;...  % Case  79
        2 5 3 5;...  % Case  80
        2 5 4 5;...  % Case  81
        2 5 5 5;...  % Case  82
        3 1 3 1;...  % Case  83
        3 1 3 2;...  % Case  84
        3 1 3 3;...  % Case  85
        3 1 4 1;...  % Case  86
        3 1 4 2;...  % Case  87
        3 1 4 3;...  % Case  88
        3 1 5 1;...  % Case  89
        3 1 5 2;...  % Case  90
        3 1 5 3;...  % Case  91
        3 2 3 3;...  % Case  92
        3 2 4 3;...  % Case  93
        3 2 5 3;...  % Case  94
        3 2 5 4;...  % Case  95
        3 2 5 5;...  % Case  96
        3 3 3 3;...  % Case  97
        3 3 3 4;...  % Case  98
        3 3 3 5;...  % Case  99
        3 3 4 3;...  % Case 100
        3 3 4 4;...  % Case 101
        3 3 4 5;...  % Case 102
        3 3 5 3;...  % Case 103
        3 3 5 4;...  % Case 104
        3 3 5 5;...  % Case 105
        3 4 3 5;...  % Case 106
        3 4 4 5;...  % Case 107
        3 4 5 5;...  % Case 108
        3 5 3 5;...  % Case 109
        3 5 4 5;...  % Case 110
        3 5 5 5;...  % Case 111
        4 1 5 1;...  % Case 112
        4 1 5 2;...  % Case 113
        4 1 5 3;...  % Case 114
        4 1 5 4;...  % Case 115
        4 1 5 5;...  % Case 116
        4 2 5 3;...  % Case 117
        4 2 5 4;...  % Case 118
        4 2 5 5;...  % Case 119
        4 3 5 3;...  % Case 120
        4 3 5 4;...  % Case 121
        4 3 5 5;...  % Case 122
        4 4 5 5;...  % Case 123
        4 5 5 5;...  % Case 124
        5 1 5 1;...  % Case 125
        5 1 5 2;...  % Case 126
        5 1 5 3;...  % Case 127
        5 1 5 4;...  % Case 128
        5 1 5 5;...  % Case 129
        5 2 5 3;...  % Case 130
        5 2 5 4;...  % Case 131
        5 2 5 5;...  % Case 132
        5 3 5 3;...  % Case 133
        5 3 5 4;...  % Case 134
        5 3 5 5;...  % Case 135
        5 4 5 5;...  % Case 136
        5 5 5 5];  % Case 137
end

intervals = [];

%t_across = ceil(d/s_r);
t_across = d/s_r;
p1 = ts1 - ht;
p2 = ts1 + ht;
p3 = ts2 + ht;
p4 = ts2 - ht;
q1 = tr1;
q2 = tr2;
q3 = tr2 + t_across;
q4 = tr1 + t_across;

if p1<q1
    i1 = 1;
elseif p1==q1
    i1 = 2;
elseif p1>q1&p1<q2
    i1 = 3;
elseif p1==q2
    i1 = 4;
else
    i1 = 5;
end
if p2<q1
    i3 = 1;
elseif p2==q1
    i3 = 2;
elseif p2>q1&p2<q2
    i3 = 3;
elseif p2==q2
    i3 = 4;
else
    i3 = 5;
end

if p3<q4
    i4 = 1;
elseif p3==q4
    i4 = 2;
elseif p3>q4&p3<q3
    i4 = 3;
elseif p3==q3
    i4 = 4;
else
    i4 = 5;
end
if p4<q4
    i2 = 1;
elseif p4==q4
    i2 = 2;
elseif p4>q4&p4<q3
    i2 = 3;
elseif p4==q3
    i2 = 4;
else
    i2 = 5;
end

index = find(itable(:,1)==i1&itable(:,2)==i2&itable(:,3)==i3...
    &itable(:,4)==i4);
if isempty(index)
    display('Impossible index');
end
if ~isempty(index)
    switch index
        case {1,2,6,7,123,124,136,137}
            intervals = [q1,q2];
        case {3,8,26,29,32,37,40,43}
            intervals = [p3-t_across,q2];
        case {4,9,14,16,17,18,19,27,30,33,38,41,44,61,62,63,86,87,88}
            intervals = [q2,q2];
        case {11,12,58,59,60,83,84,85}
            intervals = [p2,q2];
        case {13}
            if s_s<s_r
                intervals = [p3-t_across,q2];
            else
                intervals = [p2,q2];
            end
        case {69}
            intervals = [p1,q1; p2,q2];
        case {70,73}
            intervals = [p1,q1; q2,q2];
        case {71,74,75,76,77,78,79,80,81,82}
            intervals = [p1,q1];
        case {72}
            intervals = [p1,q1; p3-t_across,q2];
        case {92,93}
            intervals = [q1,q1;q2,q2];
        case {97}
            if s_s<s_r
                intervals = [q1,p1; p3-t_across,q2];
            elseif s_s==s_r
                intervals = [q1,p1; p2,q2];
            else
                intervals = [q1,p4-t_across; p2,q2];
            end
        case {94,95,96,117,118,119,130,131,132}
            intervals = [q1,q1];
        case {98,101}
            intervals = [q1,p1; q2,q2];
        case {100}
            intervals = [p1,p4-t_across; q2,q2];
        case {99,102,106,107,108,109,110,111}
            intervals = [q1,p1];
        case {105}
            if s_s<=s_r
                intervals = [q1,p1];
            else
                intervals = [q1,p4-t_across];
            end
        case {103,104,120,121,122,133,134,135}
            intervals = [q1,p4-t_across];
    end
end

tch = 0;
