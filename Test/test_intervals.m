function ok = test_intervals()
%TEST_INTERVALS Summary of this function goes here
%   Detailed explanation goes here
m = [0 2; 2.5, 3; 4 8;10,20;22,35];
ints = DisjointIntervals();
ints.setIntervals(m);
i = ints.union([3,7]);
ok = all(all(i == [0    2.0000;
               2.5000    8.0000;
               10.0000   20.0000;
               22.0000   35.0000]));

if ~ok
    error("FAIL")
end

i = ints.union([11,12]);

ok = all(all(i == [0    2.0000;
                2.5000    8.0000;
               10.0000   20.0000;
               22.0000   35.0000]));
if ~ok
    error("FAIL")
end


i = ints.union([18,21]);

ok = all(all(i == [0    2.0000;
                2.5000    8.0000;
               10.0000   21.0000;
               22.0000   35.0000]));
if ~ok
    error("FAIL")
end

i = ints.union([0,2.2]);

try
ok = all(all(i == [0    2.2;
                2.5000    8.0000;
               10.0000   21.0000;
               22.0000   35.0000]));
catch
    ints.intervals
    error("FAIL")
end
if ~ok
    error("FAIL")
end

i = ints.union([34,36]);

try
ok = all(all(i == [0    2.2;
                2.5000    8.0000;
               10.0000   21.0000;
               22.0000   36.0000]));
catch
    ints.intervals
    error("FAIL")
end
if ~ok
    error("FAIL")
end


i = ints.union([0,37]);

try
ok = all(all(i == [0    37]));
catch
    ints.intervals
    error("FAIL")
end
if ~ok
    error("FAIL")
end

ints = DisjointIntervals();
% ints.setIntervals(m);
i = ints.union([0.3,8]);

try
ok = all(all(i == [0.3    8]));
catch
    ints.intervals
    error("FAIL")
end
if ~ok
    error("FAIL")
end


ints = DisjointIntervals();
% ints.setIntervals(m);
i = ints.union([0.3,8]);

try
ok = all(all(i == [0.3    8]));
catch
    ints.intervals
    error("FAIL")
end
if ~ok
    error("FAIL")
end


ints.intervals
end

