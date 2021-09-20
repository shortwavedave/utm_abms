function P = LEM_performance(obj,flights)
% LEM_performance - calulates performance measures for LSD results
% On input:
%     flights (flights struct): flights info
% On output:
%     P (nx5 array): performance measures
%       (k,1): flight index
%       (k,2): delay (min requested start time - actual start time)
%       (k,3): flight duration
%       (k,4): deconfliction time (secs)
%       (k,5): 0 if no start time found; else 1
% Call:
%     P1 = LEM_performance(flights);
% Author:
%     T. Henderson
%     UU
%     Fall 2020
%

FAILED = 0;
ZERO_THRESH = 0.001;

num_flights = length(flights);
if num_flights<1
    P = [];
    return
else
    P = zeros(num_flights,5);
end

for f = 1:num_flights
    P(f,1) = f;
    P(f,2) = flights(f).start_time - flights(f).start_interval(1);
    P(f,3) = flights(f).end_time - flights(f).start_time;
    P(f,4) = flights(f).decon_time;
    if flights(f).type==FAILED
        P(f,5) = 1;
    end
    if flights(f).start_interval(2)+ZERO_THRESH<flights(f).start_time
        P(f,5) = 1;
    end
end
