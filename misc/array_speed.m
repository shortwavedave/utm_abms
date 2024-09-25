function res2 = array_speed()
%ARRAY_SPEED Summary of this function goes here
%   Detailed explanation goes here
num_trials = 100000;
res1 = Reservations.newReservations(num_trials);
res2.id(num_trials) = "";
res2.lane_id(num_trials) = "";
res2.uas_id(num_trials) = "";
res2.entry_time_s(num_trials) = 0;

str_arr_test(res1, res2, num_trials);
dbl_arr_test(res1, num_trials);

end

function str_arr_test(res1,res2,num_trials)
    test.id(num_trials) = "";
    for i = 1:num_trials
        res2.id(i) = "test";
        res1.id(i) = "test";
        
        test.id(i) = "test";
    end
%     res2.test(num_trials);
end

function dbl_arr_test(res, num_trials)
    for i = 1:num_trials
        res.hd(i) = i;
    end
end