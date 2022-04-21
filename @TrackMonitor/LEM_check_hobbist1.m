function HobbistOne = LEM_check_hobbist1(traj)
% LEM_CLASSIFICATION_HOBBISTONE Detects if the current trajectory fails
%   under the classification of Hobbist One. 
% Input:
%   traj (nx4): x, y, z, t
% Output:
%   HobbistOne (boolean): Indicates if the traj is Hobbist 1
% Call:
%   hobbistOne = LEM_Classification_HobbistOne(traj);
% Author:
%     T. Henderson
%     UU
%     Spring 2021
% 

FLAT_THRESH = 0.2;

HobbistOne = false;
[num_pts, ~] = size(traj);
% diff - Difference and Approximate derivations
% abs - absolute value of the differece
s = sum(abs(diff(traj(1:20:num_pts,3)))<.1);
len = length(diff(traj(1:20:num_pts,3)));
if s/len<FLAT_THRESH
    HobbistOne = true;
end

end

