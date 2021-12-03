classdef LBSDDeconfliction < matlab.unittest.TestCase
    %LBSDDECONFLICTION Summary of this class goes here
    %   Detailed explanation goes here
    
    methods (Static)
        function lbsd = singleLaneSetup()
            % A single lane of length 1000 meters
            % The lane id is "1"
            lbsd = LBSD.genSimpleLanes(1000);
        end
        
        function verifyWithinTol(testCase, actual, expected)
            import matlab.unittest.constraints.IsEqualTo
            import matlab.unittest.constraints.AbsoluteTolerance
            import matlab.unittest.constraints.RelativeTolerance
            tol = 10*eps;
            testCase.verifyThat(actual,IsEqualTo(expected, ...
                'Within',AbsoluteTolerance(tol) | RelativeTolerance(tol)));
        end
        
        function verifyTableEquality(testCase, actual, expected)
            sz1 = size(actual);
            sz2 = size(expected);
            LBSDDeconfliction.verifyWithinTol(testCase,sz1,sz2);
            for r = 1:sz1(1)
                for c = 1:sz1(2)
                    a = actual{r,c};
                    e = expected{r,c};
                    LBSDDeconfliction.verifyWithinTol(testCase,a,e);
                end
            end
        end
        
        function verifyReservation(testCase, res_table, res_id, lane_id,...
                    uas_id, entry_time_s, exit_time_s, speed, hd)
                if size(res_id,2)>size(res_id,1)
                    res_id = res_id';
                end
                r = res_table(ismember(res_table.id, res_id),:);
                for i = 1:size(r,1)
                    testCase.verifyEqual(r.lane_id(i), lane_id(i));
                    testCase.verifyEqual(r.uas_id(i), uas_id);
                    testCase.verifyWithinTol(testCase, r.entry_time_s(i), ...
                        entry_time_s(i));
                    testCase.verifyWithinTol(testCase, r.exit_time_s(i), ...
                        exit_time_s(i));
                    testCase.verifyWithinTol(testCase, r.speed(i), speed);
                    testCase.verifyWithinTol(testCase, r.hd(i), hd);
                end
        end
    end
    
    methods(Test)
        function makeReservations(testCase)
            %makeReservation test making a reservation
            num_uas = 10;
            
            lbsd = LBSDDeconfliction.singleLaneSetup();
            lane_id = "1";
            lane_length = lbsd.getLaneLengths("1");
            speed = 10; % m/s
            entry_time_s = 0;
            exit_time_s = lane_length/speed;
            hd = 10;
            ht = hd/speed;
            res_ids(num_uas) = "";
            ent_s = entry_time_s;
            r_e = ent_s; % earliest release time
            ext_s = exit_time_s;
            for uas_i = 1:num_uas
                uas_id = string(uas_i);
                [ok, res_id] = lbsd.makeReservation(lane_id, ent_s, ...
                    ext_s, speed, hd, uas_id);
                
                testCase.verifyTrue(ok);
                res_ids(uas_i) = res_id;
                ent_s = ent_s + ht;
                ext_s = ext_s + ht;
            end
            e_l = ext_s; % Latest exit time
            
            % First check if the reservation table is the same as the
            % reservation table for lane "1"
            res = lbsd.getReservations();
            
            res_test = lbsd.getLaneReservations(lane_id);
            LBSDDeconfliction.verifyTableEquality(testCase, res_test, res);
            
            res_test_i = lbsd.getLaneResInds(lane_id, r_e, e_l);
            LBSDDeconfliction.verifyTableEquality(testCase, ...
                res(res_test_i,:), res);
            
            ent_s = entry_time_s;
            ext_s = exit_time_s;
            for uas_i = 1:num_uas
                res_id = res_ids(uas_i);
                
                LBSDDeconfliction.verifyReservation(testCase, res, ...
                    res_id,lane_id, string(uas_i), ent_s, ext_s, speed, hd);
                ent_s = ent_s + ht;
                ext_s = ext_s + ht;
            end
        end
        
        function reserveLBSDTrajectory_one(testCase)
            lbsd = LBSDDeconfliction.singleLaneSetup();
            lane_id = "1";
            lane_length = lbsd.getLaneLengths("1");
            speed = 10; % m/s
            entry_time_s = 0;
            exit_time_s = lane_length/speed;
            hd = 10;
            toa_s = [entry_time_s, exit_time_s]';
            r_e = entry_time_s;
            r_l = entry_time_s;
            uas_id = "1";
            [ok, res_ids, res_toa_s] = ...
                lbsd.reserveLBSDTrajectory(lane_id, uas_id, toa_s, ...
                hd, r_e, r_l);
            testCase.verifyTrue(ok);
            testCase.verifyWithinTol(testCase, res_toa_s, toa_s);
            
            res = lbsd.getReservations();
            LBSDDeconfliction.verifyReservation(testCase, res, ...
                    res_ids,lane_id, uas_id, entry_time_s, exit_time_s,...
                    speed, hd);
        end
                
        function reserveLBSDTrajectory_two_sp_eq_no_flex(testCase)
            % reserveLBSDTrajectory_two_sp_eq_no_flex Test two resevations
            % when both speeds are equal and no flexibility
            lbsd = LBSDDeconfliction.singleLaneSetup();
            lane_id = "1";
            lane_length = lbsd.getLaneLengths("1");
            speed = 10; % m/s
            entry_time_s = 0;
            exit_time_s = lane_length/speed;
            hd = 10;
            ht = hd/speed;
            toa_s = [entry_time_s, exit_time_s]';
            r_e = entry_time_s;
            r_l = entry_time_s;
            uas1_id = "1";
            uas2_id = "2";
            
            % Reserve one spot
            [ok, res_ids, res_toa_s] = ...
                lbsd.reserveLBSDTrajectory(lane_id, uas1_id, toa_s, ...
                hd, r_e, r_l);
            testCase.verifyTrue(ok);
            testCase.verifyWithinTol(testCase, res_toa_s, toa_s);
            
            res = lbsd.getReservations();
            LBSDDeconfliction.verifyReservation(testCase, res, ...
                    res_ids,lane_id, uas1_id, entry_time_s, exit_time_s,...
                    speed, hd);
                
            
            % This case should fail since there is no flexibility
            [ok, ~, ~] = ...
                lbsd.reserveLBSDTrajectory(lane_id, uas2_id, toa_s, ...
                hd, r_e, r_l);
            testCase.verifyFalse(ok);
            
        end
        
        function reserveLBSDTrajectory_two_sp_eq_flex(testCase)
            % reserveLBSDTrajectory_two_sp_eq_no_flex Test two resevations
            % when both speeds are equal and no flexibility
            lbsd = LBSDDeconfliction.singleLaneSetup();
            lane_id = "1";
            lane_length = lbsd.getLaneLengths("1");
            speed = 10; % m/s
            entry_time_s = 0;
            exit_time_s = lane_length/speed;
            hd = 10;
            ht = hd/speed;
            toa_s = [entry_time_s, exit_time_s]';
            r_e = entry_time_s;
            r_l = entry_time_s;
            uas1_id = "1";
            uas2_id = "2";
            
            % Reserve one spot
            [ok, res_ids, res_toa_s] = ...
                lbsd.reserveLBSDTrajectory(lane_id, uas1_id, toa_s, ...
                hd, r_e, r_l);
            testCase.verifyTrue(ok);
            testCase.verifyWithinTol(testCase, res_toa_s, toa_s);
            
            res = lbsd.getReservations();
            LBSDDeconfliction.verifyReservation(testCase, res, ...
                    res_ids,lane_id, uas1_id, entry_time_s, exit_time_s,...
                    speed, hd);
                
            
            % This case should fail since there is no flexibility
            [ok, ~, ~] = ...
                lbsd.reserveLBSDTrajectory(lane_id, uas2_id, toa_s, ...
                hd, r_e, r_l);
            testCase.verifyFalse(ok);
            
            % Adding flexibility in positive direction. 
            [ok, res_ids, res_toa_s] = ...
                lbsd.reserveLBSDTrajectory(lane_id, uas2_id, toa_s, ...
                hd, r_e, r_l+2*ht);
            testCase.verifyTrue(ok);
            %  Expect the reservation immediately after the other aircraft
            exp_res = toa_s + ht;
            testCase.verifyWithinTol(testCase, res_toa_s, exp_res);
            res = lbsd.getReservations();
            LBSDDeconfliction.verifyReservation(testCase, res, ...
                    res_ids,lane_id, uas2_id, entry_time_s+ht, exit_time_s+ht,...
                    speed, hd);
            
                
            % Adding flexibility in negative direction. 
            [ok, res_ids, res_toa_s] = ...
                lbsd.reserveLBSDTrajectory(lane_id, uas2_id, toa_s, ...
                hd, r_e-2*ht, r_l);
            testCase.verifyTrue(ok);
            %  Expect the reservation immediately after the other aircraft
            exp_res = toa_s - ht;
            testCase.verifyWithinTol(testCase, res_toa_s, exp_res);
            res = lbsd.getReservations();
            LBSDDeconfliction.verifyReservation(testCase, res, ...
                    res_ids,lane_id, uas2_id, entry_time_s-ht, exit_time_s-ht,...
                    speed, hd);
                
            for t = 2:100
                % Adding flexibility in positive direction. 
                [ok, res_ids, res_toa_s] = ...
                    lbsd.reserveLBSDTrajectory(lane_id, uas2_id, toa_s, ...
                    hd, r_e, r_l+t+10);
                testCase.verifyTrue(ok);
                %  Expect the reservation immediately after the other aircraft
                exp_res = toa_s + t;
                testCase.verifyWithinTol(testCase, res_toa_s, exp_res);
                res = lbsd.getReservations();
                LBSDDeconfliction.verifyReservation(testCase, res, ...
                        res_ids,lane_id, uas2_id, entry_time_s+t, exit_time_s+t,...
                        speed, hd);
            end
            
        end
        
        function reserveLBSDTrajectory_two_speeds_p(testCase)
            % reserveLBSDTrajectory_two_sp_eq_no_flex Test two resevations
            % when both speeds are equal and no flexibility
            lbsd = LBSDDeconfliction.singleLaneSetup();
            lane_id = "1";
            lane_length = lbsd.getLaneLengths("1");
            speed = 10; % m/s
            entry_time_s = 0;
            exit_time_s = lane_length/speed;
            hd = 10;
            ht = hd/speed;
            toa_s = [entry_time_s, exit_time_s]';
            r_e = entry_time_s;
            r_l = entry_time_s;
            uas1_id = "1";
            uas2_id = "2";
            
            % Reserve one spot
            [ok, res_ids, res_toa_s] = ...
                lbsd.reserveLBSDTrajectory(lane_id, uas1_id, toa_s, ...
                hd, r_e, r_l);
            testCase.verifyTrue(ok);
            testCase.verifyWithinTol(testCase, res_toa_s, toa_s);
            
            res = lbsd.getReservations();
            LBSDDeconfliction.verifyReservation(testCase, res, ...
                    res_ids,lane_id, uas1_id, entry_time_s, exit_time_s,...
                    speed, hd);
                
            
            % This case should fail since there is no flexibility
            speed2 = 2*speed;
            exit_time_s_2 = lane_length/speed2;
            ht2 = hd/speed2;
            toa_s2 = [entry_time_s, exit_time_s_2]';
            
            [ok, ~, ~] = ...
                lbsd.reserveLBSDTrajectory(lane_id, uas2_id, toa_s2, ...
                hd, r_e, r_l);
            testCase.verifyFalse(ok);
            
            % Adding flexibility in positive direction. 
            ht_m = max(ht,ht2);
            [ok, res_ids, res_toa_s] = ...
                lbsd.reserveLBSDTrajectory(lane_id, uas2_id, toa_s2, ...
                hd, r_e, r_l+100);
            testCase.verifyTrue(ok);
            %  Expect the reservation immediately after the other aircraft
            start = (toa_s(end)+ht_m)-lane_length/speed2;
            exp_res = toa_s2 + start;
            testCase.verifyWithinTol(testCase, res_toa_s, exp_res);
            res = lbsd.getReservations();
            LBSDDeconfliction.verifyReservation(testCase, res, ...
                    res_ids,lane_id, uas2_id, exp_res(1), ...
                    exp_res(end), speed2, hd);

        end
        
        function reserveLBSDTrajectory_two_speeds_n(testCase)
            % reserveLBSDTrajectory_two_sp_eq_no_flex Test two resevations
            % when both speeds are equal and no flexibility
            lbsd = LBSDDeconfliction.singleLaneSetup();
            lane_id = "1";
            lane_length = lbsd.getLaneLengths("1");
            speed = 10; % m/s
            entry_time_s = 0;
            exit_time_s = lane_length/speed;
            hd = 10;
            ht = hd/speed;
            toa_s = [entry_time_s, exit_time_s]';
            r_e = entry_time_s;
            r_l = entry_time_s;
            uas1_id = "1";
            uas2_id = "2";
            
            % Reserve one spot
            [ok, res_ids, res_toa_s] = ...
                lbsd.reserveLBSDTrajectory(lane_id, uas1_id, toa_s, ...
                hd, r_e, r_l);
            testCase.verifyTrue(ok);
            testCase.verifyWithinTol(testCase, res_toa_s, toa_s);
            
            res = lbsd.getReservations();
            LBSDDeconfliction.verifyReservation(testCase, res, ...
                    res_ids,lane_id, uas1_id, entry_time_s, exit_time_s,...
                    speed, hd);
                
            
            % This case should fail since there is no flexibility
            speed2 = 2*speed;
            exit_time_s_2 = lane_length/speed2;
            ht2 = hd/speed2;
            toa_s2 = [entry_time_s, exit_time_s_2]';
            
            [ok, ~, ~] = ...
                lbsd.reserveLBSDTrajectory(lane_id, uas2_id, toa_s2, ...
                hd, r_e, r_l);
            testCase.verifyFalse(ok);
            
            % Adding flexibility in negative direction. 
            ht_m = max(ht, ht2);
            [ok, res_ids, res_toa_s] = ...
                lbsd.reserveLBSDTrajectory(lane_id, uas2_id, toa_s2, ...
                hd, r_e-100, r_l);
            testCase.verifyTrue(ok);
            %  Expect the reservation immediately before the other aircraft
            start = toa_s(1)-ht_m;
            exp_res = toa_s2 + start;
            testCase.verifyWithinTol(testCase, res_toa_s, exp_res);
            res = lbsd.getReservations();
            LBSDDeconfliction.verifyReservation(testCase, res, ...
                    res_ids,lane_id, uas2_id, exp_res(1), ...
                    exp_res(end), speed2, hd);

        end
        
        function reserveLBSDTrajectory_merge(testCase)
            % reserveLBSDTrajectory_two_sp_eq_no_flex Test two resevations
            % when both speeds are equal and no flexibility
            angle = 90;
            lane_length = 100;
            lbsd = LBSD.genSimpleMerge(lane_length, angle, true);
            lane_length = lbsd.getLaneLengths("1");
            
            uas1_id = "1";
            uas2_id = "2";
            
            uas_1_lane_res = ["2","1"];
            uas_2_lane_res = ["3","1"];
            
            speed = 10; % m/s
            hd = 10;
            
            entry_time_s = 0;
            isect_time_s = lane_length/speed;
            exit_time_s = isect_time_s + lane_length/speed; 
            ht = hd/speed;
            toa_s = [entry_time_s, isect_time_s, exit_time_s]';
            entry_times = toa_s(1:end-1);
            exit_times = toa_s(2:end);
            
            % Reserve one spot
            [ok, res_ids, res_toa_s] = ...
                lbsd.reserveLBSDTrajectory(uas_1_lane_res, uas1_id, toa_s, ...
                hd, toa_s(1), toa_s(3));
            testCase.verifyTrue(ok);
            testCase.verifyWithinTol(testCase, res_toa_s, toa_s);
            
            res = lbsd.getReservations();
            LBSDDeconfliction.verifyReservation(testCase, res, ...
                    res_ids, uas_1_lane_res, uas1_id, entry_times, ...
                    exit_times,...
                    speed, hd);
            res_toa_s1 = res_toa_s;

           
           % Reserve the second spot (after the first)
           [ok, res_ids, res_toa_s] = ...
                lbsd.reserveLBSDTrajectory(uas_2_lane_res, uas2_id, toa_s, ...
                hd, toa_s(1), toa_s(3));
           testCase.verifyTrue(ok);
           testCase.verifyWithinTol(testCase, res_toa_s, toa_s+ht);
            
           res = lbsd.getReservations();
           LBSDDeconfliction.verifyReservation(testCase, res, ...
                    res_ids, uas_2_lane_res, uas2_id, entry_times+ht, ...
                    exit_times+ht,...
                    speed, hd);
           res_toa_s2 = res_toa_s;
           
           
           uas1_l2_ts = [res_toa_s1(1), res_toa_s1(2)];
           uas1_l1_ts = [res_toa_s1(2), res_toa_s1(3)];  
           uas2_l3_ts = [res_toa_s2(1), res_toa_s2(2)];
           uas2_l1_ts = [res_toa_s2(2), res_toa_s2(3)];
           
           % Find time intersections
           uas12_l23 = range_intersection(uas1_l2_ts, uas2_l3_ts);
           uas12_l21 = range_intersection(uas1_l2_ts, uas2_l1_ts);
           uas12_l13 = range_intersection(uas1_l1_ts, uas2_l3_ts);
           uas12_l1 = range_intersection(uas1_l1_ts, uas2_l1_ts);
           
           x = lbsd.getVertPositions(["1","2","3","4"]);
           x1 = x(1,:);
           x2 = x(2,:);
           x3 = x(3,:);
           x4 = x(4,:);
           tf = max(res_toa_s2(3), res_toa_s1(3));
           t = linspace(0, tf, 1/(lane_length/hd/10000/speed));
           
           hd_test = hd-200*eps;
           if ~isempty(uas12_l23)
                tx = t(t >= uas12_l23(1) & t <= uas12_l23(2));
                uas1_p = x4 + tx'*(x2-x4)/speed;
                uas2_p = x3 + (tx-ht)'*(x2-x3)/speed;
                d = vecnorm(uas1_p-uas2_p,2,2);
                testCase.verifyTrue(all(d>=hd_test));
           end
           
           if ~isempty(uas12_l21)
                tx = t(t >= uas12_l21(1) & t <= uas12_l21(2));
                uas1_p = x4 + tx'*(x2-x4)/speed;
                uas2_p = x2 + (tx-ht)'*(x2-x1)/speed;
                d = vecnorm(uas1_p-uas2_p,2,2);
                testCase.verifyTrue(all(d>=hd_test));
           end
           
           if ~isempty(uas12_l13)
                tx = t(t >= uas12_l13(1) & t <= uas12_l13(2));
                uas1_p = x2 + tx'*(x2-x1)/speed;
                uas2_p = x3 + (tx-ht)'*(x2-x3)/speed;
                d = vecnorm(uas1_p-uas2_p,2,2);
                testCase.verifyTrue(all(d>=hd_test));
           end
           
           if ~isempty(uas12_l1)
                tx = t(t >= uas12_l1(1) & t <= uas12_l1(2));
                uas1_p = x2 + tx'*(x2-x1)/speed;
                uas2_p = x2 + (tx-ht)'*(x2-x1)/speed;
                d = vecnorm(uas1_p-uas2_p,2,2);
                testCase.verifyTrue(all(d>=hd_test));
           end
           
        end
        
        function reserveLBSDTrajectory_merge_vdiff(testCase)
            % TODO: Check this test
            % reserveLBSDTrajectory_two_sp_eq_no_flex Test two resevations
            % when both speeds are equal and no flexibility
            angle = 90;
            lane_length = 100;
            lbsd = LBSD.genSimpleMerge(lane_length, angle, true);
            lane_length = lbsd.getLaneLengths("1");
            
            uas1_id = "1";
            uas2_id = "2";
            
            uas_1_lane_res = ["2","1"];
            uas_2_lane_res = ["3","1"];
            
            speed1 = 10; % m/s
            speed2 = 20; % m/s
            hd = 10;
            
            entry_time_s1 = 0;
            isect_time_s1 = lane_length/speed1;
            exit_time_s1 = isect_time_s1 + lane_length/speed1; 
            ht1 = hd/speed1;
            toa_s1 = [entry_time_s1, isect_time_s1, exit_time_s1]';
            entry_times1 = toa_s1(1:end-1);
            exit_times1 = toa_s1(2:end);
            
            % Reserve one spot
            [ok, res_ids, res_toa_s] = ...
                lbsd.reserveLBSDTrajectory(uas_1_lane_res, uas1_id, toa_s1, ...
                hd, toa_s1(1), toa_s1(3));
            testCase.verifyTrue(ok);
            testCase.verifyWithinTol(testCase, res_toa_s, toa_s1);
            
            res = lbsd.getReservations();
            LBSDDeconfliction.verifyReservation(testCase, res, ...
                    res_ids, uas_1_lane_res, uas1_id, entry_times1, ...
                    exit_times1,...
                    speed1, hd);
            res_toa_s1 = res_toa_s;

            
            entry_time_s2 = 0;
            isect_time_s2 = lane_length/speed2;
            exit_time_s2 = isect_time_s2 + lane_length/speed2; 
            ht2 = hd/speed2;
            toa_s2 = [entry_time_s2, isect_time_s2, exit_time_s2]';
            entry_times2 = toa_s2(1:end-1);
            exit_times2 = toa_s2(2:end);
           
            % Reserve the second spot (after the first)
           [ok, res_ids, res_toa_s] = ...
                lbsd.reserveLBSDTrajectory(uas_2_lane_res, uas2_id, toa_s2, ...
                hd, toa_s2(1), toa_s2(3));
           testCase.verifyTrue(ok);
%            testCase.verifyWithinTol(testCase, res_toa_s, toa_s2);
            
           res = lbsd.getReservations();
%            LBSDDeconfliction.verifyReservation(testCase, res, ...
%                     res_ids, uas_2_lane_res, uas2_id, entry_times2, ...
%                     exit_times2,...
%                     speed2, hd);
           res_toa_s2 = res_toa_s;
           
           
           uas1_l2_ts = [res_toa_s1(1), res_toa_s1(2)];
           uas1_l1_ts = [res_toa_s1(2), res_toa_s1(3)];  
           uas2_l3_ts = [res_toa_s2(1), res_toa_s2(2)];
           uas2_l1_ts = [res_toa_s2(2), res_toa_s2(3)];
           
           % Find time intersections
           uas12_l23 = range_intersection(uas1_l2_ts, uas2_l3_ts);
           uas12_l21 = range_intersection(uas1_l2_ts, uas2_l1_ts);
           uas12_l13 = range_intersection(uas1_l1_ts, uas2_l3_ts);
           uas12_l1 = range_intersection(uas1_l1_ts, uas2_l1_ts);
           
           x = lbsd.getVertPositions(["1","2","3","4"]);
           x1 = x(1,:);
           x2 = x(2,:);
           x3 = x(3,:);
           x4 = x(4,:);
           tf = max(res_toa_s2(3), res_toa_s1(3));
           t = linspace(0, tf, 1/(lane_length/hd/10000/speed2));
           
           hd_test = hd-200*eps;
           u2_ts = res_toa_s(1);
           if ~isempty(uas12_l23)
                tx = t(t >= uas12_l23(1) & t <= uas12_l23(2));
                uas1_p = x4 + tx'*(x2-x4)/speed1;
                uas2_p = x3 + (tx-u2_ts)'*(x2-x3)/speed2;
                d = vecnorm(uas1_p-uas2_p,2,2);
                testCase.verifyTrue(all(d>=hd_test));
           end
           
           if ~isempty(uas12_l21)
                tx = t(t >= uas12_l21(1) & t <= uas12_l21(2));
                uas1_p = x4 + tx'*(x2-x4)/speed1;
                uas2_p = x2 + (tx-u2_ts)'*(x2-x1)/speed2;
                d = vecnorm(uas1_p-uas2_p,2,2);
                testCase.verifyTrue(all(d>=hd_test));
           end
           
           if ~isempty(uas12_l13)
                tx = t(t >= uas12_l13(1) & t <= uas12_l13(2));
                uas1_p = x2 + tx'*(x2-x1)/speed1;
                uas2_p = x3 + (tx-u2_ts)'*(x2-x3)/speed2;
                d = vecnorm(uas1_p-uas2_p,2,2);
                testCase.verifyTrue(all(d>=hd_test));
           end
           
           if ~isempty(uas12_l1)
                tx = t(t >= uas12_l1(1) & t <= uas12_l1(2));
                uas1_p = x2 + tx'*(x2-x1)/speed1;
                uas2_p = x2 + (tx-u2_ts)'*(x2-x1)/speed2;
                d = vecnorm(uas1_p-uas2_p,2,2);
                testCase.verifyTrue(all(d>=hd_test));
           end
           
        end
    end
end

