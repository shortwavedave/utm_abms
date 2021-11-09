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
                r = res_table(res_table.id==res_id,:);
                testCase.verifyEqual(r.lane_id, lane_id);
                testCase.verifyEqual(r.uas_id, uas_id);
                testCase.verifyWithinTol(testCase, r.entry_time_s, ...
                    entry_time_s);
                testCase.verifyWithinTol(testCase, r.exit_time_s, ...
                    exit_time_s);
                testCase.verifyWithinTol(testCase, r.speed, speed);
                testCase.verifyWithinTol(testCase, r.hd, hd);
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
            testCase.verifyFalse(ok);
            %  Expect the reservation immediately after the other aircraft
            exp_res = toa_s + ht;
            testCase.verifyWithinTol(testCase, res_toa_s, exp_res);
            res = lbsd.getReservations();
            LBSDDeconfliction.verifyReservation(testCase, res, ...
                    res_ids,lane_id, uas2_id, entry_time_s+ht, exit_time_s+ht,...
                    speed, hd);
            
        end
    end
end

