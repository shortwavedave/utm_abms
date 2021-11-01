classdef IntervalUnionTest < matlab.unittest.TestCase
    %INTERVALUNIONTEST Unit Test for Interval Union Logic
    
    methods (Static)
        function ints = BasicSetup()
            m = [0 2; 2.5, 3; 4 8;10,20;22,35];
            ints = DisjointIntervals();
            ints.setIntervals(m);
        end
        
        function verifyMatEquality(testCase, actual, expected)
            d = abs(expected - actual);
            for d_i = d(:)
                testCase.verifyLessThanOrEqual(d_i, eps);
            end
        end
    end

    methods(Test)
        function CaughtError1(testCase)
             m = [ 21.8074   41.8074;...
                   42.2495   83.9317;...
                   -3.6409   16.3591];
             ints = DisjointIntervals();
             ints.setIntervals(m);
             
             u = [3.8874   23.8874];
             i = ints.union(u);
             
             exp_res = [m(3,1) m(1,2);...
                         m(2,1) m(2,2)];
             
             IntervalUnionTest.verifyMatEquality(testCase, i, exp_res);
        end
        
        function SingleInterval(testCase)
            ints = DisjointIntervals();
            i = ints.union([0.3,8]);
            exp_res = [0.3    8];
            IntervalUnionTest.verifyMatEquality(testCase, i, exp_res);
        end
        
        function MultiInterval0(testCase)
            ints = IntervalUnionTest.BasicSetup();
            i = ints.union([3,7]);

            exp_res= [0    2.0000;
                    2.5000    8.0000;
                   10.0000   20.0000;
                   22.0000   35.0000];
            IntervalUnionTest.verifyMatEquality(testCase, i, exp_res);
        end
        
        function MultiInterval1(testCase)
            ints = IntervalUnionTest.BasicSetup();
            ints.union([3,7]);
            i = ints.union([11,12]);

            exp_res= [0    2.0000;
                    2.5000    8.0000;
                   10.0000   20.0000;
                   22.0000   35.0000];
            IntervalUnionTest.verifyMatEquality(testCase, i, exp_res);
        end


        function MultiInterval2(testCase)
            ints = IntervalUnionTest.BasicSetup();
            ints.union([3,7]);
            ints.union([11,12]);
            i = ints.union([18,21]);
            

            exp_res= [0    2.0000;
                            2.5000    8.0000;
                           10.0000   21.0000;
                           22.0000   35.0000];
            IntervalUnionTest.verifyMatEquality(testCase, i, exp_res);
        end
        
        function MultiInterval3(testCase)
            ints = IntervalUnionTest.BasicSetup();
            ints.union([3,7]);
            ints.union([11,12]);
            ints.union([18,21]);
            i = ints.union([0,2.2]);

            exp_res = [0    2.2;
                            2.5000    8.0000;
                           10.0000   21.0000;
                           22.0000   35.0000];
            IntervalUnionTest.verifyMatEquality(testCase, i, exp_res);          
        end
        
        function MultiInterval4(testCase)
            ints = IntervalUnionTest.BasicSetup();
            ints.union([3,7]);
            ints.union([11,12]);
            ints.union([18,21]);
            ints.union([0,2.2]);
            i = ints.union([34,36]);


            exp_res = [0    2.2;
                            2.5000    8.0000;
                           10.0000   21.0000;
                           22.0000   36.0000];
            IntervalUnionTest.verifyMatEquality(testCase, i, exp_res);           
        end

        function MultiInterval5(testCase)
            ints = IntervalUnionTest.BasicSetup();
            ints.union([3,7]);
            ints.union([11,12]);
            ints.union([18,21]);
            ints.union([0,2.2]);
            ints.union([34,36]);
            i = ints.union([0,37]);

            exp_res= [0    37];
            IntervalUnionTest.verifyMatEquality(testCase, i, exp_res);
        end
        
        function SingleInterval1(testCase)
            ints = DisjointIntervals();
            % ints.setIntervals(m);
            i = ints.union([0.3,8]);

            exp_res = [0.3    8];
            IntervalUnionTest.verifyMatEquality(testCase, i, exp_res);
        end

        function TestPreallocate(testCase)
            ints = DisjointIntervals(5);
            m = [0 2; 2.5, 3; 4 8;10,20;22,35];
            ints.setIntervals(m);
            i = ints.union([2.7,11]);
            exp_res = [0 2; 2.5,20; 22,35];
            IntervalUnionTest.verifyMatEquality(testCase, i, exp_res);
        end
        
        function TestPreallocate2(testCase)
            ints = DisjointIntervals(5);
            m = [0 2; 2.5, 3; 4 8;10,20;22,35];
            ints.setIntervals(m);
            i = ints.union([11,36]);
            exp_res = [0 2; 2.5,3; 4,8;10,36];
            IntervalUnionTest.verifyMatEquality(testCase, i, exp_res);
        end
        
        function TestPreallocate3(testCase)
            ints = DisjointIntervals(5);
            m = [0 2; 2.5, 3; 4 8;10,20;22,35];
            ints.setIntervals(m);
            i = ints.union([36,37]);
            exp_res = [0 2; 2.5, 3; 4 8;10,20;22,35;36,37];
            IntervalUnionTest.verifyMatEquality(testCase, i, exp_res);
        end
        
        function TestPreallocate4(testCase)
            ints = DisjointIntervals(3);
            m = [0 2; 2.5, 3; 4 8;10,20;22,35];
            ints.setIntervals(m);
            exp_res = [0 2; 2.5, 3; 4 8;10,20;22,35];
            IntervalUnionTest.verifyMatEquality(testCase, ints.intervals, exp_res);
        end
        
        function TestPreallocate5(testCase)
            ints = DisjointIntervals(5);
            m = [0 2; 2.5, 3; 4 8;10,20;22,35];
            ints.setIntervals(m);
            i = ints.union([-2,-1]);
            exp_res = [-2,-1;0 2; 2.5, 3; 4 8;10,20;22,35];
            IntervalUnionTest.verifyMatEquality(testCase, i, exp_res);
        end
        
        function TestPreallocate6(testCase)
            ints = DisjointIntervals(5);
            m = [0 2; 2.5, 3; 4 8;10,20;22,35];
            ints.setIntervals(m);
            i = ints.union([-2,1]);
            exp_res = [-2, 2; 2.5, 3; 4 8;10,20;22,35];
            IntervalUnionTest.verifyMatEquality(testCase, i, exp_res);
        end
    end
end

