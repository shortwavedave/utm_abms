classdef KBTests < matlab.unittest.TestCase
    %KBTests Summary of this class goes here
    %   Detailed explanation goes here
    
    methods (Static)
    end
    
    methods(Test)
        function testEntails(testCase)
            kb = KB();
            c = {'~assigned'};
            kb.tellClause(c);
            raw_model = 1;
            [entails, ~, ~] = askClause(kb, ...
                c, raw_model);
            testCase.verifyEqual(entails, true);
        end
        
        function testNotEntails(testCase)
            kb = KB();
            c = {'~assigned'};
            ctest = {'assigned'};
            kb.tellClause(c);
            raw_model = 1;
            [entails, ~, ~] = askClause(kb, ...
                ctest, raw_model);
            testCase.verifyEqual(entails, false);
        end
    end
end

