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

        %asks for a clause that exists in the KB.
        function testAskClause(testCase)
            kb = KB();
            c = {'~assigned'};
            kb.tellClause(c);
            testCase.verifyEqual(kb.askClause(c), true);
        end

        %asks for an empty clause that exists in the KB.
        function testAskEmptyClause(testCase)
            kb = KB();
            c = {''};
            kb.tellClause(c);
            testCase.verifyEqual(kb.askClause(c), true);
        end
        
        % asks for a clause that does not exists in KB.
        function testAskClauseFalse(testCase)
            kb = KB();
            c = {'~assigned'};
            cl = {'assigned'};
            kb.tellClause(c);
            testCase.verifyEqual(kb.askClause(cl), false);
        end

        % checks if the KB has knowledge about existing clause.
        function testHasKnowledge(testCase)
            kb = KB();
            c = {'~assigned'};
            kb.tellClause(c);
            raw_model = 1;
            [~, ~, has_knowledge] = askClause(kb, ...
                c, raw_model);
            testCase.verifyEqual(has_knowledge, true);
        end

        % checks if the KB has knowledge about an empty existing clause.
        function testHasKnowledgeEmptyClause(testCase)
            kb = KB();
            c = {''};
            kb.tellClause(c);
            raw_model = 1;
            [~, ~, has_knowledge] = askClause(kb, ...
                c, raw_model);
            testCase.verifyEqual(has_knowledge, true);
        end

       % asks for a clause that does not exits in KB but is partially same
       % with the existing clause.
        function testAskClausePartial(testCase)
            kb = KB();
            c = {'~assigned', 'test', 'test2', 'test3'};
            c2 = {'~assigned', 'test', 'test2'};
            kb.tellClause(c);
            testCase.verifyEqual(kb.askClause(c2), false);
        end

         % checks if the KB has knowledge about clause that does not exist.
        function testHasKnowledgeFalse(testCase)
            kb = KB();
            c = {'~assigned'};
            ctest = {'test'};
            kb.tellClause(c);
            raw_model = 1;
            [~, ~, has_knowledge] = askClause(kb, ...
                ctest, raw_model);
            testCase.verifyEqual(has_knowledge, false);
        end
    
         % since the KB does not have any knowledge about 'test4' and 'test5'
         % ,that are in c2, has_knowledge is expected to be false.
        function testHasKnowledgePartialLarge(testCase)
            kb = KB();
            c = {'~assigned', 'test', 'test2', 'test3'};
            c2 = {'~assigned', 'test', 'test2', 'test4', 'test5'};
            kb.tellClause(c);
            raw_model = 1;
            [~, ~, has_knowledge] = askClause(kb, ...
                c2, raw_model);
            testCase.verifyEqual(has_knowledge, false);
        end
        
        % since the KB has knowledge for all the atoms in c2, has_knowledge
        % is expected to be true.
        function testHasKnowledgePartial(testCase)
            kb = KB();
            c = {'~assigned', 'test', 'test2', 'test3'};
            c2 = {'~assigned', 'test', 'test2'};
            kb.tellClause(c);
            raw_model = 1;
            [~, ~, has_knowledge] = askClause(kb, ...
                c2, raw_model);
            testCase.verifyEqual(has_knowledge, true);
        end

        function testEntailsEmpty(testCase)
            kb = KB();
            c = {''};
            kb.tellClause(c);
            raw_model = 1;
            [entails, ~, ~] = askClause(kb, ...
                c, raw_model);
            testCase.verifyEqual(entails, true);
        end
        
        % test if the tellClause adds an existing clause when it is forced.
        function testTellClauseWithForce(testCase)
            kb = KB();
            c = {'~assigned'};
            c2 = {'~assigned'};
            kb.tellClause(c);
            kb.tellClause(c2, true);
           
            testCase.verifyEqual(length(kb.m_logic_mat), 2);
        end

         % test if the tellClause adds an existing clause when it is not forced.
        function testTellClauseWithNoForce(testCase)
            kb = KB();
            c = {'~assigned'};
            c2 = {'~assigned'};
            kb.tellClause(c);
            kb.tellClause(c2);
          
            testCase.verifyEqual(length(kb.m_logic_mat), 1);
        end
    end
end

