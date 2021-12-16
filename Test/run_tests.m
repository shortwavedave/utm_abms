function run_tests(rel_file)
%RUN_TESTS Summary of this function goes here
%   Detailed explanation goes here
import matlab.unittest.TestSuite
import matlab.unittest.TestRunner
import matlab.unittest.plugins.CodeCoveragePlugin
import matlab.unittest.plugins.codecoverage.CoverageReport
p = replace(mfilename('fullpath'),"run_tests","");
% suiteClass = TestSuite.fromClass(?IntervalUnionTest);

if nargin < 1
    test_files = dir(p);
    suiteClass = [];
    for i = 1:length(test_files)
        if test_files(i).isdir
            d = test_files(i);
            s = TestSuite.fromFolder(fullfile(d.folder,d.name));
            suiteClass = [suiteClass s];
        end
    end
else
    suiteClass = TestSuite.fromFile(fullfile(p,rel_file));
end

% With coverage
% runner = TestRunner.withNoPlugins;
% runner.addPlugin(CodeCoveragePlugin.forFile('DisjointIntervals.m', ...
% 'Producing',CoverageReport('interval_res', ...
% 'MainFile','interval_res.html')))
% runner.run(suiteClass)


res = suiteClass.run;
end

