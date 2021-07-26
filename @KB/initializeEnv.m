function initializeEnv(init_c_sat, init_py_sat, compile)
%INITIALIZEENV Summary of this function goes here
%   Detailed explanation goes here
    if nargin < 3
        compile = true;
    end
    currentFolder = pwd;
    c_sat_dir = './sat_libs/c/veriT-SAT_MATLAB_2014';
    py_sat_dir = './sat_libs/python/simple-sat/src';
    if ispc
       c_sat_dir = '.\sat_libs\c\veriT-SAT_MATLAB_2014';
        py_sat_dir = '.\sat_libs\python\simple-sat\src'; 
    end
    if init_c_sat
        if ~test_csat()
            if ~exist(c_sat_dir, 'dir')
                error("C-SAT directory is missing, try initializing from the top level directory");
            else
                disp 'Adding c-sat folders to path'
                addpath(c_sat_dir);
                if compile
                    cd(c_sat_dir)
                    disp 'Compiling c-sat library'
                    mex -g -largeArrayDims sat.c veriT-SAT.c veriT-qsort.c
                    cd(currentFolder);
                end
                disp 'Testing c-sat library'
                if test_csat()
                    disp 'Done.'
                else
                    error('c-sat library did not initialize correctly');
                end
            end
        end
    end
    
    if init_py_sat
        if ~test_pysat()
            if count(py.sys.path, py_sat_dir) == 0 
               disp 'Adding py-sat folders to python path'
               insert(py.sys.path, int32(0), py_sat_dir);
            end
            disp 'Testing py-sat library'
            if test_pysat()
               disp('Done');
            else
               disp 'py-sat library did not initialize correctly'
            end
        end
    end
end

function ok = test_csat()
    try
        M = [0 -1 1; 1 1 1; -1 1 1; -1 -1 1; 1 1 -1; -1 1 -1; 1 -1 -1];
        sat_init;
        [s,v]=sat_solve(M );
        sat_done;
        if s && ~isempty(v)
            ok = true;
        else
            ok = false;
        end
    catch
        ok = false;
    end
end

function ok = test_pysat()
    try
        inst = py.satinstance.SATInstance();
        if ~isempty(inst)
           ok = true;
        else
           ok = false;
        end
    catch
        ok = false;
    end
end