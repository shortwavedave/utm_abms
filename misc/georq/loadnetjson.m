function [obj] = loadnetjson(filename)
%LOADNETJSON Summary of this function goes here
%   Detailed explanation goes here
    fid = fopen(filename);
    raw = fread(fid,inf);
    str = char(raw');
    fclose(fid);
    obj = jsondecode(str);
end

