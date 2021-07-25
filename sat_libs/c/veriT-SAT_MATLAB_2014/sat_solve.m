function [s,v]=sat_solve(M)
if (issparse(M))
  [s,v]=sat(1,M');
else
  [s,v]=sat(1,M);
end

