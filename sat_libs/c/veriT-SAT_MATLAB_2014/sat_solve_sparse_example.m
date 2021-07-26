clearvars;

% Save clauses into a dense matrix
disp('Feeding dense matrix with clauses.');
M=[0 -1 1; 0 1 1; -1 1 1; -1 -1 1; 1 0 -1; -1 1 -1; 1 0 -1];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%             SPARSE MATRIX               %%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SAT solver initialization
sat_init();
% Generate sparse matrix from dense matrix
disp('Generating sparsed matrix from dense matrix.');
S = sparse(M);
% Call SAT solver with sparse matrix
% res stores satisfiabily result
% model stores model if it exists
disp('SAT solving using sparsed matrix.');
[res, model]=sat_solve(S);
% SAT solver releasing
sat_done();
disp('Result: ');
disp(res);
disp('Model: ');
disp(model);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%             DENSE MATRIX              %%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Call SAT solver with dense matrix
% res stores satisfiabiliy result
% model stores model if it exists
% SAT solver initialization
sat_init();
% disp('SAT solving using dense matrix.');
[res, model]=sat_solve(M);
% SAT solver releasing
sat_done();
disp('Result: ');
disp(res);
disp('Model: ');
disp(model);


disp('SAT solving has just finished.');
