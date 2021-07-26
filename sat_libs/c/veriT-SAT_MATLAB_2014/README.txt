To use the SAT solver within MATLAB, use mex to compile the solver and
the interface:
  mex -g -largeArrayDims sat.c veriT-SAT.c veriT-qsort.c

or activating the mex options using the option -f PATH_TO_MATHLAB/bin/mexopts.sh.

Using the SAT solver requires initialization:
  sat_init;

After initialisation, it is possible to submit a problem to the SAT
solver.  The problem is a set of clauses in matrix form, where each
row represents a clause.  Matrix columns are associated to Boolean
variables.  A variable may not appear in a clause, appear positively,
or appear negatively; the value of the number in the row of the clause
and in the column of the variable is then respectively 0, 1, or -1.
As an example consider M, defined as
  M=[0 -1 1; 1 1 1; -1 1 1; -1 -1 1; 1 1 -1; -1 1 -1; 1 -1 -1];
If Boolean variables are named v1, v2, and v3, the first clause
(represented by the first row) is NOT v2 or v3.  v1 does not appear in
the clause, hence the 0 at row 1 column 1.

To submit a problem in the above form to the SAT solver, just type
  [s,v]=sat_solve(M)
s then states if the problem is satisfiable or not, and v gives a
model if the problem is satisfiable.  The model is a vector containing
values 1 and -1.  A value 1 (-1) at position i means variable i is
positive (resp. negative) in the model found by the SAT solver.

The SAT solver should then be released using
  sat_done;

The above scenario also works with sparse matrices.

More features will be available in the future.


sat_solve_sparse_example.m: a very simple use case showing how to use
the SAT solver from Matlab.

sat_init.m: SAT solver initialization

sat_solve.m: call the SAT solver, adding the set of clauses represented as a matrix (sparse or dense representation)

sat_done.m: SAT solver releasing

Remarks:

- the SAT solver should be initialized (sat_init) before every call to sat_solve, and released (sat_done) after every call.

- all interface functions are in file sat.c.

