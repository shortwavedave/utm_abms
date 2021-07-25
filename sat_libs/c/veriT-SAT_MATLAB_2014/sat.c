
#include "mex.h" 
#include "veriT-SAT.h"
#include <stdio.h>

/* interface MATLAB pour COLLOCATION :

    [s,var] = sat_solver(M) 

    INPUT : 
    - M  : clauses dense matrix  (NbClausesxNbVar)
 
    OUTPUT : 
     - s : satisfaisability boolean 
     - var : variables vector containing a solution (NbVar) 
  */

void mexFunction(int nlhs, mxArray *plhs[], int nrhs,const mxArray *prhs[])
{
   
  mwSize NbClauses,
         NbVar,
	 un=1;

  mwIndex i,
          j,
          c,
	  k,
	  accu;
  double *M,
          val;
  
  SAT_Tlit * clause;
  SAT_Tstatus status;

  if (nrhs < 1)
    mexErrMsgIdAndTxt("sat","\n\r no function to call\n");

  M = mxGetPr(prhs[0]);

  if (M[0] == 0)
    {
      SAT_init();
      return;
    }
  else if (M[0] ==2)
    {
      SAT_done();
      return;
    }

  if (nrhs != 2 ) {
      mexErrMsgIdAndTxt("sat_solver:rhs","\n\r wrong number of arguments : one expected");
  }
  
  if ((nlhs != 1 ) && (nlhs != 2 )) {
      mexErrMsgIdAndTxt("sat_solver:lhs","\n\r wrong number of outputs : one or two expected");
  }
  
  /* The first argument must be a 3xNbP matrix.
     If sparse matrix the function expect a transposed matrix.
  */
  if (mxIsSparse(prhs[1])) {    
    NbVar = mxGetM(prhs[1]);
    NbClauses = mxGetN(prhs[1]);
  } else {
    NbClauses =  mxGetM(prhs[1]);
    NbVar = mxGetN(prhs[1]);
  }

  M = mxGetPr(prhs[1]);
  
  /* first return value : boolean indicating satisfaisability  */
  plhs[0] = mxCreateDoubleMatrix(un, un, mxREAL);
  if (nlhs == 2) {
     plhs[1] = mxCreateDoubleMatrix(NbVar, un, mxREAL);
  }
  
  /* adding variables */
  for(i = 0; i < NbVar; ++i) {
    SAT_var_new_id(i+1);
  }

  if (!mxIsSparse(prhs[1])) { /* Not sparsed matrix*/
    /* adding clauses */
    for (i= 0; i < NbClauses ; ++i) {
      /* counting the number of variables present
       in the clause i */
      accu = 0;
      for (j = 0; j < NbVar; j++) { 
        accu += (M[i+j*NbClauses] ? 1 : 0);  
      }
      clause = (SAT_Tlit*) malloc(accu * sizeof(SAT_Tlit));
      k = 0;
      for (j = 0; j < NbVar; j++) { 
        val = M[i+j*NbClauses];
	if ( val ) {
	   clause[k] = SAT_lit(j+1,  (val == 1 ? 1 : 0));    
           k++;
        }
      }
      SAT_clause_new(accu, clause); 
    }
  } else { /* Sparse matrix*/
    /* Get sparse matrix representation */
    mwIndex *jc = mxGetJc(prhs[1]);
    mwIndex *ir = mxGetIr(prhs[1]);
    /* adding clauses */
    for (c = 0, k = 0; k <= mxGetNzmax(prhs[1]); ) { 
      if ( k == jc[c+1])
        { /* the end of clause c has been reached */
          /* count the number of variables present in the clause c */
          accu = jc[c+1] - jc[c];
          clause = (SAT_Tlit*) malloc(accu * sizeof(SAT_Tlit));
          for (i = 0, j = jc[c]; j < jc[c+1]; i++, j++)
            clause[i] = SAT_lit(ir[i+jc[c]]+1,  (M[i+jc[c]] == 1 ? 1 : 0));    
        /* define the clause and reset the number of variables in the next clause */
        SAT_clause_new(accu, clause);
        c++;
      }
    else
      k++;
    }
  }
  
  status = SAT_solve();
  
  if (status == SAT_STATUS_SAT)
    {
      //mexPrintf("satis\n");
      *mxGetPr(plhs[0]) = 1;
      if (nlhs == 2) {
         for (i = 0; i <NbVar; i++)
	   *(mxGetPr(plhs[1]) + i) = (SAT_var_value(i+1)?1:-1);
      }
    }
  else if (status == SAT_STATUS_UNSAT) {
      //mexPrintf("unsatis\n");
      *mxGetPr(plhs[0]) = 0;
  }
  else {
      mexErrMsgIdAndTxt("sat_solver","\n\r strange behaviour");
  }
  
}

