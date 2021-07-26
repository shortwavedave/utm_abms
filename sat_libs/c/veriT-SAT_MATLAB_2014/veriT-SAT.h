/**
   veriT SAT solver.

   Each propositional variable is identified with a positive integer.
   Each clause is identified with a positive integer.
 */
#ifndef SAT_H
#define SAT_H

#define SAT_SYM

#include <stdbool.h>

typedef unsigned SAT_Tvar;    /**< var index into stack_var */
typedef unsigned SAT_Tlit;    /**< lit is var<<1 or var<<+1 according to polarity */ 
typedef unsigned SAT_Tclause; /**< clause index into stack_clause */ 
typedef unsigned SAT_Tlevel;  /**< level type */ 

typedef enum {
  SAT_STATUS_UNSAT = 0,
  SAT_STATUS_SAT = 1,
  SAT_STATUS_UNDEF = 2
} SAT_Tstatus;

#define SAT_VAL_FALSE 0
#define SAT_VAL_TRUE 1
#define SAT_VAL_UNDEF 2

#define SAT_VAR_UNDEF 0
#define SAT_LIT_UNDEF 0
#define SAT_CLAUSE_UNDEF 0

typedef unsigned char SAT_Tvalue;

/**
   \brief array of literals assigned by the SAT solver
   \remark it is the full model if status is SAT
   \remark it is not relevant if status is UNSAT */
extern SAT_Tlit *SAT_literal_stack;
/**
   \brief number of literals assigned by the SAT solver
   \remark it should be the number of literals if status is SAT
   \remark it is not relevant if status is UNSAT */
extern unsigned  SAT_literal_stack_n;
/**
   \brief number of literals kept unmodified in the stack
   \remark User should set it to SAT_literal_stack_n to reset it */
extern unsigned  SAT_literal_stack_hold;
/**
   \brief number of unit literals in the stack
   \remark these literals will be true in all subsequent partial models */
extern unsigned  SAT_literal_stack_unit;
/**
   \brief pointer to first literal to propagate
   \remark do not modify.  Just to keep track of hints, if call to
   SAT_propagate() is required */
extern unsigned  SAT_literal_stack_to_propagate;

/**
   \brief get the decision level of the sat solver (basically, the number of
   decisions) */
extern SAT_Tlevel SAT_level;
/**
   \brief array of levels
   \remark it is not relevant if status is UNSAT */
extern SAT_Tlevel *SAT_level_stack;
/**
   \brief number of levels kept unmodified in the stack
   \remark User should set it to SAT_level to reset it */
extern unsigned  SAT_level_stack_hold;

/**
   \brief status of the sat solver */
extern SAT_Tstatus SAT_status;


/**
   \brief creates a new propositional variable
   \return the identifier of the variable (an unsigned int)
   \remark ensures that internal data structures are resized if necessary to
   accomodate the variable 
*/
SAT_Tvar    SAT_var_new(void);

void        SAT_var_new_id(unsigned id);
SAT_Tvalue  SAT_var_value(SAT_Tvar var);
SAT_Tlevel  SAT_var_level(SAT_Tvar var);
void        SAT_var_block_decide(SAT_Tvar var);
void        SAT_var_unblock_decide(SAT_Tvar var);

#ifdef SAT_SYM
/**
   \author Pascal Fontaine
   \brief in case symmetry is used, once a unit clause about a variable
   is deduced, unit clauses about all variables in the orbit are automatically
   added.  This array stores the orbit. */
extern SAT_Tvar * SAT_var_orbit;
#endif

void        SAT_phase_cache_set(void);

/**
   \author Pascal Fontaine
   \brief build a literal from variable and polarity
   \param var the variable
   \param pol the polarity */
#define SAT_lit(var, pol) ((SAT_Tlit)((var << 1) + pol))
/**
   \author Pascal Fontaine
   \brief get variable of literal
   \param lit the literal */
#define SAT_lit_var(lit) ((SAT_Tvar)(lit >> 1))
/**
   \author Pascal Fontaine
   \brief build negation of literal
   \param lit the literal */
#define SAT_lit_neg(lit) (lit ^ 1)
/**
   \author Pascal Fontaine
   \brief get polarity of literal
   \param lit the literal */
#define SAT_lit_pol(lit) ((SAT_Tvalue) (lit & 1))

/**
   \author Pascal Fontaine
   \brief get the value associated with the literal
   \param lit the literal
   \return the value (VAL_FALSE, VAL_TRUE, or VAL_UNDEF) */
SAT_Tvalue  SAT_lit_value(SAT_Tlit lit);
/**
   \author Pascal Fontaine
   \brief get the literal level
   \param lit the literal
   \return the level at which literal has been assigned */
SAT_Tlevel  SAT_lit_level(SAT_Tlit lit);

/**
   \author Pascal Fontaine
   \brief add all explanations for literals at root level
   \remark it is necessary to call this function before adding
   conflict clauses.  However, this can not be done inside SAT_clause_new,
   since it also introduces new clauses, and it cannot be done in SAT_hint,
   since hints are sometimes sent (notably by CC) when reasons are not ready
   \todo there may be a better solution */
#ifdef PROOF
void        SAT_sanitize_root_level(void);
#endif

/**
   \author Pascal Fontaine
   \brief adds clause in SAT
   \param n the number of literals
   \param lits an array of n literals
   \remark destructive for the array of literals
   \remark returns CLAUSE_UNDEF if valid clause or problem already found unsat
   \return clause id or CLAUSE_UNDEF */
SAT_Tclause SAT_clause_new(unsigned n, SAT_Tlit * lits);
/**
   \author Pascal Fontaine
   \brief adds clause in SAT
   \param n the number of literals
   \param lits an array of n literals
   \remark destructive for the array of literals
   \remark returns CLAUSE_UNDEF if valid clause or problem already found unsat
   \return clause id or CLAUSE_UNDEF 
   \remark just as above, but clause may be purged later */
SAT_Tclause SAT_clause_new_conflict(unsigned n, SAT_Tlit * lits);
/**
   \author Pascal Fontaine
   \brief adds a clause as late explanation for a literal
   \param n the number of literals
   \param lit an array of n literals
   \remark this should be called on the SAT solver request for explanation of
   a propagated literal with lazy clause
   \remark destructive for the array of literals
   \pre one of the literals is true
   \pre all other literals are false
   \pre level of true literal and at least one other is maximum level of all
   literals in clause
   \return clause id or CLAUSE_UNDEF */
SAT_Tclause SAT_clause_new_lazy(unsigned n, SAT_Tlit * lit);

void        SAT_push(void);
void        SAT_pop(void);

/**
   \author Pascal Fontaine
   \brief propagates until a decision has to be done
   \return SAT_STATUS_SAT, SAT_STATUS_UNSAT, or SAT_STATUS_UNDEF */
SAT_Tstatus SAT_propagate(void);
/**
   \author Pascal Fontaine
   \brief adds hint, i.e. propagated literal with lazy clause
   \remark may be applied repeatedly
   \remark lit is either true (discarded) or undefined but never false */
void        SAT_hint(SAT_Tlit lit);
/**
   \author Pascal Fontaine
   \brief performs a decision
   \pre SAT_propagate applied just before (no SAT_hint meantime)
   \return false iff there is nothing to decide (therefore SAT) */
bool        SAT_decide(void);
/**
   \author Pascal Fontaine
   \brief restart SAT solver */
void        SAT_restart(void);

/**
   \author Pascal Fontaine
   \brief runs until a model is found or unsat
   \return SAT_STATUS_SAT, SAT_STATUS_UNSAT */
SAT_Tstatus SAT_solve(void);

/* It is counterproductive to work only on decided literals, since
   propagated literals may also be removed */
#define SAT_MIN_USE_TAUTOLOGIES 1
#define SAT_MIN_SKIP_PROPAGATED 2
void        SAT_minimal_model(SAT_Tlit ** PPlit, unsigned *n, unsigned options);

void        SAT_init(void);
void        SAT_done(void);
void        SAT_reset(void);

#ifdef PROOF
extern unsigned SAT_proof;
#if defined(INSIDE_VERIT)
extern unsigned SAT_proof_stack_n;
extern SAT_Tlit *SAT_proof_stack_lit;
extern SAT_Tclause *SAT_proof_stack_clause;
#endif
#endif

#endif
