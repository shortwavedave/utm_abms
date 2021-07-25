/* PARAM
   define BCLAUSE_GENERATION
   define BCLAUSE xor BCLAUSE_LIGHT xor NONE
   define SIMP
   define RESTART_MIN_INTERVAL 7

   define LEARNTS_ADJ_FACT 1.5
   define LEARNTS_MAX_FACT 1.1
   define LEARNTS_ADJ_INIT 100
   define LEARNTS_FACT_INIT 0.33
   define BLACK_MAGIC
*/

/*
#define DEBUG_SAT 
#define DEBUG_LEVEL 2
*/

/*
#define PROOF
#define PROOF_PRINT
#define PROOF_PRINT_CLAUSES */

 /* #define BCLAUSE_GENERATION */

/* #define BLACK_MAGIC */
/* #define BCLAUSE */
/* #define BCLAUSE_LIGHT */

/** define BACKTRACK if need to remove clauses */
/* #define BACKTRACK */

#ifdef DEBUG_SAT
#define ON_DEBUG_SAT(X) X
#else
#define ON_DEBUG_SAT(X)
#endif

#define REUSE_TRAIL

#if defined(BCLAUSE) || defined(BCLAUSE_LIGHT)
#ifdef BACKTRACK
#error binary clauses are not compatible with backtracking
#endif
#endif

#define SIMP
#define CLAUSE_MIN

#include <assert.h>
#include <limits.h>
#include <stdbool.h>
#ifdef DEBUG_SAT
#include <stdio.h>
#endif
#include <stdlib.h>
#include <string.h>

#include "veriT-qsort.h"

#include "veriT-SAT.h"

#ifdef INSIDE_VERIT
#include "config.h"
#include "general.h"
#define HINTS
/* #define HINT_AS_DECISION */ /* MAGIC PARAM */
#endif

#ifndef STATS_LEVEL
#define STATS_LEVEL 0
#endif
/* Level 1 store basic stats */
/* Level 4 printf quite a lot of internal information */

#if STATS_LEVEL >= 1
#include "general.h"
#include "statistics.h"
#endif

/**
   \brief Tunable constants
   \remark the actual numbers are 1<<X */

/**
   \brief coefficient of the luby suite for number of conflicts between restarts */
/* tried 6 7 8 9 (7 is best) */
#define RESTART_MIN_INTERVAL 7

#define LEARNTS_ADJ_FACT 1.5
#define LEARNTS_MAX_FACT 1.1
#define LEARNTS_ADJ_INIT 100
#define LEARNTS_FACT_INIT 0.33

/*
  PF
  use unsigned index rather than pointers for everything that is used often
  use separate datastructures for anything used rarely

  SEE IMPROVE

  Question: how to efficiently update watch lists?

  My idea was to be able to remove any clause, so that instances could
  later be removed if proved unuseful.  However, even so, it leaves
  the literals (except if they are also removed).  And the model will
  contain those and polute the theory reasoner.

  IMPROVE
  - code for binary clauses
  - preprocessing and periodic simplification
*/

/**
   \remark Limits:
   \li clause ids should not exceed 30 bits
   \li number of literals in clauses should not exceed 27 bits */

/**
   \remark This solver is fully backtrackable, i.e. clauses may be
   added at any point and retreived lateron.  SAT_push adds a
   backtrackable point.  In order to maintain the clause stack
   chronological, all clauses in the older levels are kept undeleted.
   PRESERVE_CLAUSES disable all clause deletions (clauses are
   "inactivated", but maintained in memory) */

/*
  TODO
  Backtracking: review all code
  I propose to remove all learnt clause if SAT_pop
  It will be also necessary to remember unit clauses and restore the
  stack_lit so that units are taken into account
  Input clauses should not be simplified/removed from watch lists if BACKTRACKABLE
  history should be rethought */

/*
  --------------------------------------------------------------
  Internal simplification 
  --------------------------------------------------------------
*/

#define Tvar SAT_Tvar
#define Tlit SAT_Tlit
#define Tclause SAT_Tclause
#define Tlevel SAT_Tlevel
#define Tvalue SAT_Tvalue

#define Tstatus SAT_Tstatus

#define VAL_FALSE SAT_VAL_FALSE
#define VAL_TRUE SAT_VAL_TRUE
#define VAL_UNDEF SAT_VAL_UNDEF

#define VAR_UNDEF SAT_VAR_UNDEF
#define LIT_UNDEF SAT_LIT_UNDEF
#define CLAUSE_UNDEF SAT_CLAUSE_UNDEF
#define CLAUSE_LAZY UINT_MAX

/*
  --------------------------------------------------------------
  Miscaleneous early declarations
  --------------------------------------------------------------
*/

#define ROOT_LEVEL 0

Tstatus SAT_status = SAT_STATUS_UNDEF;     /**< status of the sat solver */
Tlevel SAT_level = ROOT_LEVEL;           /**< decision level */
Tclause SAT_empty_clause = CLAUSE_UNDEF; /**< clause id of empty clause */
static unsigned conflict_nb = 0;

#ifdef PROOF
unsigned SAT_proof = 0;
#endif

#if STATS_LEVEL >= 1
unsigned stat_n_conflict = 0;
unsigned stat_n_conflict_lit = 0;
unsigned stat_n_decision = 0;
unsigned stat_n_tp = 0;
unsigned stat_n_delete = 0;
unsigned stat_n_restart = 0;
unsigned stat_n_purge = 0;
unsigned stat_n_clauses = 0;
unsigned stat_n_prop = 0;
#if STATS_LEVEL >= 2
unsigned stat_n_watched = 0;
unsigned stat_prop_lit_call_nowatch = 0;
unsigned stat_prop_call = 0;
unsigned stat_prop_call_waste = 0;
unsigned stat_prop_call_noprop = 0;
#endif
#endif

static unsigned misc_stack_size = 0;
static unsigned misc_stack_n = 0;
static Tlit *   misc_stack = NULL;

/*
  --------------------------------------------------------------
  Miscaleneous early declarations
  --------------------------------------------------------------
*/

#ifdef DEBUG_SAT
static void check_consistency(void);
static void check_consistency_final(void);
static void check_consistency_propagation(void);
static void check_consistency_heap(void);
static void print_stack(void);
#endif
static inline void var_order_insert(Tvar var);

#ifdef PROOF
static void proof_begin(Tclause clause);
static void proof_resolve(Tlit lit, Tclause clause);
static void proof_end(Tclause clause);
#ifdef PROOF_PRINT
static void proof_print(Tclause clause);
#endif
#endif

#ifdef HINTS
extern void (*hint_explain)(Tlit lit);
#endif
/*
  --------------------------------------------------------------
  Utilities
  --------------------------------------------------------------
*/

#ifndef INSIDE_VERIT
#if STATS_LEVEL == 0

#include <stdarg.h>
#include <stdio.h>

#define MY_MALLOC(v,s) \
  v = malloc(s); \
  if (s && !v) \
    my_error("malloc error on line %d in file " __FILE__ "\n", __LINE__)
#define MY_REALLOC(v,s) \
  v = realloc(v,s); \
  if (s && !v) \
    my_error("realloc error on line %d in file " __FILE__ "\n", __LINE__)

#ifdef DEBUG_SAT
#define MY_BREAK_N(n) \
{static int i = 0; \
 if (++i == n) breakpoint(); \
 fprintf(stderr,__FILE__ ", %d : %d pass\n", __LINE__, i); }
#endif

/*--------------------------------------------------------------*/

static void
my_error(char *format, ...)
{
  va_list params;
  va_start(params, format);
  fprintf(stderr, "error : ");
  vfprintf(stderr, format, params);
  va_end(params);
  exit(1);
}

/*--------------------------------------------------------------*/

#ifdef DEBUG_SAT
static void
breakpoint(void)
{
  fprintf(stderr, "breakpoint\n");
}
#endif

/*--------------------------------------------------------------*/

typedef int (* TFcmp) (const void *, const void *);

#endif
#endif

/*
  --------------------------------------------------------------
  Randomization functions
  --------------------------------------------------------------
*/

/* #define RANDOMIZE_DECISION */
#ifdef RANDOMIZE_DECISION

#define RANDOMIZE_SEED 123456
#define RANDOMIZE_FREQ 100

static unsigned int seed = RANDOMIZE_SEED;

/* Taken from http://software.intel.com/en-us/articles/fast-random-number-generator-on-the-intel-pentiumr-4-processor/
   http://en.wikipedia.org/wiki/Linear_congruential_generator
   And modified.  This is certainly not good random at all, but good enough. */

static inline unsigned
fastrand(unsigned upper)
{ 
  seed = (214013*seed+2531011); 
  return (seed >> 1) % upper;
}
#endif

/*
  --------------------------------------------------------------
  Stack generic functions
  --------------------------------------------------------------
*/

#define STACK_RESIZE_EXP(Pstack, n, size, type_size)	\
  if (size < n)						\
    {							\
      if (!size)					\
	size = 2;					\
      while (size < n)					\
        size *= 2;					\
      MY_REALLOC(Pstack, size * type_size);		\
    }							\

#define STACK_RESIZE_LIN(Pstack, n, size, type_size)	\
  if (size < n)						\
    {							\
      size = n;						\
      MY_REALLOC(Pstack, size * type_size);		\
    }							\

/*
  --------------------------------------------------------------
  Other generic functions
  --------------------------------------------------------------
*/

#define SWITCH_LIT(A,B) { Tlit tmp = A; A = B; B = tmp; }

/*
  --------------------------------------------------------------
  History (for top-level backtracking)
  --------------------------------------------------------------
*/

/**
   \defgroup history history datastructure (for top-level backtrackability)

   \brief these fields define the history stack.  This is related to
   top level backtrackability.  Top-level backtrackability allow to
   retrieve all user-added clauses in the LIFO style.

   \invariant history_size (the allocated size) >= history_n + 1
   \remark the 3 first bits of the clause have a special meaning

   \li if 000, the history field gives the id of the last clause
   before last non backtracked push.  It also acts as a markup for
   information relavant to a given push
   
   @{ */

#ifdef BACKTRACK
#define PUSH_MARKUP 0
#define STATUS_CHANGED 1
#define CLAUSE_UNSET_WATCHED 2
#ifndef PRESERVE_CLAUSES
#define SAVE_CLAUSE_FREE_LIST 3
#endif

typedef struct Thistory
{
  unsigned history_type:2;
  Tclause clause:30;
} Thistory;

static unsigned history_size = 0; /**< size of allocated stack */
static unsigned history_n = 0;    /**< nb of fields in history */
static Thistory * history = NULL;  /**< array of clauses id */
/** @} */

static inline void
history_status_changed(void)
{
  STACK_RESIZE_EXP(history, history_n + 1, history_size, sizeof(Thistory));
  history[history_n].history_type = STATUS_CHANGED;
  history[history_n++].clause = CLAUSE_UNDEF;
}

/*--------------------------------------------------------------*/

static inline void
history_clause_unset_watched(Tclause clause)
{
  STACK_RESIZE_EXP(history, history_n + 1, history_size, sizeof(Thistory));
  history[history_n].history_type = CLAUSE_UNSET_WATCHED;
  history[history_n++].clause = clause;
}
#endif /* BACKTRACK */

/*
  --------------------------------------------------------------
  Literal watch
  --------------------------------------------------------------
*/

typedef struct Twatch {
  unsigned n;
  unsigned size;
  Tclause * Pclause;
} Twatch;

Twatch * watch = NULL;

/**
   \author Pascal Fontaine
   \brief adds a clause to the watched clauses of literal
   \param lit the literal
   \param clause the clause */
static inline void
lit_watch(Tlit lit, Tclause clause)
{
  if (watch[lit].n == watch[lit].size)
    {
      watch[lit].size <<= 1;
      MY_REALLOC(watch[lit].Pclause,
		 (watch[lit].size * sizeof(Tclause)));
    }
  watch[lit].Pclause[watch[lit].n++] = clause;
}

/*--------------------------------------------------------------*/

#ifdef BACKTRACK
/**
   \author Pascal Fontaine
   \brief removes clause from the watched clauses of literal
   \param lit the literal
   \param clause the clause
   \remark a call to this function may be expensive */
static inline void
lit_watch_remove(Tlit lit, Tclause clause)
{
  Tclause * i = watch[lit].Pclause, * n = i + watch[lit].n;
  while (clause != *i)
    i++;
  do
    *i = *(i + 1);
  while (++i != n);
  watch[lit].n--;
}
#endif

/*
  --------------------------------------------------------------
  Variable
  --------------------------------------------------------------
*/

/**
   \author Pascal Fontaine
   \brief container for variable information */ 
typedef struct TSvar
{
  unsigned char phase_cache;  /**< previous polarity assignment */
  unsigned char seen;         /**< helper bit for conflict analyse */
  /* IMPROVE experiment with decide set only on real SMT atoms, not tseitin */
  unsigned decide:1;          /**< 1 iff decision on var is allowed */
  unsigned discarded:1;       /**< 1 iff discarded for minimal model */
  unsigned required:1;        /**< 1 iff selected for minimal model */
  unsigned misc:5;            /**< unused (for alignment) */
#ifdef PEDANTIC
  unsigned padding:8;
#endif
  SAT_Tlevel level;        /**< level of assignment */
  SAT_Tclause reason;      /**< clause responsible for propagation */
  double activity;         /**< variable activity */
} TSvar;

/**
   \defgroup SAT_stack_var variable stack
   \brief these fields define the stack for variables
   \invariant SAT_stack_var_n is always the maximum id of variables
   \invariant SAT_stack_var_size (the allocated size) >= SAT_stack_var_n + 1
   @{ */
static unsigned SAT_stack_var_size = 0; /**< size of allocated stack for vars */
unsigned SAT_stack_var_n = 0;           /**< highest var id in the stack */
static TSvar * SAT_stack_var = NULL;    /**< array of vars */
/** @} */

#ifdef SAT_SYM
/**
   \author Pascal Fontaine
   \brief in case symmetry is used, once a unit clause about a variable
   is deduced, unit clauses about all variables in the orbit are automatically
   added.  This array stores the orbit. */
Tvar * SAT_var_orbit = NULL;
#endif

static unsigned char * assign = NULL;   /**< assignment */

#ifdef HINT_AS_DECISION
static unsigned hint_n = 0;
static unsigned hint_p = 0;
static unsigned hint_size = 0;
static Tlit * hints = NULL;
#endif

#if defined(BCLAUSE) || defined(BCLAUSE_LIGHT)
static void bclause_resize(unsigned size_old, unsigned size_new);
#endif

/**
   \author Pascal Fontaine
   \brief creates a new variable
   \return the new variable id */
Tvar
SAT_var_new(void)
{
  if (SAT_status != SAT_STATUS_UNSAT)
    SAT_status = SAT_STATUS_UNDEF;
  SAT_stack_var_n++; /* PF var start at 1 */
  if (SAT_stack_var_size < SAT_stack_var_n + 1)
    {
      if (!SAT_stack_var_size)
	SAT_stack_var_size = 2;
      while (SAT_stack_var_size < SAT_stack_var_n + 1)
	SAT_stack_var_size *= 2;
      MY_REALLOC(SAT_stack_var, SAT_stack_var_size * sizeof(TSvar));
      MY_REALLOC(assign, SAT_stack_var_size * sizeof(Tvalue));
      MY_REALLOC(watch, (2 * SAT_stack_var_size * sizeof(Twatch)));
#if defined(BCLAUSE) || defined(BCLAUSE_LIGHT)
      bclause_resize(SAT_stack_var_n * 2, 2 * SAT_stack_var_size);
#endif
#ifdef SAT_SYM
      MY_REALLOC(SAT_var_orbit, SAT_stack_var_size * sizeof(Tvar));
#endif
      {
	unsigned i;
	for (i = SAT_stack_var_n * 2; i < 2 * SAT_stack_var_size; i++)
	  {
	    watch[i].n = 0;
	    watch[i].size = 2;
	    MY_MALLOC(watch[i].Pclause, 2 * sizeof(Tclause));
	  }
      }
    }
  assign[SAT_stack_var_n] = VAL_UNDEF;
  SAT_stack_var[SAT_stack_var_n].phase_cache = 0;
  SAT_stack_var[SAT_stack_var_n].seen = 0;
  SAT_stack_var[SAT_stack_var_n].decide = 1;
  SAT_stack_var[SAT_stack_var_n].discarded = 0;
  SAT_stack_var[SAT_stack_var_n].required = 0;
  SAT_stack_var[SAT_stack_var_n].misc = 0;
  SAT_stack_var[SAT_stack_var_n].level = 0;
  SAT_stack_var[SAT_stack_var_n].reason = CLAUSE_UNDEF;
  SAT_stack_var[SAT_stack_var_n].activity = 0;
#ifdef SAT_SYM
  SAT_var_orbit[SAT_stack_var_n] = VAR_UNDEF;
#endif
  var_order_insert(SAT_stack_var_n);
  return SAT_stack_var_n;
}

/*--------------------------------------------------------------*/

static inline void
SAT_var_free(Tvar var)
{
  free(watch[var<<1].Pclause);
  free(watch[(var<<1) + 1].Pclause);
  watch[var<<1].n = 0;
  watch[(var<<1) + 1].n = 0;
  watch[var<<1].size = 0;
  watch[(var<<1) + 1].size = 0;
  watch[var<<1].Pclause = NULL;
  watch[(var<<1) + 1].Pclause = NULL;
}

/*--------------------------------------------------------------*/

/**
   \author Pascal Fontaine
   \brief ensures variable of a given id exists
   \param id the variable id */
void
SAT_var_new_id(unsigned id)
{
  while (SAT_stack_var_n < id)
    SAT_var_new();
}

/*--------------------------------------------------------------*/

/**
   \author Pascal Fontaine
   \brief get the variable value
   \param var the variable
   \return the value (VAL_FALSE, VAL_TRUE, or VAL_UNDEF) */
inline Tvalue
SAT_var_value(Tvar var)
{
  assert(var <= SAT_stack_var_n);
  return assign[var];
}

/*--------------------------------------------------------------*/

/**
   \author Pascal Fontaine
   \brief disallow decision on var
   \param var the variable
   \remark while using this function you should know what you are
   doing.  The semantics of SAT/UNSAT depends on this */
void
SAT_var_block_decide(Tvar var)
{
  if (SAT_level != ROOT_LEVEL)
    my_error("SAT_var_block_decide call not at root level");
  SAT_stack_var[var].decide = 0;
}

/*--------------------------------------------------------------*/

/**
   \author Pascal Fontaine
   \brief allow decision on var
   \param var the variable
   \remark while using this function you should know what you are
   doing.  The semantics of SAT/UNSAT depends on this */
void
SAT_var_unblock_decide(Tvar var)
{
  if (SAT_level != ROOT_LEVEL)
    my_error("SAT_var_unblock_decide call not at root level");
  SAT_stack_var[var].decide = 1;
  var_order_insert(var);
}

/*--------------------------------------------------------------*/

/**
   \author Pascal Fontaine
   \brief check if variable can be chosen as a decision variable
   \param var the variable
   \return 1 if suitable for decision, 0 otherwise */
static inline unsigned
SAT_var_decision(Tvar var)
{
  assert(var <= SAT_stack_var_n);
  return SAT_stack_var[var].decide;
}

/*--------------------------------------------------------------*/

/**
   \author Pascal Fontaine
   \brief get the phase cache for variable
   \param var the variable
   \return 1 if positive polarity, 0 otherwise
   \remark this is set in var_set_value */
static inline unsigned
SAT_var_phase_cache(Tvar var)
{
  assert(var <= SAT_stack_var_n);
  return SAT_stack_var[var].phase_cache;
}

/*--------------------------------------------------------------*/

/**
   \author Pascal Fontaine
   \brief check if variable seen
   \param var the variable
   \return 1 iff seen */
static inline unsigned
SAT_var_seen(Tvar var)
{
  assert(var <= SAT_stack_var_n);
  return SAT_stack_var[var].seen;
}

/*--------------------------------------------------------------*/

/**
   \author Pascal Fontaine
   \brief set variable as seen
   \param var the variable */
static inline void
SAT_var_set_seen(Tvar var)
{
  assert(var <= SAT_stack_var_n);
  SAT_stack_var[var].seen = 1;
}

/*--------------------------------------------------------------*/

/**
   \author Pascal Fontaine
   \brief set variable as unseen
   \param var the variable */
static inline void
SAT_var_set_unseen(Tvar var)
{
  assert(var <= SAT_stack_var_n);
  SAT_stack_var[var].seen = 0;
}

/*--------------------------------------------------------------*/

/**
   \author Pascal Fontaine
   \brief access variable activity
   \param v the variable */
#define SAT_var_activity(v) SAT_stack_var[v].activity

/*--------------------------------------------------------------*/

/**
   \author Pascal Fontaine
   \brief order for variable decision
   \param v1 the first variable
   \param v2 the second variable
   \return 1 if first variable should be higher in the heap (less) */
static inline int
SAT_var_less(Tvar v1, Tvar v2)
{
  return SAT_var_activity(v1) > SAT_var_activity(v2);
}

/*--------------------------------------------------------------*/

/**
   \author Pascal Fontaine
   \brief get the variable level
   \param var the variable
   \return the level at which variable has been assigned */
inline Tlevel
SAT_var_level(Tvar var)
{
  return SAT_stack_var[var].level;
}

/*--------------------------------------------------------------*/

/**
   \author Pascal Fontaine
   \brief get the reason for variable value
   \param var the variable
   \return the clause that propagated the variable */
static inline Tclause
SAT_var_reason(Tvar var)
{
  assert (assign[var] != VAL_UNDEF);
  return SAT_stack_var[var].reason;
}

/*--------------------------------------------------------------*/

/**
   \author Pascal Fontaine
   \brief set the reason for variable value (lazy clause)
   \param var the variable
   \param reason the clause */
static inline void
SAT_var_set_reason(Tvar var, Tclause reason)
{
  assert (SAT_stack_var[var].reason == CLAUSE_LAZY);
  SAT_stack_var[var].reason = reason;
}

/*--------------------------------------------------------------*/

/**
   \author Pascal Fontaine
   \brief set the value associated with the variable
   \param var the variable
   \param value the value to set
   \param level the level at which the variable is asserted
   \param reason the clause (CLAUSE_UNDEF for decisions) propagating the val
   \return 1 if succeeded, 0 if conflict */
static inline void
var_set_value(Tvar var, Tvalue value, Tlevel level, Tclause reason)
{
  assert(assign[var] == VAL_UNDEF);
  assign[var] = value;
  assert(SAT_stack_var[var].reason == CLAUSE_UNDEF);
  SAT_stack_var[var].reason = reason;
  SAT_stack_var[var].level = level;
  SAT_stack_var[var].phase_cache = value & 1; /* (value == VAL_TRUE)?1:0; */
}

/*--------------------------------------------------------------*/

/**
   \author Pascal Fontaine
   \brief unset the value associated with the variable
   \param var the variable to unset */
static inline void
var_unset(Tvar var)
{
  assert(assign[var] != VAL_UNDEF);
  assign[var] = VAL_UNDEF;
  SAT_stack_var[var].reason = CLAUSE_UNDEF;
  SAT_stack_var[var].level = 0;
  var_order_insert(var);
}

/*
  --------------------------------------------------------------
  phase cache setting
  --------------------------------------------------------------
*/

void
SAT_phase_cache_set(void)
{
  /*   unsigned i; */
  /*   for (i = 1; i <= SAT_stack_var_n; i++) */
  /* 17 Fail, 6579 < 5s */
  /*    SAT_stack_var[i].phase_cache = 1; */
  /* 16 Fail, 6585 < 5s */
  /*   SAT_stack_var[i].phase_cache = 0; */
  /* 19 Fail, 6578 < 5s */
  /* SAT_stack_var[i].phase_cache = */
  /*   SAT_stack_var[i].watch_n[VAL_TRUE] > SAT_stack_var[i].watch_n[VAL_FALSE]; */
  /* 19 Fail, 6581 < 5s */
  /* SAT_stack_var[i].phase_cache = */
  /*   SAT_stack_var[i].watch_n[VAL_TRUE] >= SAT_stack_var[i].watch_n[VAL_FALSE]; */
  /* 17 Fail, 6586 < 5s */
  /*   SAT_stack_var[i].phase_cache = */
  /*     SAT_stack_var[i].watch_n[VAL_TRUE] < SAT_stack_var[i].watch_n[VAL_FALSE]; */
  /* 19 Fail, 6579 < 5s */
  /* SAT_stack_var[i].phase_cache = */
  /*   SAT_stack_var[i].watch_n[VAL_TRUE] <= SAT_stack_var[i].watch_n[VAL_FALSE]; */
}

/*
  --------------------------------------------------------------
  var heap and activity
  --------------------------------------------------------------
*/

static unsigned heap_var_n = 0;
static unsigned heap_var_size = 0;
static Tvar * heap_var = NULL;
static unsigned heap_index_size = 0;
static unsigned * heap_index = NULL;

#define HEAP_INDEX_UNDEF UINT_MAX

static inline unsigned
left(unsigned i)
{
  return i*2+1;
}

/*--------------------------------------------------------------*/

static inline unsigned
right(unsigned i)
{
  return i*2+2;
}

/*--------------------------------------------------------------*/

static inline unsigned
parent(unsigned i)
{
  return (i-1)>>1;
}

/*--------------------------------------------------------------*/

static inline void
sift_up(unsigned i)
{
  Tvar var = heap_var[i];
  unsigned p = parent(i);
  while (i && SAT_var_less(var, heap_var[p]))
    {
      heap_var[i] = heap_var[p];
      heap_index[heap_var[p]] = i;
      i = p;
      p = parent(p);
    }
  heap_var[i] = var;
  heap_index[var] = i;
}

/*--------------------------------------------------------------*/

static inline void
sift_down(unsigned i)
{
  Tvar var = heap_var[i];
  while (left(i) < heap_var_n)
    {
      unsigned child;
      if (right(i) < heap_var_n &&
	  SAT_var_less(heap_var[right(i)], heap_var[left(i)]))
	child = right(i);
      else
	child = left(i);
      if (!SAT_var_less(heap_var[child], var))
	break;
      heap_var[i] = heap_var[child];
      heap_index[heap_var[child]] = i;
      i = child;
    }
  heap_var[i] = var;
  heap_index[var] = i;
}

/*--------------------------------------------------------------*/

static inline int
heap_var_in(Tvar var)
{
  assert (var != VAR_UNDEF);
  return var < heap_index_size && heap_index[var] != HEAP_INDEX_UNDEF;
}

/*--------------------------------------------------------------*/

static inline void
heap_var_insert(Tvar var)
{
  assert(var!=VAR_UNDEF);
  if (!heap_var_size)
    {
      MY_MALLOC(heap_var, 2 * sizeof(Tvar));
      heap_var_size = 2;
    }
  while (heap_var_size < heap_var_n + 1)
    {
      heap_var_size *= 2;
      MY_REALLOC(heap_var, heap_var_size * sizeof(Tvar));
    }

  if (heap_index_size < SAT_stack_var_size)
    {
      unsigned i;
      MY_REALLOC(heap_index, SAT_stack_var_size * sizeof(int));
      for (i = heap_index_size; i < SAT_stack_var_size; ++i)
	heap_index[i] = HEAP_INDEX_UNDEF;
      heap_index_size = SAT_stack_var_size;
    }
  assert(!heap_var_in(var));
  heap_var[heap_var_n] = var;
  heap_index[var] = heap_var_n;
  sift_up(heap_var_n++);
}

/*--------------------------------------------------------------*/

static inline void
heap_var_decrease(Tvar var)
{
  assert(heap_var_in(var));
  sift_up(heap_index[var]);
}

/*--------------------------------------------------------------*/

#if 0
static inline void
heap_var_increase(Tvar var)
{
  assert(heap_var_in(var));
  sift_down(heap_index[var]);
}
#endif

/*--------------------------------------------------------------*/

static inline Tvar
heap_var_remove_min(void)
{
  Tvar var = heap_var[0];
  heap_index[var] = HEAP_INDEX_UNDEF;
  heap_var[0] = heap_var[--heap_var_n];
  if (heap_var_n)
    sift_down(0); /* index will be set in sift_down */
  return var;
}

/*--------------------------------------------------------------*/

static inline Tvar
heap_var_get_min(void)
{
  return heap_var[0];
}

/*--------------------------------------------------------------*/

#if 0
static inline void
heap_var_update(Tvar var)
{
  if (!heap_var_in(var))
    heap_var_insert(var);
  else
    {
      sift_up(heap_index[var]);
      sift_down(heap_index[var]);
    }
}
#endif

/*--------------------------------------------------------------*/

static inline int
heap_var_empty(void)
{
  return heap_var_n == 0;
}

/*--------------------------------------------------------------*/

#if 0
static void
heap_var_build(Tvar * vs, unsigned n)
{
  int i;
  heap_var_n = 0;

  for (i = 0; i < (int) n; i++)
    {
      heap_index[vs[i]] = i;
      heap_var[heap_var_n++] = vs[i];
    }

  for (i = heap_var_n / 2 - 1; i >= 0; i--)
    sift_down(i);
}
#endif

/*--------------------------------------------------------------*/

static inline void
heap_var_free(void)
{
  heap_var_n = 0;
  free(heap_var);
  heap_var = NULL;
  heap_var_size = 0;
  free(heap_index);
  heap_index = NULL;
  heap_index_size = 0;
}

/*--------------------------------------------------------------*/

static double var_inc = 1;
static double var_decay = 0.95;

/*--------------------------------------------------------------*/

static inline void
var_order_insert(Tvar var)
{
  if (!heap_var_in(var) && SAT_var_decision(var))
    heap_var_insert(var);
}

/*--------------------------------------------------------------*/

static inline void
var_decrease_activity(void)
{
  var_inc /= var_decay;
}

/*--------------------------------------------------------------*/

static inline void
var_increase_activity(Tvar var)
{
  if ( (SAT_var_activity(var) += var_inc) > 1e100 )
    {
      unsigned i;
      /* rescale all activity */
      for (i = 1; i <= SAT_stack_var_n; i++)
	SAT_var_activity(i) *= 1e-100;
      var_inc *= 1e-100;
    }
  /* Update order_heap with respect to new activity: */
  if (heap_var_in(var))
    heap_var_decrease(var);
}

/*
  --------------------------------------------------------------
  Literal
  --------------------------------------------------------------
*/

inline Tvalue
SAT_lit_value(Tlit lit)
{
#ifdef PEDANTIC
  Tvalue tmp = SAT_lit_pol(lit) ^ 1;
  tmp ^= SAT_var_value(SAT_lit_var(lit));
  return tmp;
#else
  return SAT_var_value(SAT_lit_var(lit)) ^ (SAT_lit_pol(lit) ^ 1);
#endif
}

/*--------------------------------------------------------------*/

/**
   \author Pascal Fontaine
   \brief get the value associated with the literal
   \param lit the literal
   \return the value (VAL_FALSE, VAL_TRUE, or VAL_UNDEF) */
static inline Tvalue
SAT_lit_value_undef(Tlit lit)
{
  return SAT_var_value(SAT_lit_var(lit)) == VAL_UNDEF;
}

/*--------------------------------------------------------------*/

/**
   \author Pascal Fontaine
   \brief get the value associated with the literal
   \param lit the literal
   \return the value (VAL_FALSE, VAL_TRUE, or VAL_UNDEF) */
static inline int
SAT_lit_value_is_true(Tlit lit)
{
  return (SAT_var_value(SAT_lit_var(lit)) ^ SAT_lit_pol(lit)) == VAL_FALSE;
}

/*--------------------------------------------------------------*/

inline Tlevel
SAT_lit_level(Tlit lit)
{
  return SAT_var_level(SAT_lit_var(lit));
}

/*--------------------------------------------------------------*/

/**
   \author Pascal Fontaine
   \brief check if literal has been seen
   \param lit the literal
   \return 1 if seen, 0 otherwise */
static inline unsigned
SAT_lit_seen(Tlit lit)
{
  return SAT_var_seen(SAT_lit_var(lit));
}

/*--------------------------------------------------------------*/

/**
   \author Pascal Fontaine
   \brief set variable as seen
   \param lit the literal */
static inline void
SAT_lit_set_seen(Tlit lit)
{
  SAT_var_set_seen(SAT_lit_var(lit));
}

/*--------------------------------------------------------------*/

/**
   \author Pascal Fontaine
   \brief set literal as unseen
   \param lit the literal */
static inline void
SAT_lit_set_unseen(Tlit lit)
{
  SAT_var_set_unseen(SAT_lit_var(lit));
}

/*--------------------------------------------------------------*/

/**
   \author Pascal Fontaine
   \brief get the reason for literal value
   \param lit the literal
   \return the clause that propagated the literal */
static inline Tclause
SAT_lit_reason(Tlit lit)
{
  return SAT_var_reason(SAT_lit_var(lit));
}

/*--------------------------------------------------------------*/

/**
   \author Pascal Fontaine
   \brief compare two literals (for qsort)
   \param lit1 pointer to the first literal
   \param lit2 pointer to the second literal
   \return an integer less than, equal to, or greater than zero if the
       first argument is considered to be respectively less than,
       equal to, or greater than the second */
static int
SAT_lit_compare(const Tlit * lit1, const Tlit * lit2)
{
  Tvar var1 = SAT_lit_var(*lit1);
  Tvar var2 = SAT_lit_var(*lit2);
  if (var1 != var2) return (int) var1 - (int) var2;
  if (*lit1 == *lit2) return 0;
  if (SAT_lit_pol(*lit1)) return 1;
  return -1;
}

/*--------------------------------------------------------------*/

/**
   \author Pascal Fontaine
   \brief compare two literals (for qsort), according to an order for
   watch list
   \param Plit1 pointer to the first literal
   \param Plit2 pointer to the second literal
   \return an integer less than, equal to, or greater than zero if the
   first argument is considered to be respectively less than, equal
   to, or greater than the second
   \remark true literals should be first by increasing level
   \remark undefined literals should be in the middle, in whatever order
   \remark false literals should be last in decreasing level */
static int
SAT_lit_compare_level(const Tlit * Plit1, const Tlit * Plit2)
{
  switch (SAT_lit_value(*Plit1))
    {
    case VAL_TRUE:
      if (SAT_lit_value(*Plit2) == VAL_TRUE)
	return ((int)SAT_lit_level(*Plit1)) - ((int)SAT_lit_level(*Plit2));
      return -1;
    case VAL_FALSE:
      if (SAT_lit_value(*Plit2) == VAL_FALSE)
	return ((int)SAT_lit_level(*Plit2)) - ((int)SAT_lit_level(*Plit1));
      return 1;
    default:
      switch (SAT_lit_value(*Plit2))
	{
	case VAL_TRUE: return 1;
	case VAL_FALSE: return -1;
	default: return ((int)SAT_lit_var(*Plit1)) - ((int)SAT_lit_var(*Plit2));
	}
    }
  return 0;
}

/*
  --------------------------------------------------------------
  Literal stack
  --------------------------------------------------------------
*/

#define stack_lit SAT_literal_stack
#define stack_lit_n SAT_literal_stack_n
#define stack_lit_hold SAT_literal_stack_hold
#define stack_lit_unit SAT_literal_stack_unit
#define stack_lit_to_propagate SAT_literal_stack_to_propagate

/**
   \defgroup stack_lit literal stack
   \brief these fields define the stack for literals
   \invariant stack_lit_n is the index of the next literal
   \invariant stack_lit_size (the allocated size) >= stack_lit_n
   @{ */
static unsigned  stack_lit_size = 0;  /**< size of allocated stack for literals */
unsigned  stack_lit_n = 0;     /**< index of next position */
Tlit     *stack_lit = NULL;    /**< array of literals */
unsigned  stack_lit_to_propagate = 0; /**< index to the next literal to propagate */
unsigned  stack_lit_hold = 0;
unsigned  stack_lit_unit = 0;

/**
   \author Pascal Fontaine
   \brief add literal to the stack
   \param lit the literal to add
   \param reason clause propagating the literal (CLAUSE_UNDEF if decision) */
static inline void
stack_lit_add(Tlit lit, Tclause reason)
{
  STACK_RESIZE_EXP(stack_lit, stack_lit_n + 1, stack_lit_size, sizeof(Tlit));
  stack_lit[stack_lit_n++] = lit;
  /* printf("stack_lit_add %d %d\n", lit, reason); */
  var_set_value(SAT_lit_var(lit), SAT_lit_pol(lit), SAT_level, reason);
  if (SAT_level == ROOT_LEVEL)
    {
#ifdef PROOF
      if (!SAT_proof)
	SAT_lit_set_seen(lit);
#else
      SAT_lit_set_seen(lit); /* TODO This should be backtracked somewhere */
#endif
      stack_lit_unit = stack_lit_n;
    }
}

/*--------------------------------------------------------------*/

static inline Tlit
stack_lit_get(unsigned index)
{
  return stack_lit[index];
}

/** @} */

/*
  --------------------------------------------------------------
  Level
  --------------------------------------------------------------
*/

#define stack_level SAT_level_stack
#define stack_level_hold SAT_level_stack_hold

/**
   \defgroup stack_level level stack
   \brief these fields define the stack for levels.  It is remembered
   which position in the stack is the first to correspond to next level
   \invariant SAT_level is the index of the next stack_lit index
   \invariant stack_level_size (the allocated size) >= SAT_level
   \invariant stack_level[i] is the first literal asserted at level i + 1
   @{ */
unsigned stack_level_size = 0;
Tlevel * stack_level = NULL;
unsigned stack_level_hold = 0;

/*--------------------------------------------------------------*/

/**
   \author Pascal Fontaine
   \brief add decision literal to the stack, changing the level
   \param lit the literal to add */
static inline void
level_push(Tlit lit)
{
  STACK_RESIZE_EXP(stack_level, SAT_level + 1,
		   stack_level_size, sizeof(unsigned));
  stack_level[SAT_level] = stack_lit_n;
  SAT_level++;
  stack_lit_add(lit, CLAUSE_UNDEF);
}

/*--------------------------------------------------------------*/

/**
   \author Pascal Fontaine
   \brief backtrack to a given level
   \param level the level not to backtrack */
static inline void
level_backtrack(Tlevel level)
{
  unsigned stack_lit_bt;
  if (level >= SAT_level)
    return;
#ifdef HINT_AS_DECISION
  hint_n = hint_p = 0;
#endif
  stack_lit_bt = stack_level[level];
  assert (stack_lit_to_propagate >= stack_lit_bt);
  SAT_level = level;
  /* PF there is no particular reason to do this backwards but easier for
     debugging purposes */
  while (stack_lit_n > stack_lit_bt)
    {
      stack_lit_n--;
      var_unset(SAT_lit_var(stack_lit_get(stack_lit_n)));
    }
  assert(stack_lit_n == stack_lit_bt);
  stack_lit_n = stack_lit_to_propagate = stack_lit_bt;
  if (stack_lit_n < stack_lit_hold)
    stack_lit_hold = stack_lit_n;
  if (level < stack_level_hold)
    stack_level_hold = level;
}

/** @} */

/*--------------------------------------------------------------*/

/**
   \author Rodrigo Castano
   \brief Find a decision literal in the trail with lower score than the
   next decision literal (top of the heap, discarding already assigned)
   and return its level
   \remark returns UINT_MAX if there's none */
#ifdef REUSE_TRAIL
static inline Tlevel
find_level_on_restart(void)
{
  unsigned i;
  double next_decision_act;
  Tvar var;
  while (1)
    {
      assert(!heap_var_empty());
      var = heap_var_get_min();
      if (SAT_var_value(var) == VAL_UNDEF && SAT_var_decision(var))
	break;
      heap_var_remove_min();
    }
  next_decision_act = SAT_var_activity(var);
  /* Iterate over all literals in the trail (literal stack) */
  for (i = 0; i < stack_lit_n; ++i) 
    if (!SAT_var_reason(var = SAT_lit_var(stack_lit[i])) &&
	SAT_var_activity(var) < next_decision_act)
      return SAT_var_level(var) - 1; 
  return UINT_MAX;
}
#else
static inline Tlevel
find_level_on_restart(void)
{
  return ROOT_LEVEL;
}
#endif /* REUSE_TRAIL */

/*
  --------------------------------------------------------------
  Clause
  --------------------------------------------------------------
*/

/**
   \author Pascal Fontaine
   \brief Main data-structure for representing clauses */
typedef struct TSclause
{
#ifdef PEDANTIC
  unsigned n;
  unsigned char deleted;
  unsigned char conflict;
  unsigned char learnt;
  unsigned char watched;
#else
  unsigned n:28;        /**< number of literals */
  unsigned deleted:1;   /**< deleted */
  unsigned conflict:1;  /**< conflict clause by external solver */
  unsigned learnt:1;    /**< is it a learnt clause */
  unsigned watched:1;   /**< clause is in watch lists
			     (any but empty, unit, or valid) */
#endif
  double activity;      /**< variable activity */
  Tlit blocker;
  Tlit * lit;           /**< literals */
} TSclause;
/* MiniSAT uses memory after clause to store literals.  Here it is not
   possible since clause stack is an array */

#ifndef PRESERVE_CLAUSES
static unsigned first_free_clause = CLAUSE_UNDEF;
#endif

/**
   \defgroup stack_clause clause stack
   \brief these fields define the stack for clauses
   \invariant stack_clause_n is always the maximum clause id
   \invariant stack_clause_size (the allocated size) >= stack_clause_n + 1
   @{ */
static unsigned stack_clause_size = 0; /**< size of allocated stack */
static unsigned stack_clause_n = 0;    /**< highest clause id in the stack */
static TSclause * stack_clause = NULL; /**< array of clauses */
/** @} */

static inline void
clause_learnts_push(Tclause clause);
#if defined(BCLAUSE) || defined(BCLAUSE_LIGHT)
static inline void
bclause_add(Tclause clause, Tlit lit1, Tlit lit2);
#endif

/*--------------------------------------------------------------*/

/**
   \author Pascal Fontaine
   \brief adds a clause
   \param n the number of literals
   \param lit an array of n literals
   \param learnt tag
   \param watched add in watch lists
   \param conflict a flag indicating if the clause is a conflict clause.
   \return clause id
   \remark watches are the first two literals.  Choose adequately (IMPROVE)
   \remark memory allocated by lit is confiscated, but can still be used (not freed or moved)
   while no simplification occur */
static inline Tclause
clause_new(unsigned n, Tlit * lit,
	   unsigned char learnt, unsigned char watched, unsigned char conflict)
{
  TSclause * PSclause;
  Tclause clause;
#if STATS_LEVEL >= 4
  if (n == 1)
    {
      static int j = 1;
      fprintf(stderr, "Unit clause added %d\n", j++);
      fprintf(stderr, "Subsumed %d, Strengthened %d\n",
	      tmp_check_subsumed_unit(lit[0]),
	      tmp_check_strengthenable_unit(lit[0]));
    }
  if (n == 2)
    {
      static j = 1;
      fprintf(stderr, "Bin clause added %d\n", j++);
    }
#endif
#ifndef PRESERVE_CLAUSES
  if (first_free_clause != CLAUSE_UNDEF)
    {
      clause = first_free_clause;
      PSclause = stack_clause + clause;
      first_free_clause = PSclause->n;
    }
  else
#endif
    {
      ++stack_clause_n;
      if (stack_clause_n == 1<<30)
	my_error ("too many clauses\n");
      if (n >= 1<<27)
	my_error ("too many literals in clause\n");
      STACK_RESIZE_EXP(stack_clause, stack_clause_n + 1,
		       stack_clause_size, sizeof(TSclause));
      clause = stack_clause_n;
      PSclause = stack_clause + clause;
    }
#if STATS_LEVEL >= 1
  stats_counter_inc(stat_n_clauses);
#endif
  PSclause->n = n;
  PSclause->lit = lit;
  PSclause->learnt = learnt;
  if (learnt)
    clause_learnts_push(clause);
  PSclause->deleted = 0;
  PSclause->activity = 0;
  PSclause->conflict = conflict;
  PSclause->blocker = LIT_UNDEF;
#if defined(BCLAUSE) || defined(BCLAUSE_LIGHT)
  PSclause->watched = watched && (n >= 3);
  if (watched && n >= 3)
    {
      lit_watch(PSclause->lit[0], clause);
      lit_watch(PSclause->lit[1], clause);
    }
  if (n == 2)
    bclause_add(clause, PSclause->lit[0], PSclause->lit[1]);
#else
  PSclause->watched = watched && (n >= 2);
  if (watched && n >= 2)
    {
      lit_watch(PSclause->lit[0], clause);
      lit_watch(PSclause->lit[1], clause);
    }
#endif
  return clause;
}

/*--------------------------------------------------------------*/

#if STATS_LEVEL >= 4
int
tmp_check_subsumed_unit(Tlit lit)
{
  unsigned i, j, n = 0;
  for (i = 1; i < stack_clause_n; i++)
    if (stack_clause[i].deleted || !stack_clause[i].watched)
      continue;
    else
      {
	for (j = 0; j < stack_clause[i].n; j++)
	  if (lit == stack_clause[i].lit[j])
	    n++;
      }
  return n;
}

/*--------------------------------------------------------------*/

int
tmp_check_strengthenable_unit(Tlit lit)
{
  unsigned i, j, n = 0;
  for (i = 1; i < stack_clause_n; i++)
    if (stack_clause[i].deleted || !stack_clause[i].watched)
      continue;
    else
      {
	for (j = 0; j < stack_clause[i].n; j++)
	  if (lit == -stack_clause[i].lit[j])
	    n++;
      }
  return n;
}
#endif

/*--------------------------------------------------------------*/

/**
   \author Pascal Fontaine
   \brief remove a clause
   \param clause the clause to remove
   \remark this is only used in SAT_done, so minimalistic work accepted */
static void
SAT_clause_free(Tclause clause)
{
  free(stack_clause[clause].lit);
}

/*--------------------------------------------------------------*/

#if 0
/**
   \author Pascal Fontaine
   \brief desactivate a clause
   \param clause the clause to desactivate */
static void
clause_unset_watched(Tclause clause)
{
  TSclause * PSclause = stack_clause + clause;
  if (!PSclause->watched)
    return;
  lit_watch_remove(PSclause->lit[0], clause);
  lit_watch_remove(PSclause->lit[1], clause);
  PSclause->watched = 0;
}
#endif

/*--------------------------------------------------------------*/

#ifdef BACKTRACK
/**
   \author Pascal Fontaine
   \brief activate clause
   \param clause the clause to activate */
static void
clause_set_watched(Tclause clause)
{
  TSclause * PSclause = stack_clause + clause;
  if (PSclause->watched || PSclause->n < 2)
    return;
  lit_watch(PSclause->lit[0], clause);
  lit_watch(PSclause->lit[1], clause);
  PSclause->watched = 1;
}
#endif

/*--------------------------------------------------------------*/

#ifdef BACKTRACK
/**
   \author Pascal Fontaine
   \brief remove a clause
   \param clause the clause to remove
   \remark this is only used in popping, so no need to update
   first_free_clause, or to put fields in an acceptable state */
static void
clause_remove(Tclause clause)
{
  TSclause * PSclause = stack_clause + clause;
  if (PSclause->watched)
    {
      lit_watch_remove(PSclause->lit[0], clause);
      lit_watch_remove(PSclause->lit[1], clause);
    }
  free(PSclause->lit);
}
#endif

/*--------------------------------------------------------------*/

/**
   \author Pascal Fontaine
   \brief remove a clause
   \param clause the clause to remove
   \remark this is only used in clause_purge */
static inline void
clause_lazy_delete(Tclause clause)
{
  TSclause * PSclause = stack_clause + clause;
  if (PSclause->deleted)
    return;
  PSclause->deleted = 1;
  PSclause->watched = 0;
#ifndef PRESERVE_CLAUSES
#ifdef PROOF
  if (SAT_proof)
    return;
#endif
  PSclause->n = first_free_clause;
  first_free_clause = clause;
  free(PSclause->lit);
  PSclause->lit = NULL;
  PSclause->learnt = 0;
#endif
#if STATS_LEVEL >= 1
  stats_counter_inc(stat_n_delete);
#endif
}

/*--------------------------------------------------------------*/

#if defined(PROOF_PRINT_CLAUSES) || defined(DEBUG)
void
clause_print(Tclause clause)
{
  unsigned i;
  TSclause * PSclause = stack_clause + clause;
  if (PSclause->deleted)
    {
      fprintf(stderr, "D");
      return;
    }
  if (!PSclause->watched)
    fprintf(stderr, "Unwatched");
  if (PSclause->conflict)
    fprintf(stderr, "C");
  if (PSclause->learnt)
    fprintf(stderr, "L");
  for (i = 0; i < PSclause->n; i++)
    fprintf(stderr, i?" %d":"%d", PSclause->lit[i]);
}
#endif

/*--------------------------------------------------------------*/

#ifdef DEBUG
void
clause_print_all(void)
{
  unsigned i;
  for (i = 1; i < stack_clause_n; i++)
    {
      fprintf(stderr, "%d : ", i);
      clause_print(i);
      fprintf(stderr, "\n");
    }
}
#endif

/*--------------------------------------------------------------*/

#if 0
static int
clause_by_size(const Tclause * Pclause1, const Tclause * Pclause2)
{
  return stack_clause[*Pclause1].n - stack_clause[*Pclause2].n;
}
#endif

/*
  --------------------------------------------------------------
  Clause activity/purge
  --------------------------------------------------------------
*/

static double clause_inc = 1;
static double clause_decay = 0.999;

static Tclause * learnts = NULL;
static unsigned learnts_n = 0;
static unsigned learnts_size = 0;

/*--------------------------------------------------------------*/

/**
   \author Pascal Fontaine
   \brief access clause activity
   \param clause the clause */
#define clause_activity(clause) stack_clause[clause].activity

/*--------------------------------------------------------------*/

static inline void
clause_decrease_activity(void)
{
  clause_inc /= clause_decay;
}

/*--------------------------------------------------------------*/

static inline void
clause_increase_activity(Tclause clause)
{
  if ( (clause_activity(clause) += clause_inc) > 1e20 )
    {
      unsigned i;
      /* rescale all activity */
      for (i = 0; i < learnts_n; i++)
	clause_activity(learnts[i]) *= 1e-20;
      clause_inc *= 1e-20;
    }
}

/*--------------------------------------------------------------*/

static inline void
clause_learnts_push(Tclause clause)
{
  learnts_n++;
  STACK_RESIZE_EXP(learnts, learnts_n, learnts_size, sizeof(Tclause));
  learnts[learnts_n - 1] = clause;
}

/*--------------------------------------------------------------*/

static inline int
clause_propagating(Tclause clause)
{
  return SAT_lit_value(stack_clause[clause].lit[0]) == VAL_TRUE &&
    SAT_lit_reason(stack_clause[clause].lit[0]) == clause;
}

/*--------------------------------------------------------------*/

/**
   \brief remove all deleted clauses from watch lists */
static inline void
batch_rm_clauses_1(void)
{
  unsigned i;
  for (i = 2; i <= (SAT_stack_var_n << 1) + 1; i++)
    if (watch[i].n)
      {
	Tclause *j, *k ,*n;
	j = k = watch[i].Pclause;
	n = j + watch[i].n;
	for (; j != n; j++)
	  if (!stack_clause[*j].deleted && stack_clause[*j].watched)
	    *(k++) = *j;
	watch[i].n -= (unsigned) (j - k);
      }
}

/*--------------------------------------------------------------*/

/* lowest activity will be at the end */
static int
cmp_clause(Tclause * clause1, Tclause * clause2)
{
  /*  if (stack_clause[*clause1].n <= 2)
    {
      if (stack_clause[*clause2].n > 2)
	return -1;
      return 0;
    }
  if (stack_clause[*clause2].n <= 2)
  return 1; */
  if (stack_clause[*clause1].activity < stack_clause[*clause2].activity)
    return 1;
  /*  if (stack_clause[*clause1].activity == stack_clause[*clause2].activity)
      return 0; */
  return -1;
}

/*--------------------------------------------------------------*/


/**
   \author Pascal Fontaine
   \brief remove false literals from clauses, and true clauses */
static inline void 
purge_valid(void)
{
#ifdef SIMP
  TSclause * i, *n;
  assert (SAT_level == ROOT_LEVEL);
  assert (SAT_status != SAT_STATUS_UNSAT);
  assert (stack_lit_to_propagate == stack_lit_n);
  i = stack_clause + 1;
  n = stack_clause + stack_clause_n + 1;
  for (; i != n; i++)
    {
      Tlit * k, *l, *m;
      if (i->deleted || !i->watched ||
	  (SAT_lit_value(i->lit[0]) == VAL_TRUE &&
	   stack_clause + SAT_lit_reason(i->lit[0]) == i))
	continue;
      if (i->learnt)
	{
	  k = l = i->lit;
	  m = i->lit + i->n;
	  for (; k != m; k++)
	    if (SAT_lit_value_undef(*k))
	      *(l++) = *k;
	    else if (SAT_lit_value_is_true(*k))
	      {
		clause_lazy_delete((unsigned)(i - stack_clause));
		goto super_continue;
	      }
	  i->n -= (unsigned) (k - l);
	  continue;
	}
      /* PF do not remove literals from non-learnt clause
	 (for backtracking) */
#ifdef BACKTRACK
      k = i->lit;
      m = i->lit + i->n;
      for (; k != m; k++)
	if (SAT_lit_value_is_true(*k))
	  {
	    i->watched = 0;
	    history_clause_unset_watched(i - stack_clause);
	    break;
	  }
#else
      k = l = i->lit;
      m = i->lit + i->n;
      for (; k != m; k++)
	if (SAT_lit_value_undef(*k))
	  *(l++) = *k;
	else if (SAT_lit_value_is_true(*k))
	  {
	    i->watched = 0;
	    goto super_continue;
	  }
      i->n -= (unsigned) (k - l);
#if defined(BCLAUSE) || defined(BCLAUSE_LIGHT)
      if (i->n == 2)
	{
	  i->watched = 0;
	  bclause_add(i - stack_clause, i->lit[0], i->lit[1]);
	}
#endif
#endif /* BACKTRACK */
    super_continue: ;
    }
  {
    Tclause * i, * j, * n;
    i = j = learnts;
    n = learnts + learnts_n;
    for (; i != n; i++)
      if (!stack_clause[*i].deleted)
	*(j++) = *i;
    learnts_n -= (unsigned) (n - j);
  }
  batch_rm_clauses_1();
#endif /* SIMP */
}

/*--------------------------------------------------------------*/

static inline void 
purge(void)
{
  /* TODO could try to be a bit more agressive towards purging:
     - sort binary clauses to the beginning,
     - eliminate all clauses beyond threshold
     - remove a portion of the rest */
  double threshold = clause_inc / learnts_n;
  Tclause * i, * j, *n;
#if STATS_LEVEL >= 1
  stats_counter_inc(stat_n_purge);
#endif /* STATS_LEVEL >= 1 */
#if STATS_LEVEL >= 4
  fprintf(stderr, "Starting purge\n");
#endif
  veriT_qsort(learnts, learnts_n, sizeof(Tclause), (TFcmp) cmp_clause);
  /*  n = learnts + learnts_n / 2;
  for (i = learnts; i != n; i++)
  fprintf(stderr, "%f\n", stack_clause[*i].activity); */

  i = j = learnts;
  n = learnts + learnts_n / 2;
  for (; i != n; i++)
    if (stack_clause[*i].activity < threshold && stack_clause[*i].n > 2 &&
	!clause_propagating(*i))
      clause_lazy_delete(*i);
    else
      *(j++) = *i;
  assert (i == learnts + learnts_n / 2);
  n = learnts + learnts_n;
  for (; i != n; i++)
    if (stack_clause[*i].n > 2 && !clause_propagating(*i))
      clause_lazy_delete(*i);
    else
      *(j++) = *i;
#if STATS_LEVEL >= 4
  fprintf(stderr, "%d learnt, %ld eliminated\n", learnts_n, n-j);
#endif
  learnts_n -= (unsigned) (n - j);
  batch_rm_clauses_1();
}

/*
  TODO
  Eliminate satisfied clauses
*/

/*
  --------------------------------------------------------------
  Purging
  --------------------------------------------------------------
*/

#if 0
/*
  Light purging maintains the database free of clauses
  that are subsumed by unit or binary clauses
  and ensures clauses cannot be strengthened by unit resolution
  or by resolution with one binary clause.
  To this end, it maintains the full list of clauses indexed by literals. */

typedef struct TSvar_ext
{
  unsigned occur_n[2];
  unsigned occur_size[2];
  Tclause * occur[2];
} TSvar_ext;
  
TSvar_ext * SAT_stack_var_ext = NULL;    /**< array of vars extended info */
unsigned    SAT_stack_var_ext_size = 0;  /**< alloc size of prev. array */

/*--------------------------------------------------------------*/

/**
   \author Pascal Fontaine
   \brief add extended information for all variables */
static inline void
SAT_var_ext_expand(void)
{
  unsigned i = SAT_stack_var_ext_size;
  STACK_RESIZE_EXP(SAT_stack_var_ext, SAT_stack_var_n + 1,
		   SAT_stack_var_ext_size, sizeof(TSvar_ext));
  for (; i < SAT_stack_var_ext_size; i++)
    {
      SAT_stack_var_ext[i].occur_n[VAL_FALSE] = 0;
      SAT_stack_var_ext[i].occur_n[VAL_TRUE] = 0;
      SAT_stack_var_ext[i].occur_size[VAL_FALSE] = 0;
      SAT_stack_var_ext[i].occur_size[VAL_TRUE] = 0;
      SAT_stack_var_ext[i].occur[VAL_FALSE] = NULL;
      SAT_stack_var_ext[i].occur[VAL_TRUE] = NULL;
    }
}

/*--------------------------------------------------------------*/

/**
   \author Pascal Fontaine
   \brief free extended information variables
   \param var the variable */
static inline void
SAT_var_ext_free(Tvar var)
{
  free(SAT_stack_var_ext[var].occur[VAL_FALSE]);
  free(SAT_stack_var_ext[var].occur[VAL_TRUE]);
  SAT_stack_var_ext[var].occur_n[VAL_FALSE] = 0;
  SAT_stack_var_ext[var].occur_n[VAL_TRUE] = 0;
  SAT_stack_var_ext[var].occur_size[VAL_FALSE] = 0;
  SAT_stack_var_ext[var].occur_size[VAL_TRUE] = 0;
}

/*--------------------------------------------------------------*/

/**
   \author Pascal Fontaine
   \brief adds a clause to the watched clauses of variable, with given polarity
   \param var the variable id
   \param clause the clause
   \param pol the polarity */
static inline void
var_ext_occur(Tvar var, Tclause clause, unsigned pol)
{
  TSvar_ext * Pvar = &SAT_stack_var_ext[var];
  Pvar->occur_n[pol]++;
  STACK_RESIZE_LIN(Pvar->occur[pol], Pvar->occur_n[pol],
		   Pvar->occur_size[pol], sizeof(Tclause));
  Pvar->occur[pol][Pvar->occur_n[pol] - 1] = clause;
}

/*--------------------------------------------------------------*/

Tclause * light_purge_clauses = NULL;     /**< array of small clauses */
unsigned  light_purge_clauses_n = 0;      /**< number of small clauses */
unsigned  light_purge_clauses_size = 0;   /**< alloc size of prev. array */

static inline void
light_purge_push(Tclause clause)
{
  light_purge_clauses_n++;
  STACK_RESIZE_EXP(light_purge_clauses, light_purge_clauses_n,
		   light_purge_clauses_size, sizeof(Tclause));
  light_purge_clauses[light_purge_clauses_n - 1] = clause;
}

/*--------------------------------------------------------------*/

/** \brief add clauses to internal datastructure,
    compute strengthened clauses, delete subsumed,
    remove deleted clauses from datastructure */
static inline void
light_purge_check(void)
{
  unsigned i, j;
  TSclause * PSclause;
  assert(SAT_level == ROOT_LEVEL);
  SAT_var_ext_expand();
  /* Add all clauses */
  for (i = 0; i < light_purge_clauses_n; i++)
    {
      PSclause = stack_clause + light_purge_clauses[i];
      for (j = 0; j < PSclause->n; j++)
	var_ext_occur(SAT_lit_var(PSclause->lit[j]), light_purge_clauses[i],
		      SAT_lit_pol(PSclause->lit[j]));
    }
  /* First check unit clauses */
  for (i = 0; i < light_purge_clauses_n; i++)
    if ((stack_clause + light_purge_clauses[i])->n == 1)
      {
	Tlit lit = (stack_clause + light_purge_clauses[i])->lit[0];
	TSvar_ext * Pvar  = &SAT_stack_var_ext[SAT_lit_var(lit)];
	unsigned pol = SAT_lit_pol(lit);
	/* Eliminate subsumed */
	for (j = 0; j < Pvar->occur_n[pol]; j++)
	  if (Pvar->occur[pol][j] != light_purge_clauses[i])
	    clause_lazy_delete(Pvar->occur[pol][j]);
#if 0
	/* Strengthen */
	for (j = 0; j < Pvar->occur_n[!pol]; j++)
	  if (!(PSclause = stack_clause + Pvar->occur[pol][j])->deleted)
	    {
	      /* Resolve Pvar->occur[!pol][j] with light_purge_clauses[i] */
	      unsigned k, n = PSclause->n;
	      Tlit * Plit = NULL;
	      Tclause clause;
	      assert (n > 1); /* Complementary unit clauses should not occur */
	      MY_MALLOC(Plit, (n - 1) * sizeof(Tlit));
	      for (k = 0; k < n ; k++)
		{
		  Plit[k] = PSclause->lit[k];
		  if (Plit[k] == SAT_lit_neg(lit))
		    break;
		}
	      assert (k < n);
	      for (k++; k < n; k++)
		Plit[k - 1] = PSclause->lit[k];
	      n--;
	      clause = clause_new(n, Plit, 1, 1, 0);
#ifdef PROOF
	      if (SAT_proof)
		{
		  proof_begin(Pvar->occur[pol][j]);
		  proof_resolve(lit, light_purge_clauses[i]);
		  proof_end(clause);
		}
#endif
	      clause_lazy_delete(Pvar->occur[pol][j]);
	      /* Add the clause */
	      light_purge_push(clause);
	      for (k = 0; k < n; j++)
		var_ext_occur(SAT_lit_var(Plit[j]), clause,
			      SAT_lit_pol(Plit[j]));
	    }
#endif
      }
  light_purge_clauses_n = 0;
  /* TODO make sure OK when removing clauses because we delete
     original clauses here */
}

/*--------------------------------------------------------------*/

/** \brief remove all deleted clauses */
static inline void
light_purge_clean(void)
{
  
}

/*--------------------------------------------------------------*/

/** \brief remove all deleted clauses from occurence lists */
static inline
batch_rm_clauses_2(void)
{
  unsigned i, j;
  TSvar_ext * PSvar = SAT_stack_var_ext + 1;
  for (i = 1; i <= SAT_stack_var_n; ++PSvar, ++i)
    {
      Tclause * occur1, * occur2;
      assert(PSvar == SAT_stack_var_ext + i);
      occur1 = occur2 = PSvar->occur[VAL_FALSE];
      for (j = PSvar->occur_n[VAL_FALSE]; j-- > 0; ++occur1)
	if (!stack_clause[*occur1].deleted)
	  *(occur2++) = *occur1;
      PSvar->occur_n[VAL_FALSE] = (unsigned) (occur2 - PSvar->occur[VAL_FALSE]);
      occur1 = occur2 = PSvar->occur[VAL_TRUE];
      for (j = PSvar->occur_n[VAL_TRUE]; j-- > 0; ++occur1)
	if (!stack_clause[*occur1].deleted)
	  *(occur2++) = *occur1;
      PSvar->occur_n[VAL_TRUE] = (unsigned) (occur2 - PSvar->occur[VAL_TRUE]);
    }
}

#endif

/*
  --------------------------------------------------------------
  Proof checker
  --------------------------------------------------------------
*/
#ifdef PROOF

static unsigned proof_stack_size = 0;
#ifdef INSIDE_VERIT
void proof_SAT_learnt(SAT_Tclause clause);
void proof_SAT_set_id(SAT_Tclause clause_id);
#define proof_stack_n SAT_proof_stack_n
#define proof_stack_lit SAT_proof_stack_lit
#define proof_stack_clause SAT_proof_stack_clause
unsigned proof_stack_n = 0;
Tlit *   proof_stack_lit = NULL;
Tclause * proof_stack_clause = NULL;
#else
static unsigned proof_stack_n = 0;
static Tlit *   proof_stack_lit = NULL;
static Tclause * proof_stack_clause = NULL;
#endif /* INSIDE_VERIT */

static unsigned proof_stack_size_2 = 0;
static unsigned proof_stack_n_2 = 0;
static Tlit *   proof_stack_lit_2 = NULL;
static Tclause * proof_stack_clause_2= NULL;

/* State machine for variables.  States: */
#define STATE_INIT 0
#define STATE_POSITIVE 1
#define STATE_NEGATIVE 2
#define STATE_POSITIVE_RESOLVING 3
#define STATE_NEGATIVE_RESOLVING 4
#define STATE_RESOLVED 5
#define STATE_FAILED 6

/* Transitions
   0 --  p --> 1
   0 -- -p --> 2
   1 --  p --> 1
   1 -- -p --> 6
   2 --  p --> 6
   2 -- -p --> 2
   3 --  p --> 6
   3 -- -p --> 5
   4 --  p --> 5
   4 -- -p --> 6
   5 --  * --> 6
   6 --  * --> 6
   0 --  R --> 6
   1 --  R --> 3
   2 --  R --> 4
   3 --  R --> 6
   4 --  R --> 6
   5 --  R --> 6
   Accepting states 1-2-5
   Two passes :
   - first pass to compute final state for every variable
   - second pass to collect all variables in state 1-2.
     Any variable in state 0-3-4-7 generates an error */

/*--------------------------------------------------------------*/

static void
proof_begin(Tclause clause)
{
  if (proof_stack_n != 0)
    {
      /* PF second level proof:
	 may come from simplification in added clause while explaining hint
	 in conflict */
      assert(proof_stack_n_2 == 0);
      SWAP(unsigned, proof_stack_n, proof_stack_n_2);
      SWAP(unsigned, proof_stack_size, proof_stack_size_2);
      SWAP(Tlit *, proof_stack_lit, proof_stack_lit_2);
      SWAP(Tclause *, proof_stack_clause, proof_stack_clause_2);
    }
  proof_stack_n = 1;
  if (!proof_stack_size)
    {
      MY_MALLOC(proof_stack_lit, sizeof(Tlit));
      MY_MALLOC(proof_stack_clause, sizeof(Tclause));
      proof_stack_size = 1;
    }
  proof_stack_clause[0] = clause;
}

/*--------------------------------------------------------------*/

static void
proof_resolve(Tlit lit, Tclause clause)
{
  assert(proof_stack_size > 0);
  assert(clause != CLAUSE_LAZY);
  proof_stack_n++;
  while (proof_stack_size < proof_stack_n)
    {
      proof_stack_size *= 2;
      MY_REALLOC(proof_stack_lit, proof_stack_size * sizeof(Tlit));
      MY_REALLOC(proof_stack_clause, proof_stack_size * sizeof(Tclause));
    }
  proof_stack_lit[proof_stack_n - 2] = lit;
  proof_stack_clause[proof_stack_n - 1] = clause;
}

/*--------------------------------------------------------------*/

static inline int
proof_update_lit(Tlit lit)
{
  TSvar * PSvar = SAT_stack_var + SAT_lit_var(lit);
  switch (PSvar->misc)
    {
    case STATE_INIT :
      PSvar->misc = SAT_lit_pol(lit)?STATE_POSITIVE:STATE_NEGATIVE;
      return 1;
    case STATE_POSITIVE :
      if (!SAT_lit_pol(lit))
	{
	  PSvar->misc = STATE_FAILED;
	  my_error("proof error\n");
	}
      return 0;
    case STATE_NEGATIVE :
      if (SAT_lit_pol(lit))
	{
	  PSvar->misc = STATE_FAILED;
	  my_error("proof error\n");
	}
      return 0;
    case STATE_POSITIVE_RESOLVING :
      if (SAT_lit_pol(lit))
	{
	  PSvar->misc = STATE_FAILED;
	  my_error("proof error\n");
	}
      PSvar->misc = STATE_RESOLVED;
      return -1;
    case STATE_NEGATIVE_RESOLVING :
      if (!SAT_lit_pol(lit))
	{
	  PSvar->misc = STATE_FAILED;
	  my_error("proof error\n");
	}
      PSvar->misc = STATE_RESOLVED;
      return -1;
    case STATE_RESOLVED :
    case STATE_FAILED :
    default :
      my_error("proof error\n");
      PSvar->misc = STATE_FAILED;
      return 0;
    }
}

/*--------------------------------------------------------------*/

static inline void
proof_resolve_lit(Tlit lit)
{
  TSvar * PSvar = SAT_stack_var + SAT_lit_var(lit);
  switch (PSvar->misc)
    {
    case STATE_INIT :
      PSvar->misc = STATE_FAILED;
      my_error("proof error\n");
      break;
    case STATE_POSITIVE :
      PSvar->misc = STATE_POSITIVE_RESOLVING;
      break;
    case STATE_NEGATIVE :
      PSvar->misc = STATE_NEGATIVE_RESOLVING;
      break;
    case STATE_POSITIVE_RESOLVING :
    case STATE_NEGATIVE_RESOLVING :
    case STATE_RESOLVED :
    case STATE_FAILED :
      my_error("proof error\n");
      PSvar->misc = STATE_FAILED;
    }
}

/*--------------------------------------------------------------*/

static void
proof_end(Tclause clause)
{
  unsigned i, j, count = 0;
  TSclause * PSclause;
#ifdef PROOF_PRINT
  proof_print(clause);
#endif
  /* first traversal */
  for (i = 0; i + 1 < proof_stack_n; ++i)
    {
      PSclause = stack_clause + proof_stack_clause[i];
      for (j = 0; j < PSclause->n; j++)
	count = (unsigned) ((int) count + proof_update_lit(PSclause->lit[j]));
      proof_resolve_lit(proof_stack_lit[i]);
    }
  PSclause = stack_clause + proof_stack_clause[i];
  for (j = 0; j < PSclause->n; j++)
    count = (unsigned) ((int) count + proof_update_lit(PSclause->lit[j]));
  /* second traversal */
  PSclause = stack_clause + clause;
  if (PSclause->n != count)
    my_error("proof error\n");
  for (j = 0; j < PSclause->n; j++)
    switch (SAT_stack_var[SAT_lit_var(PSclause->lit[j])].misc)
      {
      case STATE_POSITIVE :
      case STATE_NEGATIVE :
	SAT_stack_var[SAT_lit_var(PSclause->lit[j])].misc = 0;
	break;
      default :
	my_error("proof error\n");
      }
  for (i = 0; i < proof_stack_n; ++i)
    {
      PSclause = stack_clause + proof_stack_clause[i];
      for (j = 0; j < PSclause->n; j++)
	switch (SAT_stack_var[SAT_lit_var(PSclause->lit[j])].misc)
	  {
	  case STATE_INIT :
	  case STATE_RESOLVED :
	    SAT_stack_var[SAT_lit_var(PSclause->lit[j])].misc = 0;
	    break;
	  case STATE_POSITIVE :
	  case STATE_NEGATIVE :
	  case STATE_POSITIVE_RESOLVING :
	  case STATE_NEGATIVE_RESOLVING :
	  case STATE_FAILED :
	    SAT_stack_var[SAT_lit_var(PSclause->lit[j])].misc = 0;
	    my_error("proof error\n");
	  }
    }
#ifdef INSIDE_VERIT
  proof_SAT_learnt(clause);
#endif
  proof_stack_n = 0;
  if (proof_stack_n_2 != 0)
    {
      /* PF restore second level proof */
      SWAP(unsigned, proof_stack_n, proof_stack_n_2);
      SWAP(unsigned, proof_stack_size, proof_stack_size_2);
      SWAP(Tlit *, proof_stack_lit, proof_stack_lit_2);
      SWAP(Tclause *, proof_stack_clause, proof_stack_clause_2);
    }
}

/*--------------------------------------------------------------*/

#ifdef PROOF_PRINT

static void
proof_print(Tclause clause)
{
  unsigned i;
  for (i = 0; i + 1 < proof_stack_n; ++i)
    {
      printf("%u", proof_stack_clause[i]);
#ifdef PROOF_PRINT_CLAUSES
      printf(" ("); clause_print(proof_stack_clause[i]); printf(")");
#endif
      printf(" [%d] ", SAT_lit_var(proof_stack_lit[i]));
    }
  printf("%u", proof_stack_clause[i]);
#ifdef PROOF_PRINT_CLAUSES
  printf(" ("); clause_print(proof_stack_clause[i]); printf(")");
#endif
  printf(" --> %d", clause);
#ifdef PROOF_PRINT_CLAUSES
  printf(" ("); clause_print(clause); printf(")");
#endif
  printf("\n");
}

#endif /* PROOF_PRINT */
#endif /* PROOF */

/*
  --------------------------------------------------------------
  Core
  --------------------------------------------------------------
*/

/**
   \author Pascal Fontaine
   \brief computes the numbers of the luby series
   \param i the index of the luby number to compute
   \return the luby number of index i
   \remark this is fairly fast.  Computes all luby number from 0 to
   400000 in a fraction of a second.  No need to store anything. */
static inline unsigned
luby(unsigned i)
{
  unsigned e;
  i += 2;
  e = 1;
  /* computes the nearest power of two <= i */
  while (e <= i) e <<= 1;
  e >>= 1;
  while (e != i)
    {
      i -= e;
      ++i;
      while (e > i) e >>= 1;
    }
  return e >> 1;
}

/*--------------------------------------------------------------*/

static inline unsigned
restart_suite(unsigned i)
{
  return luby(i) << RESTART_MIN_INTERVAL;
}

/*--------------------------------------------------------------*/

/**
   \author Pascal Fontaine
   \brief get the next decision (1st part)
   \return the literal that is decided
   \remark this can be called any number of time, and will return the
   same result again, unless decision_pop is called */
static inline Tlit
decision_get(void)
{
  Tvar next;
  if (heap_var_empty())
    return LIT_UNDEF;
#ifdef HINT_AS_DECISION
  while (hint_p < hint_n)
    {
      Tlit lit = hints[hint_p++];
      if (SAT_lit_value(lit) == VAL_UNDEF)
	return lit;
    }
  hint_p = hint_n = 0;
#endif
#ifdef RANDOMIZE_DECISION
  if (fastrand(RANDOMIZE_FREQ << 2) < 4)
    {
      next = heap_var[fastrand(heap_var_n)];
      if (SAT_var_value(next) == VAL_UNDEF && SAT_var_decision(next))
	return SAT_lit(next, fastrand(2));
    }
#endif
  /* IMPROVE here optionally randomize variables a bit */
  while (1)
    {
      next = heap_var_get_min();
      heap_var_remove_min();
      if (SAT_var_value(next) == VAL_UNDEF && SAT_var_decision(next))
	/* IMPROVE here optionally randomize polarity a bit */
	return SAT_lit(next, SAT_var_phase_cache(next));
      if (heap_var_empty())
	return LIT_UNDEF;
    }
  return LIT_UNDEF;
}

/*
  --------------------------------------------------------------
  Bclause light
  --------------------------------------------------------------
*/

#ifdef BCLAUSE_LIGHT

typedef struct Tprop
{
  Tlit lit;
  Tclause clause;
} Tprop;

typedef struct Tbclause_lit
{
  unsigned n;         /**< number of consequences */
  unsigned size;      /**< place allocated for direct consequences */
  Tprop * prop;       /**< consequences */
} Tbclause_lit;

Tbclause_lit * bclause_lit = NULL;

/*--------------------------------------------------------------*/

/**
   \author Pascal Fontaine
   \brief adds a binary clause
   \param clause the clause
   \param lit1 the first literal
   \param lit2 the second literal
   \return unsatisfiable if this leads to an unsatisfiable set of
   binary clauses */
static inline void
bclause_add(Tclause clause, Tlit lit1, Tlit lit2)
{
  /* All literals implying not lit1 will imply more,
     consequence lists should be updated */
  /* printf("bclause_add %d %d (%d)\n", lit1, lit2, clause); */
  lit1 = SAT_lit_neg(lit1);
  lit2 = SAT_lit_neg(lit2);
  bclause_lit[lit1].n++;
  STACK_RESIZE_EXP(bclause_lit[lit1].prop, bclause_lit[lit1].n,
		   bclause_lit[lit1].size, sizeof(Tprop));
  bclause_lit[lit1].prop[bclause_lit[lit1].n - 1].lit = SAT_lit_neg(lit2);
  bclause_lit[lit1].prop[bclause_lit[lit1].n - 1].clause = clause;
  bclause_lit[lit2].n++;
  STACK_RESIZE_EXP(bclause_lit[lit2].prop, bclause_lit[lit2].n,
		   bclause_lit[lit2].size, sizeof(Tprop));
  bclause_lit[lit2].prop[bclause_lit[lit2].n - 1].lit = SAT_lit_neg(lit1);
  bclause_lit[lit2].prop[bclause_lit[lit2].n - 1].clause = clause;
}

/*--------------------------------------------------------------*/

/**
   \author Pascal Fontaine
   \brief compute the consequences of a literal
   \pre consequences of all direct consequences should be computed
   \pre misc bit of all direct consequences should be 0
   \pre misc bit of the consequences of all direct consequences should be 0
   \post misc bit are reset to 0
   \post consequences of lit are computed */
static inline Tclause
bclause_propagate(void)
{
  unsigned i, j;
  for (i = stack_lit_to_propagate; i < stack_lit_n; i++)
    {
      Tlit lit = stack_lit[i];
      for (j = 0; j < bclause_lit[lit].n; j++)
	switch (SAT_lit_value(bclause_lit[lit].prop[j].lit))
	  {
	  case VAL_TRUE: continue;
	  case VAL_FALSE:
#ifdef PROOF
	    /* Propagation at ROOT_LEVEL: compute explicit unit clause */
	    if (SAT_level == ROOT_LEVEL && SAT_proof)
	      {
		Tclause clause = bclause_lit[lit].prop[j].clause;
		proof_begin(clause);
		proof_resolve(stack_clause[clause].lit[0],
			      SAT_lit_reason(stack_clause[clause].lit[0]));
		proof_resolve(stack_clause[clause].lit[1],
			      SAT_lit_reason(stack_clause[clause].lit[1]));
		clause = clause_new(0, NULL, 0, 0, 0);
		proof_end(clause);
		return clause;
	      }
#endif
	    return bclause_lit[lit].prop[j].clause;
	  default:
#ifdef PROOF
	    /* Propagation at ROOT_LEVEL: compute explicit unit clause */
	    if (SAT_level == ROOT_LEVEL && SAT_proof)
	      {
		Tclause clause = bclause_lit[lit].prop[j].clause;
		Tlit * Plit;
		proof_begin(clause);
		if (stack_clause[clause].lit[0] == bclause_lit[lit].prop[j].lit)
		  proof_resolve(stack_clause[clause].lit[1],
				SAT_lit_reason(stack_clause[clause].lit[1]));
		else
		  proof_resolve(stack_clause[clause].lit[0],
				SAT_lit_reason(stack_clause[clause].lit[0]));
		MY_MALLOC(Plit, sizeof(Tlit));
		Plit[0] = bclause_lit[lit].prop[j].lit;
		clause = clause_new(1, Plit, 0, 0, 0);
		stack_lit_add(bclause_lit[lit].prop[j].lit, clause);
		proof_end(clause);
		continue;
	      }
#endif
	    stack_lit_add(bclause_lit[lit].prop[j].lit,
			  bclause_lit[lit].prop[j].clause);
	  }
    }
  return CLAUSE_UNDEF;
}

/*--------------------------------------------------------------*/

static void
bclause_resize(unsigned size_old, unsigned size_new)
{
  unsigned i;
  MY_REALLOC(bclause_lit, size_new * sizeof(Tbclause_lit));
  for (i = size_old; i < size_new; i++)
    {
      bclause_lit[i].n = 0;
      bclause_lit[i].size = 0;
      bclause_lit[i].prop = NULL;
    }
}

#endif /* BCLAUSE_LIGHT */

/*
  --------------------------------------------------------------
  Bclause
  --------------------------------------------------------------
*/

#ifdef BCLAUSE

typedef struct Tprop
{
  Tlit lit;
  Tclause clause;
} Tprop;

typedef struct Tbclause_lit
{
  unsigned n;         /**< number of consequences */
  unsigned direct:30; /**< number of direct consequences */
  unsigned updated:1; /**< consequences are up-to-date */
  unsigned misc:1;    /**< misc usage bit */
  unsigned size:30;   /**< place allocated for direct consequences */
  unsigned cycle:1;   /**< set if a cycle is detected from lit
			 Prevents memoizing of children, just DFS */
  unsigned misc2:1;   /**< misc usage bit */
  Tprop * prop;       /**< consequences */
} Tbclause_lit;

Tbclause_lit * bclause_lit = NULL;

#ifdef CYCLE_DETECTION

typedef struct Tcycle
{
  Tlit reach;
  Tlit start;
} Tcycle;

Tcycle * cycle = NULL;
unsigned cycle_size = 0;
unsigned cycle_n = 0;

#endif

/*--------------------------------------------------------------*/

/**
   \author Pascal Fontaine
   \brief invalidate all consequence lists of lit and parents
   \param lit the literal */
/*
  Assume l1 v l2 is added
  then l2 will be added to consequences of -l1
    All parents of -l1 (that have -l1 as consequence) should be updated
    To obtain the parents of -l1, one takes the consequences of l1
    If l0 is a consequence of l1, then -l0 is a parent of -l1
  and  l1 will be added to consequences of -l2 */
static inline void
bclause_invalidate_parents(Tlit lit)
{
  unsigned i, j;
#if defined(DEBUG_SAT) && DEBUG_LEVEL > 1
  for (i = 0; i < (SAT_stack_var_n + 1) << 1; i++)
    if (!bclause_lit[i].updated)
      assert(bclause_lit[i].n == bclause_lit[i].direct);
#endif
  /* misc_stack will contain the negation of all parents */
  STACK_RESIZE_EXP(misc_stack, 4, misc_stack_size, sizeof(Tlit));
  misc_stack_n = 1;
  misc_stack[0] = lit;
  bclause_lit[lit].updated = 0;
  bclause_lit[lit].n = bclause_lit[lit].direct;
  /* collect all parents */
  for (i = 0; i < misc_stack_n; i++)
    {
      Tbclause_lit * Pbclause_lit = bclause_lit + SAT_lit_neg(misc_stack[i]);
      STACK_RESIZE_EXP(misc_stack, misc_stack_n + Pbclause_lit->n,
		       misc_stack_size, sizeof(Tlit));
      for (j = 0; j < Pbclause_lit->direct; j++)
	{
	  Tlit lit = SAT_lit_neg(Pbclause_lit->prop[j].lit);
	  if (!bclause_lit[lit].updated && !bclause_lit[lit].cycle)
	    continue;
	  if (bclause_lit[lit].misc)
	    continue;
	  bclause_lit[lit].updated = 0;
	  bclause_lit[lit].misc = 1;
	  bclause_lit[lit].n = bclause_lit[lit].direct;
	  misc_stack[misc_stack_n++] = lit;
	}
    }
  for (i = 0; i < misc_stack_n; i++)
    bclause_lit[misc_stack[i]].misc = 0;
  misc_stack_n = 0;
#if defined(DEBUG_SAT) && DEBUG_LEVEL > 1
  for (i = 0; i < (SAT_stack_var_n + 1) << 1; i++)
    if (!bclause_lit[i].updated)
      assert(bclause_lit[i].n == bclause_lit[i].direct);
#endif
}

/*--------------------------------------------------------------*/

#ifdef CYCLE_DETECTION

static void
bclause_cycle(Tlit lit1, Tlit lit2)
{
  cylce_n++;
  STACK_RESIZE_EXP(cycle, cycle_n, cycle_size, sizeof(Tcycle));
  cycle[cycle_n - 1].reach = lit1;
  cycle[cycle_n - 1].start = lit2;
}

#endif

/*--------------------------------------------------------------*/

/**
   \author Pascal Fontaine
   \brief adds a binary clause
   \param clause the clause
   \param lit1 the first literal
   \param lit2 the second literal
   \return unsatisfiable if this leads to an unsatisfiable set of
   binary clauses */
static inline void
bclause_add(Tclause clause, Tlit lit1, Tlit lit2)
{
  /* All literals implying not lit1 will imply more,
     consequence lists should be updated */
  /* printf("bclause_add %d %d (%d)\n", lit1, lit2, clause); */
  lit1 = SAT_lit_neg(lit1);
  lit2 = SAT_lit_neg(lit2);
  bclause_invalidate_parents(lit1);
  bclause_invalidate_parents(lit2);
  bclause_lit[lit1].n++;
  STACK_RESIZE_EXP(bclause_lit[lit1].prop, bclause_lit[lit1].n,
		   bclause_lit[lit1].size, sizeof(Tprop));
  bclause_lit[lit1].prop[bclause_lit[lit1].direct].lit = SAT_lit_neg(lit2);
  bclause_lit[lit1].prop[bclause_lit[lit1].direct].clause = clause;
  bclause_lit[lit1].direct++;
  bclause_lit[lit2].n++;
  STACK_RESIZE_EXP(bclause_lit[lit2].prop, bclause_lit[lit2].n,
		   bclause_lit[lit2].size, sizeof(Tprop));
  bclause_lit[lit2].prop[bclause_lit[lit2].direct].lit = SAT_lit_neg(lit1);
  bclause_lit[lit2].prop[bclause_lit[lit2].direct].clause = clause;
  bclause_lit[lit2].direct++;
  return;
}

/*--------------------------------------------------------------*/

static inline void
bclause_cycle_closure()
{
  unsigned i, j, k;
  Tlit lit = misc_stack[0];
  /* first mark all nodes in path as cycling (not optimal) and clean */
  for (k = 1; k < misc_stack_n; k++)
    if (bclause_lit[misc_stack[k]].misc)
      {
	bclause_lit[misc_stack[k]].cycle = 1;
	bclause_lit[misc_stack[k]].misc = 0;
      }
  misc_stack_n = 1;
  bclause_lit[lit].cycle = 1;
  bclause_lit[lit].misc = 1;
  assert(!bclause_lit[lit].updated);
  assert(bclause_lit[lit].n == bclause_lit[lit].direct);
  /* first add direct consequences on stack but not on list (already there) */
  STACK_RESIZE_EXP(misc_stack, bclause_lit[lit].direct + 1,
		   misc_stack_size, sizeof(Tlit));
  for (j = 0; j < bclause_lit[lit].n; j++)
    {
      Tprop prop = bclause_lit[lit].prop[j];
      if (!bclause_lit[prop.lit].misc)
	{
	  misc_stack[misc_stack_n++] = prop.lit;
	  bclause_lit[prop.lit].misc = 1;
	}
    }
  /* misc_stack will be used to contain the bfs stack */
  /* collect all children */
  for (i = 0; i < misc_stack_n; i++)
    {
      STACK_RESIZE_EXP(bclause_lit[lit].prop,
		       bclause_lit[lit].n +
		       bclause_lit[misc_stack[i]].n,
		       bclause_lit[lit].size, sizeof(Tprop));
      if (!bclause_lit[misc_stack[i]].updated)
	{
	  STACK_RESIZE_EXP(misc_stack,
			   misc_stack_n + bclause_lit[misc_stack[i]].n,
			   misc_stack_size, sizeof(Tlit));
	  for (j = 0; j < bclause_lit[misc_stack[i]].n; j++)
	    {
	      Tprop prop = bclause_lit[misc_stack[i]].prop[j];
	      if (!bclause_lit[prop.lit].misc)
		{
		  bclause_lit[prop.lit].misc = 1;
		  bclause_lit[lit].prop[bclause_lit[lit].n++] = prop;
		  misc_stack[misc_stack_n++] = prop.lit;
		}
	    }
	}
      else
	for (j = 0; j < bclause_lit[misc_stack[i]].n; j++)
	  {
	    Tprop prop = bclause_lit[misc_stack[i]].prop[j];
	    if (!bclause_lit[prop.lit].misc)
	      {
		bclause_lit[prop.lit].misc = 1;
		bclause_lit[lit].prop[bclause_lit[lit].n++] = prop;
	      }
	  }
    }
  bclause_lit[lit].misc = 0;
  for (j = 0; j < bclause_lit[lit].n; j++)
    bclause_lit[bclause_lit[lit].prop[j].lit].misc = 0;
  bclause_lit[lit].updated = 1;
#if defined(DEBUG_SAT) && DEBUG_LEVEL > 1
  for (i = 0; i < misc_stack_n; i++)
    assert(!bclause_lit[misc_stack[i]].misc);
  for (i = 0; i < bclause_lit[lit].n; i++)
    {
      unsigned j;
      Tlit lit2 = stack_clause[bclause_lit[lit].prop[i].clause].lit[0];
      if (lit2 == SAT_lit_neg(lit))
	{
	  assert(stack_clause[bclause_lit[lit].prop[i].clause].lit[1] ==
		 bclause_lit[lit].prop[i].lit);
	  continue;
	}
      lit2 = stack_clause[bclause_lit[lit].prop[i].clause].lit[1];
      if (lit2 == SAT_lit_neg(lit))
	{
	  assert(stack_clause[bclause_lit[lit].prop[i].clause].lit[0] ==
		 bclause_lit[lit].prop[i].lit);
	  continue;
	}
      if (lit2 == bclause_lit[lit].prop[i].lit)
	lit2 = stack_clause[bclause_lit[lit].prop[i].clause].lit[0];
      else
	assert(stack_clause[bclause_lit[lit].prop[i].clause].lit[0] ==
	       bclause_lit[lit].prop[i].lit);
      for (j = 0; j < i; j++)
	if (bclause_lit[lit].prop[j].lit == SAT_lit_neg(lit2))
	  break;
      assert(j < i);
    }
#endif
}

/*--------------------------------------------------------------*/

/**
   \author Pascal Fontaine
   \brief compute the consequences of a literal
   \param lit the literal
   \pre consequences of all direct consequences should be computed
   \pre misc bit of all direct consequences should be 0
   \pre misc bit of the consequences of all direct consequences should be 0
   \post misc bit are reset to 0
   \post consequences of lit are computed */
static inline void
bclause_compute_closure(Tlit lit)
{
  unsigned i, j;
  assert(!bclause_lit[lit].updated);
  for (i = 0; i < bclause_lit[lit].direct; i++)
    {
      Tlit lit2 = bclause_lit[lit].prop[i].lit;
      assert(bclause_lit[lit2].updated);
      assert(!bclause_lit[lit2].misc2);
      bclause_lit[lit2].misc2 = 1;
    }
  for (i = 0; i < bclause_lit[lit].direct; i++)
    {
      Tlit lit2 = bclause_lit[lit].prop[i].lit;
      STACK_RESIZE_EXP(bclause_lit[lit].prop,
		       bclause_lit[lit].n + bclause_lit[lit2].n,
		       bclause_lit[lit].size, sizeof(Tprop));
      for (j = 0; j < bclause_lit[lit2].n; j++)
	{
	  Tprop prop = bclause_lit[lit2].prop[j];
	  if (!bclause_lit[prop.lit].misc2)
	    {
	      bclause_lit[lit].prop[bclause_lit[lit].n++] = prop;
	      bclause_lit[prop.lit].misc2 = 1;
	    }
	}
    }
  for (i = 0; i < bclause_lit[lit].n; i++)
    bclause_lit[bclause_lit[lit].prop[i].lit].misc2 = 0;
  bclause_lit[lit].updated = 1;
}

/*--------------------------------------------------------------*/

#if defined(DEBUG_SAT) && DEBUG_LEVEL > 1
static void
bclause_print(Tlit lit)
{
  unsigned i;
  printf("lit %d :", lit);
  for (i = 0; i < bclause_lit[lit].n; i++)
    printf("%d (%d: %d v %d) ", bclause_lit[lit].prop[i].lit,
	   bclause_lit[lit].prop[i].clause,
	   stack_clause[bclause_lit[lit].prop[i].clause].lit[0],
	   stack_clause[bclause_lit[lit].prop[i].clause].lit[1]);
  printf("\n");
}

/*--------------------------------------------------------------*/

static void
bclause_ext_print_aux(Tlit lit)
{
  unsigned i;
  if (bclause_lit[lit].misc2)
    return;
  bclause_lit[lit].misc2 = 1;
  printf("lit %d :", lit);
  for (i = 0; i < bclause_lit[lit].direct; i++)
    printf("%d! (%d: %d v %d) ", bclause_lit[lit].prop[i].lit,
	   bclause_lit[lit].prop[i].clause,
	   stack_clause[bclause_lit[lit].prop[i].clause].lit[0],
	   stack_clause[bclause_lit[lit].prop[i].clause].lit[1]);
  for (; i < bclause_lit[lit].n; i++)
    printf("%d (%d: %d v %d) ", bclause_lit[lit].prop[i].lit,
	   bclause_lit[lit].prop[i].clause,
	   stack_clause[bclause_lit[lit].prop[i].clause].lit[0],
	   stack_clause[bclause_lit[lit].prop[i].clause].lit[1]);
  printf("\n");
  for (i = 0; i < bclause_lit[lit].direct; i++)
    bclause_ext_print_aux(bclause_lit[lit].prop[i].lit);
}

/*--------------------------------------------------------------*/

static void
bclause_ext_print_clean(Tlit lit)
{
  unsigned i;
  if (!bclause_lit[lit].misc2)
    return;
  bclause_lit[lit].misc2 = 0;
  for (i = 0; i < bclause_lit[lit].direct; i++)
    bclause_ext_print_clean(bclause_lit[lit].prop[i].lit);
}

/*--------------------------------------------------------------*/

static void
bclause_ext_print(Tlit lit)
{
  bclause_ext_print_aux(lit);
  bclause_ext_print_clean(lit);
}

#endif

/*--------------------------------------------------------------*/

/**
   \author Pascal Fontaine
   \brief compute the consequences of a literal
   \param lit the literal
   \pre misc bit should be set to 0 for all literals */
static inline void
bclause_closure(Tlit lit)
{
  unsigned i;
  /* misc_stack will be used to contain the dfs stack */
  STACK_RESIZE_EXP(misc_stack, 4, misc_stack_size, sizeof(Tlit));
  misc_stack[0] = lit;
  misc_stack_n = 1;
  if (bclause_lit[lit].cycle)
    goto cycle;
  while (misc_stack_n)
    {
      Tlit lit2 = misc_stack[misc_stack_n - 1];
      STACK_RESIZE_EXP(misc_stack, misc_stack_n + bclause_lit[lit2].n,
		       misc_stack_size, sizeof(Tlit));
      if (bclause_lit[lit2].misc)
	{
	  /* all consequences have been examined */
	  bclause_compute_closure(lit2);
	  bclause_lit[lit2].misc = 0;
	  misc_stack_n--;
	  continue;
	}
      /* lit2 was added only if !updated
	 updated meanwhile means DAG going to same literal; discard */
      if (bclause_lit[lit2].updated)
	{
	  misc_stack_n--;
	  continue;
	}
      bclause_lit[lit2].misc = 1;
      for (i = 0; i < bclause_lit[lit2].direct; i++)
	{
	  Tlit lit3 = bclause_lit[lit2].prop[i].lit;
	  if (bclause_lit[lit3].updated)
	    continue;
	  misc_stack[misc_stack_n++] = lit3;
	  if (bclause_lit[lit3].misc || bclause_lit[lit3].cycle)
	    goto cycle;
	}
    }
#if defined(DEBUG_SAT) && DEBUG_LEVEL > 1
  assert(!bclause_lit[lit].misc);
  for (i = 0; i < bclause_lit[lit].n; i++)
    {
      unsigned j;
      Tlit lit2 = stack_clause[bclause_lit[lit].prop[i].clause].lit[0];
      assert(!bclause_lit[bclause_lit[lit].prop[i].lit].misc);
      if (lit2 == SAT_lit_neg(lit))
	{
	  assert(stack_clause[bclause_lit[lit].prop[i].clause].lit[1] ==
		 bclause_lit[lit].prop[i].lit);
	  continue;
	}
      lit2 = stack_clause[bclause_lit[lit].prop[i].clause].lit[1];
      if (lit2 == SAT_lit_neg(lit))
	{
	  assert(stack_clause[bclause_lit[lit].prop[i].clause].lit[0] ==
		 bclause_lit[lit].prop[i].lit);
	  continue;
	}
      if (lit2 == bclause_lit[lit].prop[i].lit)
	lit2 = stack_clause[bclause_lit[lit].prop[i].clause].lit[0];
      else
	assert(stack_clause[bclause_lit[lit].prop[i].clause].lit[0] ==
	       bclause_lit[lit].prop[i].lit);
      for (j = 0; j < i; j++)
	if (bclause_lit[lit].prop[j].lit == SAT_lit_neg(lit2))
	  break;
      assert(j < i);
    }
#endif
  return;
 cycle:
  /* cycle found, alternative computation starts
     A bit severe but keep it simple.  Certainly better to do */
  bclause_cycle_closure();
  return;	      
}

/*--------------------------------------------------------------*/

#if 0

static void
bclause_cycle_repair(void)
{
  /* misc_stack contains a path from lit to lit */
  unsigned i, j, k;
  Tlit lit = misc_stack[misc_stack];
  misc_stack_n--;
  assert(!bclause_lit[lit].updated && bclause_lit[lit].misc);
  /* compute the path */
  k = misc_stack_n;
  while (misc_stack[k] != lit)
    {
      assert(k > 0);
      k--;
    }
  for (i = j = k + 1; i < misc_stack_n; i++)
    if (bclause_lit[misc_stack[i]].misc)
      misc_stack[j++] = misc_stack[i];
  misc_stack_n = j;
  /* path is now in misc_stack, from index k to misc_stack_n - 1 */
  /* all literals in that trailing part are equivalent */
  for (i = k; i < misc_stack_n; i++)
    {
      assert(bclause_lit[misc_stack[i]].misc);
      assert(!bclause_lit[misc_stack[i]].updated);
      assert(bclause_lit[misc_stack[i]].direct =
	     bclause_lit[misc_stack[i]].n);
    }
  /* collect all direct consequences of those literals
     and add them as direct consequences of lit */
  for (j = 0; j < bclause_lit[lit].direct; i++)
    bclause_lit[bclause_lit[lit].prop[j].lit].misc2 = 1;
  /* for each of those literals */
  /* direct consequences of lit2
     - are collected and added to consequences of lit
     - are removed */
  for (i = k + 1; i < misc_stack_n; i++)
    {
      STACK_RESIZE_EXP(bclause_lit[lit].prop,
		       bclause_lit[lit].n + bclause_lit[misc_stack[i]].n,
		       bclause_lit[lit].size, sizeof(Tprop));
      for (j = 0; j < bclause_lit[misc_stack[i]].direct; i++)
	{
	  Tprop prop = bclause_lit[misc_stack[i]].prop[j];
	  if (!bclause_lit[prop.lit].misc2)
	    {
	      bclause_lit[lit].prop[bclause_lit[lit].direct] = prop;
	      bclause_lit[lit].direct++;
	      bclause_lit[prop.lit].misc2 = 1;
	      clause_replace_lit(prop.clause, prop.lit, lit);
	    }
	}
      bclause_lit[misc_stack[i]].direct = 0;
      bclause_lit[misc_stack[i]].n = 0;
    }
  for (j = 0; j < bclause_lit[lit].direct; i++)
    bclause_lit[bclause_lit[lit].prop[j].lit].misc2 = 0;
  /* mark lit2 as being equivalent to lit */
  /* for each lit3 that has lit2 as consequence,
     replace lit2 by lit3 */
  /* add lit2 as consequence of lit */
  /* add -lit2 as consequence of -lit */
  /* these 2 occurences should then be
     and remain the only occurences of lit2 */
  /* while propagating, or adding clause, lit should silently be
     replaced by lit2 */
  /* while propagating lit, nothing special has to be done
     while propagating lit2, propagate lit first, then do as propagating lit */
  /* make sure the top level SAT solver also has this information:
     i.e. modified clauses should match
     and there should be clauses asserting equivalence of lit and lit2 */
}

#endif

/*--------------------------------------------------------------*/

static Tclause
bclause_propagate(void)
{
  unsigned i, j, n = stack_lit_n;
  for (i = stack_lit_to_propagate; i < n; i++)
    {
      Tlit lit = stack_lit[i];
      if (bclause_lit[lit].n == 0)
	continue;
      if (!bclause_lit[lit].updated)
	bclause_closure(lit);
      for (j = 0; j < bclause_lit[lit].n; j++)
	{
	  if (SAT_lit_value_undef(bclause_lit[lit].prop[j].lit))
#ifndef PROOF
	    stack_lit_add(bclause_lit[lit].prop[j].lit,
			  bclause_lit[lit].prop[j].clause);
#else
	    {
	      /* Propagation at ROOT_LEVEL: compute explicit unit clause */
	      if (SAT_proof && SAT_level == ROOT_LEVEL)
		{
		  Tclause clause = bclause_lit[lit].prop[j].clause;
		  Tlit * Plit;
		  proof_begin(clause);
		  if (stack_clause[clause].lit[0] == bclause_lit[lit].prop[j].lit)
		    proof_resolve(stack_clause[clause].lit[1],
				  SAT_lit_reason(stack_clause[clause].lit[1]));
		  else
		    proof_resolve(stack_clause[clause].lit[0],
				  SAT_lit_reason(stack_clause[clause].lit[0]));
		  MY_MALLOC(Plit, sizeof(Tlit));
		  Plit[0] = bclause_lit[lit].prop[j].lit;
		  clause = clause_new(1, Plit, 0, 0, 0);
		  stack_lit_add(bclause_lit[lit].prop[j].lit, clause);
		  proof_end(clause);
		}
	      else
		stack_lit_add(bclause_lit[lit].prop[j].lit,
			      bclause_lit[lit].prop[j].clause);
	    }
#endif /* PROOF */
	  else if (SAT_lit_value(bclause_lit[lit].prop[j].lit) == VAL_FALSE)
#ifndef PROOF
	    return bclause_lit[lit].prop[j].clause;
#else
	    {
	      /* Propagation at ROOT_LEVEL: compute explicit unit clause */
	      if (SAT_proof && SAT_level == ROOT_LEVEL)
		{
		  Tclause clause = bclause_lit[lit].prop[j].clause;
		  proof_begin(clause);
		  proof_resolve(stack_clause[clause].lit[0],
				SAT_lit_reason(stack_clause[clause].lit[0]));
		  proof_resolve(stack_clause[clause].lit[1],
				SAT_lit_reason(stack_clause[clause].lit[1]));
		  clause = clause_new(0, NULL, 0, 0, 0);
		  proof_end(clause);
		  return clause;
		}
	      return bclause_lit[lit].prop[j].clause;
	    }
#endif
#if defined(DEBUG_SAT) && DEBUG_LEVEL > 1
	  assert(bclause_lit[lit].prop[j].lit ==
		 stack_clause[bclause_lit[lit].prop[j].clause].lit[0] ||
		 bclause_lit[lit].prop[j].lit ==
		 stack_clause[bclause_lit[lit].prop[j].clause].lit[1]);
	  assert(SAT_lit_value(stack_clause[bclause_lit[lit].prop[j].clause].lit[0]) == VAL_TRUE ||
		 SAT_lit_value(stack_clause[bclause_lit[lit].prop[j].clause].lit[1]) == VAL_TRUE);
	  assert(SAT_lit_value(stack_clause[bclause_lit[lit].prop[j].clause].lit[0]) == VAL_FALSE ||
		 SAT_lit_value(stack_clause[bclause_lit[lit].prop[j].clause].lit[1]) == VAL_FALSE);
#endif
	}
#if defined(DEBUG_SAT) && DEBUG_LEVEL > 1
      for (j = 0; j < bclause_lit[lit].n; j++)
	{
	  unsigned k;
	  for (k = 0; k < bclause_lit[bclause_lit[lit].prop[j].lit].n; k++)
	    assert(SAT_lit_value(bclause_lit[bclause_lit[lit].prop[j].lit].prop[k].lit) == VAL_TRUE);
	}
#endif
    }
  return CLAUSE_UNDEF;
}

/*--------------------------------------------------------------*/

static void
bclause_resize(unsigned size_old, unsigned size_new)
{
  unsigned i;
  MY_REALLOC(bclause_lit, size_new * sizeof(Tbclause_lit));
  for (i = size_old; i < size_new; i++)
    {
      bclause_lit[i].n = 0;
      bclause_lit[i].direct = 0;
      bclause_lit[i].updated = 0;
      bclause_lit[i].misc = 0;
      bclause_lit[i].misc2 = 0;
      bclause_lit[i].size = 0;
      bclause_lit[i].cycle = 0;
      bclause_lit[i].prop = NULL;
    }
}

#endif /* BCLAUSE */

/*
  --------------------------------------------------------------
  Propagation
  --------------------------------------------------------------
*/

/**
   \author Pascal Fontaine
   \brief propagates all implications
   \return conflicting clause if conflict, CLAUSE_UNDEF otherwise
   \remark we choose here to return after the first conflict.
   \remark literal stack is truncated after the first conflict. */
__attribute__((noinline))
static Tclause
propagate(void)
{
#if defined(BCLAUSE) || defined(BCLAUSE_LIGHT)
  unsigned stack_lit_to_bpropagate = stack_lit_to_propagate;
#endif
#ifdef BLACK_MAGIC
  /* PF two following line are a piece of black magic for efficiency */
  static long long int v, w;
  v = 0; w = 0;
#endif
#if STATS_LEVEL >= 2
  unsigned old_stack_lit_to_propagate = stack_lit_to_propagate;
  stats_counter_inc(stat_prop_call);
  if (stack_lit_to_propagate >= stack_lit_n)
    stats_counter_inc(stat_prop_call_waste);
#endif
#if PROOF
  if (SAT_proof && SAT_level == ROOT_LEVEL)
    {
      unsigned i;
      for (i = SAT_literal_stack_to_propagate; i < stack_lit_n; i++)
	if (SAT_lit_reason(stack_lit[i]) == CLAUSE_LAZY)
	  hint_explain(stack_lit[i]);
    }
#endif
  while (stack_lit_to_propagate < stack_lit_n)
    {
      Tlit lit = stack_lit[stack_lit_to_propagate];
      Tclause *i, *j, *n;
#if STATS_LEVEL >= 2
      stats_counter_inc(stat_n_prop);
      if (!watch[lit].n)
	stats_counter_inc(stat_prop_lit_call_nowatch);
#endif
#if defined(BCLAUSE) || defined(BCLAUSE_LIGHT)
      if (stack_lit_to_propagate == stack_lit_to_bpropagate)
	{
	  Tclause clause = bclause_propagate();
	  if (clause != CLAUSE_UNDEF) return clause;
	  stack_lit_to_bpropagate = stack_lit_n;
	}
#endif
      stack_lit_to_propagate++;
      lit = SAT_lit_neg(lit);
      if (!watch[lit].n)
	continue;
      i = j = watch[lit].Pclause;
      n = i + watch[lit].n;
      for (; i != n; ++i)
	{
	  TSclause * PSclause = stack_clause + *i;
	  Tlit * lits;
	  unsigned k;
	  if (SAT_lit_value_is_true(PSclause->blocker))
	    {
	      *(j++) = *i;
	      continue;
	    }
	  lits = PSclause->lit;
#if STATS_LEVEL >= 2
	  stats_counter_inc(stat_n_watched);
#endif
	  /* PF put lit in position 1 */
	  if (lits[0] == lit)
	    {
	      lits[0] = lits[1];
	      lits[1] = lit;
	    }
	  /* PF satisfied clause ? */
	  if (SAT_lit_value_is_true(lits[0]))
	    {
	      /* PF leaving the clause in the watch, but no need to find
		 another watch since this literal will remain true */
	      PSclause->blocker = lits[0];
	      *(j++) = *i;
	      continue;
	    }
	  /* PF look for a new watch */
	  for (k = 2; k < PSclause->n; ++k)
	    if (SAT_lit_value(lits[k]) != VAL_FALSE)
	      {
		lits[1] = lits[k];
		lits[k] = lit;
		lit_watch(lits[1], *i);
		/* delete the clause from the watch list:
		   j is not incremented */
		goto next_watch;
	      }
	  /* clause is either propagating or conflicting.
	     Keep in watch list */
	  *(j++) = *i;
#ifdef PROOF
	  /* Propagation at ROOT_LEVEL: compute explicit unit clause */
	  if (SAT_level == ROOT_LEVEL)
	    {
	      Tclause clause;
	      assert(PSclause->n > 1);
	      if (SAT_proof)
		proof_begin(*i);
	      for (k = 1; k < PSclause->n; k++)
		{
		  assert(SAT_lit_value(lits[k]) == VAL_FALSE);
		  if (SAT_proof)
		    {
		      clause = SAT_lit_reason(lits[k]);
		      assert(clause != CLAUSE_LAZY);
		      proof_resolve(lits[k], clause);
		    }
		}
	      if (SAT_lit_value(lits[0]) == VAL_FALSE)
		{
		  Tclause clause = clause_new(0, NULL, 0, 0, 0);
		  PSclause = stack_clause + *i;
		  /* conflicting clause */
		  i++; /* remove from watch */
		  for ( ; i != n; ++i, ++j)
		    *j = *i;
		  watch[lit].n -= (unsigned)(i - j);
		  if (SAT_proof)
		    {
		      Tclause clause2 = SAT_lit_reason(lits[0]);
		      assert(clause2 != CLAUSE_LAZY);
		      proof_resolve(lits[0], clause2);
		      proof_end(clause);
		    }
		  return clause;
		}
	      else
		{
		  Tclause clause;
		  Tlit * Plit2;
		  MY_MALLOC(Plit2, sizeof(Tlit));
		  Plit2[0] = lits[0];
		  clause = clause_new(1, Plit2, 0, 0, 0);
		  stack_lit_add(lits[0], clause);
		  if (SAT_proof)
		    proof_end(clause);
		  goto next_watch;
		}
	    }
#endif /* PROOF */
	  if (SAT_lit_value(lits[0]) == VAL_FALSE)
	    {
	      /* conflicting clause */
	      Tclause clause = *i;
	      i++; /* remove from watch */
	      /* First version
	      for ( ; i != n; ++i, ++j)
	       *j = *i; */
	      /* Second version: no better
	      if ((unsigned char *) i - (unsigned char *) j >
		  (unsigned char *) n - (unsigned char *) i)
		memcpy(j, i, ((unsigned char *) n - (unsigned char *) i));
	      else
		memcpy(j, (unsigned char *) n -
		       ((unsigned char *) i - (unsigned char *) j),
		       ((unsigned char *) i - (unsigned char *) j));
	      */
	      /* Third version: very slight improvement */
	      memmove(j, i, (unsigned)(((char *) n) - ((char *) i)));
	      watch[lit].n -= (unsigned) (i - j);
#if defined(DEBUG_SAT) && DEBUG_LEVEL > 1
	      for (k = 0; k < PSclause->n; ++k)
		assert(SAT_lit_value(lits[k]) == VAL_FALSE);
#endif
#if STATS_LEVEL >= 2
	      if (old_stack_lit_to_propagate + 1 == stack_lit_n)
		stats_counter_inc(stat_prop_call_noprop);
#endif
	      return clause;
	    }
	  /* propagating clause */
	  stack_lit_add(lits[0], *i);
	next_watch: ;
	}
      watch[lit].n -= (unsigned) (i - j);
    }
#if STATS_LEVEL >= 2
  if (old_stack_lit_to_propagate + 1 == stack_lit_n)
    stats_counter_inc(stat_prop_call_noprop);
#endif
  return CLAUSE_UNDEF;
}

/*
  --------------------------------------------------------------
  Bclause generation
  --------------------------------------------------------------
*/

#ifdef BCLAUSE_GENERATION
/* The idea here is to generate bclause that are not consequences of other
   bclauses so that propagation can occur "back" */

#ifdef PROOF
#error BCLAUSE_GENERATION incompatible with PROOFS
#endif

static Tstatus
bclause_generate(void)
{
  unsigned i, j, k, counter = 0;
  /* We assume root level propagation and subsumption by unit clauses */
  for (i = 2; i < 2 * (SAT_stack_var_n + 1); i++)
    if (SAT_lit_value_undef(i))
      {
	unsigned stack_lit_n_old = stack_lit_n;
	/* for each literal, propagate */
	level_push(i);
	if (propagate() != CLAUSE_UNDEF)
	  { 
	    Tclause clause;
	    Tlit * Plit;
	    /* asserting i leads to unsat, -i should be asserted */
	    level_backtrack(ROOT_LEVEL);
	    MY_MALLOC(Plit, sizeof(Tlit));
	    Plit[0] = SAT_lit_neg(i);
	    clause = clause_new(1, Plit, 0, 0, 0);
	    stack_lit_add(Plit[0], clause);
	    if (propagate() != CLAUSE_UNDEF)
	      return SAT_STATUS_UNSAT;
	    continue;
	  }
	for (j = stack_lit_n_old + 1; j < stack_lit_n; j++)
	  if (stack_clause[SAT_lit_reason(stack_lit[j])].n > 2)
	    {
	      /* deduce a binary clause */
	      /* there is a unique implication point whose consequences are
		 all but literal j */
	      Tclause clause = SAT_lit_reason(stack_lit[j]);
	      Tlit * Plit;
	      MY_MALLOC(Plit, 2 * sizeof(Tlit));
	      Plit[0] = stack_lit[j];
	      assert(stack_clause[clause].lit[0] == Plit[0]);
	      counter = 0;
	      for (k = 1; k < stack_clause[clause].n; k++)
		if (!SAT_lit_seen(stack_clause[clause].lit[k]))
		  {
		    counter++;
		    SAT_lit_set_seen(stack_clause[clause].lit[k]);
		    Plit[1] = stack_clause[clause].lit[k];
		  }
	      for (; counter > 1; )
		if (SAT_lit_seen(stack_lit[--j]))
		  {
		    Tclause clause = SAT_lit_reason(stack_lit[j]);
		    counter--;
		    SAT_lit_set_unseen(stack_lit[j]);
		    assert(stack_clause[clause].n == 2);
		    assert(stack_clause[clause].lit[0] == stack_lit[j] ||
			   stack_clause[clause].lit[1] == stack_lit[j]);
		    Plit[1] = stack_clause[clause].lit[(stack_clause[clause].lit[0] ==
							stack_lit[j])?1:0];
		    if (!SAT_lit_seen(Plit[1]))
		      {
			counter++;
			SAT_lit_set_seen(Plit[1]);
			/* Here we could set a variable.
			   If the variable is not set at the end,
			   new bclause is subsuming original clause */
		      }
		  }
	      assert(SAT_lit_seen(Plit[1]));
	      SAT_lit_set_unseen(Plit[1]);
	      level_backtrack(ROOT_LEVEL);
	      clause_new(2, Plit, 0, 0, 0);
	      /* clause will anyway not be propagating at root level
		 no need for a call to propagate */

	      /* I think that, if the UIP is not in the clause,
		 then the clause can not be strengthened by unit and binary clauses */
	      i--;
	      /* check again that literal for other clauses.
		 Maybe there are better ways, but OK for now */
	      continue;
	    }
	level_backtrack(ROOT_LEVEL);
      }
  return SAT_STATUS_SAT;
}
#endif

/*
  --------------------------------------------------------------
  Analyse
  --------------------------------------------------------------
*/

typedef unsigned Talevel;

/*--------------------------------------------------------------*/

/**
   \author Pascal Fontaine
   \brief repair clauses from conflicting state
   \param clause the conflicting or propagating clause
   \param level at which the clause is propagating
   \remark this should work for empty clauses
   \remark this should work for unit clauses
   \remark this should work for propagating clauses
   \remark this should work for clauses propagating at root level */
static void
repair_conflict(Tclause clause, Tlevel level)
{
  assert (stack_clause[clause].n != 0 || level == ROOT_LEVEL);
  assert (stack_clause[clause].n != 1 || level == ROOT_LEVEL);
  if (stack_clause[clause].n == 0)
    {
      level_backtrack(level);
      return;
    }
  if (stack_clause[clause].n == 1)
    {
      level_backtrack(level);
      stack_lit_add(stack_clause[clause].lit[0], clause);
#ifdef SAT_SYM
      if (SAT_var_orbit[SAT_lit_var(stack_clause[clause].lit[0])])
	{
	  Tvar v, v_end = SAT_lit_var(stack_clause[clause].lit[0]);
	  Tvar pol = SAT_lit_pol(stack_clause[clause].lit[0]);
	  v = v_end;
	  while ((v = SAT_var_orbit[v]) != v_end)
	    if (SAT_var_value(v) == VAL_UNDEF)
	      {
		Tlit * Plit;
		MY_MALLOC(Plit, 1 * sizeof(Tlit));
		Plit[0] = SAT_lit(v, pol);
		clause = clause_new(1, Plit, 1, 1, 0);
		stack_lit_add(stack_clause[clause].lit[0], clause);
	      }
	}
#endif
      return;
    }
  assert(SAT_lit_value(stack_clause[clause].lit[0]) == VAL_FALSE);
  assert(SAT_lit_value(stack_clause[clause].lit[1]) == VAL_FALSE);
  assert(SAT_lit_level(stack_clause[clause].lit[1]) <
	 SAT_lit_level(stack_clause[clause].lit[0]));
  assert(SAT_lit_level(stack_clause[clause].lit[1]) == level);
  level_backtrack(level);
  assert(SAT_lit_value_undef(stack_clause[clause].lit[0]));
  stack_lit_add(stack_clause[clause].lit[0], clause);
}

/*--------------------------------------------------------------*/

#ifdef CLAUSE_MIN

static inline bool
analyse_required_clause(Tlit lit, Talevel alevel)
{
  unsigned j;
  TSclause * Pclause = stack_clause + SAT_lit_reason(lit);
  assert(SAT_lit_reason(lit) != CLAUSE_UNDEF);
#ifdef HINTS
  assert(SAT_lit_reason(lit) != CLAUSE_LAZY);
#endif
  STACK_RESIZE_EXP(misc_stack, misc_stack_n + Pclause->n,
		   misc_stack_size, sizeof(Tlit));
#if defined(BCLAUSE) || defined(BCLAUSE_LIGHT)
  if (Pclause->n == 2 && Pclause->lit[0] != SAT_lit_neg(lit))
    {
      Tlit lit = Pclause->lit[0];
      Pclause->lit[0] = Pclause->lit[1];
      Pclause->lit[1] = lit;
    }
#endif
  assert(Pclause->lit[0] == SAT_lit_neg(lit));
  /* PF first add literals that are not already in conflict on misc_stack */
  for (j = 1; j < Pclause->n; j++)
    if (!SAT_lit_seen(Pclause->lit[j]))
      {
	if (SAT_lit_reason(Pclause->lit[j]) == CLAUSE_UNDEF ||
#ifdef HINTS
	    SAT_lit_reason(Pclause->lit[j]) == CLAUSE_LAZY ||
#endif
	    ((1u << (SAT_lit_level(Pclause->lit[j]) & 31u)) & alevel) == 0)
	  return true;
	SAT_lit_set_seen(Pclause->lit[j]);
	misc_stack[misc_stack_n++] = Pclause->lit[j];
      }
  return false;
}

/*--------------------------------------------------------------*/

static inline bool
analyse_required(Tlit lit, Talevel alevel)
{
  /* Use misc_stack from misc_stack_n */
  unsigned i, top = misc_stack_n;
  if (analyse_required_clause(lit, alevel))
    goto clean;
  /* PF then recursively add, for each lit on stack, the reasons */
  for (i = top; i < misc_stack_n; i++)
    if (analyse_required_clause(misc_stack[i], alevel))
      goto clean;
  return false;
 clean :
  for (i = top; i < misc_stack_n; i++)
    SAT_lit_set_unseen(misc_stack[i]);
  misc_stack_n = top;
  return true;
}

#endif /* CLAUSE_MIN */

/*--------------------------------------------------------------*/

 __attribute__((noinline))
/**
   \author Pascal Fontaine
   \brief learns the clauses from conflicting state
   \param clause the conflicting clause */
/* Remarks:
   - increasing clause activity of the conflict clause proved slightly counterproductive
   - computing level at the end was counterproductive */
static void
analyse(Tclause clause)
{
  Tlevel level;
  unsigned i, j, index, counter = 0;
  Tlit p;
  Tlit * Plit = stack_clause[clause].lit;
  unsigned n = stack_clause[clause].n;
  clause_increase_activity(clause);
  assert(SAT_level != ROOT_LEVEL);
  misc_stack_n = 1;
  STACK_RESIZE_EXP(misc_stack, misc_stack_n + 1,
		   misc_stack_size, sizeof(Tlit));
  assert(stack_lit_n > 0);
#if defined(DEBUG_SAT) && DEBUG_LEVEL > 1
  for (i = 0; i < n; ++i)
    assert(SAT_lit_value(Plit[i]) == VAL_FALSE);
  for (i = 0; i < n; i++)
    if (SAT_lit_level(Plit[i]) == SAT_level)
      break;
  assert(i < n);
#endif

#ifdef PROOF
  if (SAT_proof) proof_begin(clause);
#endif
  index = stack_lit_n - 1;
  assert (index > 0);
  do
    {
      STACK_RESIZE_EXP(misc_stack, misc_stack_n + n,
		       misc_stack_size, sizeof(Tlit));
      for (i = 0; i < n; i++)
	if (!SAT_lit_seen(Plit[i]))
	  {
	    Tvar var = SAT_lit_var(Plit[i]);
	    assert(SAT_lit_value(Plit[i]) == VAL_FALSE);
#ifndef PROOF
	    assert(SAT_var_level(var) > ROOT_LEVEL);
#endif
	    SAT_var_set_seen(var);
	    var_increase_activity(var);
	    if (SAT_var_level(var) == SAT_level)
	      counter++; /* count all literals at current level */
	    else
	      misc_stack[misc_stack_n++] = Plit[i];
	  }
      /* take next clause */
      /* by proceding by reverse order, last assigned lit are taken last
	 initially there should be at least one literal at the current
	 level (further than the conflicting literal) that is to be
	 examined, otherwise clause would have been propagating.
	 counter == 0 iff no more literals of the current level are to be
	 examined */
      while (!SAT_lit_seen(stack_lit_get(index)))
	{
	  assert(index > 0 && SAT_lit_level(stack_lit_get(index)) == SAT_level);
	  index--;
	}
      p = stack_lit_get(index);
      SAT_lit_set_unseen(p);
      counter--;
      if (counter != 0)
	{
	  clause = SAT_lit_reason(p);
#ifdef HINTS
	  if (clause == CLAUSE_LAZY)
	    {
	      assert(SAT_lit_value(p) == VAL_TRUE)
	      hint_explain(p);
	      clause = SAT_lit_reason(p);
	      assert(clause != CLAUSE_LAZY);
	    }
#endif
	  clause_increase_activity(clause);
#if defined(BCLAUSE) || defined(BCLAUSE_LIGHT)
	  /* In bclauses, literals are not reordered */
	  if (stack_clause[clause].n == 2 && stack_clause[clause].lit[0] != p)
	    {
	      Tlit tmp = stack_clause[clause].lit[0];
	      stack_clause[clause].lit[0] = stack_clause[clause].lit[1];
	      stack_clause[clause].lit[1] = tmp;
	    }
#endif
	  Plit = stack_clause[clause].lit + 1;
	  n = stack_clause[clause].n - 1;
	  assert(SAT_lit_value(*Plit) == VAL_FALSE);
#ifdef PROOF
	  if (SAT_proof) proof_resolve(p, clause);
#endif
	  assert(clause != CLAUSE_UNDEF && p == stack_clause[clause].lit[0]);
	}
    }
  while (counter != 0);
  misc_stack[0] = SAT_lit_neg(p);
  MY_MALLOC(Plit, misc_stack_n * sizeof(Tlit));
  memcpy(Plit, misc_stack, misc_stack_n * sizeof(Tlit));
  n = misc_stack_n;
#ifdef PROOF
  if (SAT_proof)
    {
      for (i = 1, j = 1; i < n; i++)
	if (SAT_lit_level(Plit[i]) != ROOT_LEVEL)
	  Plit[j++] = Plit[i];
	else
	  proof_resolve(Plit[i], SAT_lit_reason(Plit[i]));
    }
  else
    {
      Talevel alevels = 0;
      for (i = 1; i < n; i++)
	alevels |= 1u << (SAT_lit_level(Plit[i]) & 31);
      for (i = 1, j = 1; i < n; i++)
	if (SAT_lit_reason(Plit[i]) == CLAUSE_UNDEF ||
#ifdef HINTS
	    SAT_lit_reason(Plit[i]) == CLAUSE_LAZY ||
#endif /* HINTS */
	    analyse_required(Plit[i], alevels))
	  Plit[j++] = Plit[i];
    }
  n = j;
#else
#ifdef CLAUSE_MIN
  {
    Talevel alevels = 0;
    for (i = 1; i < n; i++)
      alevels |= 1u << (SAT_lit_level(Plit[i]) & 31);
    for (i = 1, j = 1; i < n; i++)
      if (SAT_lit_reason(Plit[i]) == CLAUSE_UNDEF ||
#ifdef HINTS
	  SAT_lit_reason(Plit[i]) == CLAUSE_LAZY ||
#endif /* HINTS */
	  analyse_required(Plit[i], alevels))
	Plit[j++] = Plit[i];
    n = j;
  }
#endif /* CLAUSE_MIN */
#endif /* PROOF */
  level = ROOT_LEVEL;
  if (n > 1)
    {
      for (i = 1, j = 1; i < n; i++)
	if (level < SAT_lit_level(Plit[i]))
	  level = SAT_lit_level(Plit[j = i]);
      p = Plit[1];
      Plit[1] = Plit[j];
      Plit[j] = p;
    }
  assert(!SAT_lit_seen(misc_stack[0]));
  for (i = 1; i < misc_stack_n; i++)
    SAT_lit_set_unseen(misc_stack[i]);
#if defined(DEBUG_SAT) && DEBUG_LEVEL > 1
  for (i = 0; i < n; ++i)
    assert(SAT_lit_value(Plit[i]) == VAL_FALSE);
#endif
#if STATS_LEVEL >= 1
  stats_counter_inc(stat_n_conflict);
  stats_counter_add(stat_n_conflict_lit, (int) n);
#endif
  MY_REALLOC(Plit, n * sizeof(Tlit));
  clause = clause_new(n, Plit, 1, 1, 0);
  repair_conflict(clause, level);
#ifdef PROOF
  if (SAT_proof) proof_end(clause);
#endif
}

/*
  --------------------------------------------------------------
  solving
  --------------------------------------------------------------
*/

/**
   \author Pascal Fontaine
   \brief propagates until a decision has to be done
   \return SAT_STATUS_SAT, SAT_STATUS_UNSAT, or SAT_STATUS_UNDEF */
Tstatus
SAT_propagate(void)
{
  static unsigned restart_n = 0; /**< number of restarts */
  static unsigned conflict_restart_n = 1 << RESTART_MIN_INTERVAL;
  static unsigned learnts_max = 0, learnts_n_adj_cnt = LEARNTS_ADJ_INIT;
  static double learnts_n_adj_cnt_restart = LEARNTS_ADJ_INIT;
  static bool next_purge_valid = false;
  Tclause conflict;
  /* IMPROVE REMOVE THIS TEST TO SEE IF SIGNIFICANT OVERHEAD */
  if (SAT_status != SAT_STATUS_UNDEF)
    return SAT_status;
  if (!learnts_max)
    {
      /* PF First call to SAT_propagate after adding clauses */
      learnts_max = (unsigned) (stack_clause_n * LEARNTS_FACT_INIT + 1);
#ifdef SIMP
#ifdef PROOF
      if (!SAT_proof)
#endif /* PROOF */
	{
	  if (propagate() != CLAUSE_UNDEF)
	    return (SAT_status = SAT_STATUS_UNSAT);
	  purge_valid();
#ifdef BCLAUSE_GENERATION
	  bclause_generate();
#endif /* BCLAUSE_GENERATION */
	}
#endif /* SIMP */
    }
  ON_DEBUG_SAT(check_consistency());
  ON_DEBUG_SAT(check_consistency_heap());
  while ((conflict = propagate()) != CLAUSE_UNDEF)
    {
      if (SAT_level == ROOT_LEVEL)
	{
#ifdef BACKTRACK
	  history_status_changed();
#endif
#ifdef PROOF
	  SAT_empty_clause = conflict;
#endif
	  return (SAT_status = SAT_STATUS_UNSAT);
	}
      analyse(conflict);
      if (conflict_restart_n-- == 0)
	{
#if STATS_LEVEL >= 1
	  stats_counter_inc(stat_n_restart);
#endif
	  level_backtrack(find_level_on_restart());
	  conflict_restart_n = restart_suite(++restart_n);
	  next_purge_valid = true;
	}
      if (--learnts_n_adj_cnt == 0)
	{
	  /* number of conflict between adjustment */
	  learnts_n_adj_cnt_restart *= LEARNTS_ADJ_FACT;
	  learnts_n_adj_cnt = (unsigned)learnts_n_adj_cnt_restart;
	  learnts_max = (unsigned) (learnts_max * LEARNTS_MAX_FACT);
	}
      conflict_nb++;
      var_decrease_activity();
      clause_decrease_activity();
    }
  if (SAT_level == ROOT_LEVEL && next_purge_valid)
    {
#ifdef PROOF
      if (!SAT_proof) purge_valid();
#else
      purge_valid();
#endif
      next_purge_valid = false;
    }
  if (learnts_n >= learnts_max + stack_lit_n)
    {
      purge();
    }
  ON_DEBUG_SAT(check_consistency());
  ON_DEBUG_SAT(check_consistency_heap());
  ON_DEBUG_SAT(check_consistency_propagation());
#if defined(DEBUG_SAT) && DEBUG_LEVEL > 1
  print_stack();
#endif
  return SAT_STATUS_UNDEF;
}

/*--------------------------------------------------------------*/

#if PROOF
  /* PF first sanitize all propagations at ROOT level */
void
SAT_sanitize_root_level(void)
{
  unsigned i;
  if (!SAT_proof || SAT_level != ROOT_LEVEL)
    return;
  for (i = SAT_literal_stack_to_propagate; i < stack_lit_n; i++)
    if (SAT_lit_reason(stack_lit[i]) == CLAUSE_LAZY)
      hint_explain(stack_lit[i]);
}
#endif

/*--------------------------------------------------------------*/

#ifdef HINTS
/**
   \author Pascal Fontaine
   \brief adds hint, i.e. propagated literal with lazy clause */
void
SAT_hint(Tlit lit)
{
  assert(SAT_lit_var(lit) <= SAT_stack_var_n);
  if (!SAT_lit_value_undef(lit))
    return;
#if STATS_LEVEL >= 1
  stats_counter_inc(stat_n_tp);
#endif /* STATS_LEVEL >= 1 */
#ifdef HINT_AS_DECISION
  if (hint_n + 1 > hint_size)
    {
      hint_size *= 2;
      MY_REALLOC(hints, hint_size * sizeof(lit));
    }
  hints[hint_n++] = lit;
#else
  stack_lit_add(lit, CLAUSE_LAZY);
#endif /* HINT_AS_DECISION */
  /* Clauses at root level will be explained in propagate()
     Not here because DP work may not be completed */
}
#endif /* HINTS */

/*--------------------------------------------------------------*/

/**
   \author Pascal Fontaine
   \brief adds decision */
bool
SAT_decide(void)
{
  Tlit lit;
  ON_DEBUG_SAT(check_consistency_propagation());
  lit = decision_get();
  if (!lit) /* All variables assigned */
    {
      ON_DEBUG_SAT(check_consistency_final());
      SAT_status = SAT_STATUS_SAT;
      return false;
    }
  assert(SAT_lit_value_undef(lit));
#if STATS_LEVEL >= 1
  stats_counter_inc(stat_n_decision);
#endif /* STATS_LEVEL >= 1 */
  level_push(lit);
  return true;
}

/*--------------------------------------------------------------*/

/**
   \author Pascal Fontaine
   \brief restart SAT solver */
void
SAT_restart(void)
{
  level_backtrack(ROOT_LEVEL);
#if PROOF
  if (SAT_propagate() == SAT_STATUS_UNDEF && !SAT_proof)
    purge_valid();
#else
  if (SAT_propagate() == SAT_STATUS_UNDEF)
    purge_valid();
#endif
}

/*--------------------------------------------------------------*/

/**
   \author Pascal Fontaine
   \brief adds decision
   \return SAT_STATUS_SAT, SAT_STATUS_UNSAT, or SAT_STATUS_UNDEF */
Tstatus
SAT_solve(void)
{
  while (SAT_propagate() == SAT_STATUS_UNDEF)
    SAT_decide();
  return SAT_status;
}

/*
  --------------------------------------------------------------
  Adding clauses, push, pop
  --------------------------------------------------------------
*/

/**
   \author Pascal Fontaine
   \brief adds a clause
   \param n the number of literals
   \param lit an array of n literals
   \param conflict flag indicating if it is a conflict clause
   \remark this may be called at any time
   \remark destructive for the array of literals
   \remark returns CLAUSE_UNDEF if valid clause or problem already found unsat
   \return clause id or CLAUSE_UNDEF
   \remark added clause may require different treatment

   <ul>
   \li if the clause has at least two literals not falsified (true or
   undefined), the clause would not have been propagating in the current
   decision trail. It can safely be added with those two literals as watchers

   \li if the clause has just one literal true, and the level of the true
   literal is <= than the highest level of the false literals, the clause would
   not have been propagating in the current decision trail, or propagation has
   been made through another clause.  It can safely be added with the true
   literal and the highest false literals as watchers

   \li if the clause has just one literal undefined or true, backtrack
   to the highest level of falsified literals and propagate

   \li if the clause has only falsified literals, and the highest
   level is met with only one literal, backtrack to the fore-highest
   level, and propagate

   \li if the clause has only falsified literals, and the highest
   level is met with several literals, backtrack to this level and do
   a conflict analysis
   </ul> */
static inline Tclause
SAT_clause_new_aux(unsigned n, Tlit * lit, unsigned char conflict)
{
  unsigned i, j;
  Tclause clause;
  if (SAT_status == SAT_STATUS_UNSAT)
    {
#if defined(PROOF) && defined(INSIDE_VERIT)
      if (SAT_proof)
	proof_SAT_set_id(CLAUSE_UNDEF);
#endif
      free(lit);
      return CLAUSE_UNDEF;
    }
  SAT_status = SAT_STATUS_UNDEF;
#if defined(DEBUG_SAT) && DEBUG_LEVEL > 1
  fprintf(stderr, "SAT_clause_new :");
  for (i = 0; i < n; ++i)
    fprintf(stderr, " %d", lit[i]);
  fprintf(stderr, "\n");
  check_consistency();
  check_consistency_heap();
#endif
  if (n == 0)
    {
      /* input clause is empty clause */
      clause = clause_new(n, NULL, 0, 0, 0);
#ifdef BACKTRACK
      history_status_changed();
#endif
#ifdef PROOF
      if (SAT_proof)
	{
	  SAT_empty_clause = clause;
#ifdef INSIDE_VERIT
	  proof_SAT_set_id(clause);
#endif
	}
#endif
      SAT_status = SAT_STATUS_UNSAT;
      return clause;
    }
  veriT_qsort(lit, n, sizeof(Tlit), (TFcmp) SAT_lit_compare);
  /* checking for complementary literals, true literals,
     and eliminating duplicates */
  if (SAT_lit_value(lit[0]) == VAL_TRUE && SAT_lit_level(lit[0]) == ROOT_LEVEL)
      {
	/* true literal, valid clause */
#if defined(PROOF) && defined(INSIDE_VERIT)
	if (SAT_proof)
	  proof_SAT_set_id(CLAUSE_UNDEF);
#endif
	free(lit);
	return CLAUSE_UNDEF;
      }
  for (j = 1, i = 1; i < n; ++i)
    if (lit[i] == lit[j - 1])
      continue;
    else if (SAT_lit_var(lit[i]) == SAT_lit_var(lit[j - 1]) ||
	     (SAT_lit_value(lit[i]) == VAL_TRUE &&
	      SAT_lit_level(lit[i]) == ROOT_LEVEL))
      {
	/* complementary literals or true literal, valid clause */
	free(lit);
#if defined(PROOF) && defined(INSIDE_VERIT)
	if (SAT_proof)
	  proof_SAT_set_id(CLAUSE_UNDEF);
#endif
	return CLAUSE_UNDEF;
      }
    else
      lit[j++] = lit[i];

  n = j;
  for (j = 0, i = 0; i < n; ++i)
    if (SAT_lit_value(lit[i]) != VAL_FALSE ||
	SAT_lit_level(lit[i]) != ROOT_LEVEL)
      lit[j++] = lit[i];
#ifdef PROOF
    else if (SAT_proof)
      {
	if (i == j)
	  {
	    Tlit * Plit2;
	    MY_MALLOC(Plit2, n * sizeof(Tlit));
	    memcpy(Plit2, lit, n * sizeof(Tlit));
	    clause = clause_new(n, Plit2, 0, 0, 0);
#ifdef INSIDE_VERIT
	    proof_SAT_set_id(clause);
#endif
	    proof_begin(clause);
	  }
	proof_resolve(lit[i], SAT_lit_reason(lit[i]));
      }
  if (SAT_proof && n != j)
    {
      n = j;
      MY_REALLOC(lit, n * sizeof(lit));
      veriT_qsort(lit, n, sizeof(Tlit), (TFcmp) SAT_lit_compare_level);
      clause = clause_new(n, lit, 0, 1, conflict);
      proof_end(clause);
    }
  else
    {
      n = j;
      MY_REALLOC(lit, n * sizeof(lit));
      veriT_qsort(lit, n, sizeof(Tlit), (TFcmp) SAT_lit_compare_level);
      clause = clause_new(n, lit, 0, 1, conflict);
#ifdef INSIDE_VERIT
      if (SAT_proof)
	proof_SAT_set_id(clause);
#endif
    }
#else /* PROOF */
  n = j;
  MY_REALLOC(lit, n * sizeof(lit));
  veriT_qsort(lit, n, sizeof(Tlit), (TFcmp) SAT_lit_compare_level);
  clause = clause_new(n, lit, 0, 1, conflict);
#endif /* PROOF */

  if (n == 0)
    {
      /* empty clause */
      level_backtrack(ROOT_LEVEL);
#ifdef BACKTRACK
      history_status_changed ();
#endif
#ifdef PROOF
      if (SAT_proof)
	SAT_empty_clause = clause;
#endif
      SAT_status = SAT_STATUS_UNSAT;
    }
  else if (n == 1)
    {
      level_backtrack(ROOT_LEVEL);
      /* should be propagating otherwise reduced to empty clause */
      stack_lit_add(stack_clause[clause].lit[0], clause);
    }
  else if (SAT_lit_value(lit[1]) != VAL_FALSE)
    /* first case: clause would never have been propagating
       no backtracking required
       clause can be safely added */
    {
    }
  else if (SAT_lit_value(lit[0]) == VAL_TRUE &&
	   SAT_lit_level(lit[0]) <= SAT_lit_level(lit[1]))
    /* second case: clause may never have been propagating since blocked
       by true literal
       clause can be safely added */
    {
    }
  else if (SAT_lit_value(lit[0]) != VAL_FALSE ||
	   SAT_lit_level(lit[0]) != SAT_lit_level(lit[1])) /* > */
    /*  assert (SAT_lit_value(lit[1]) == VAL_FALSE);
	assert (SAT_lit_value(lit[0]) != VAL_TRUE ||
                SAT_lit_level(lit[0]) > SAT_lit_level(lit[1])); */
    {
      /* third and fourth case */
      level_backtrack(SAT_lit_level(lit[1]));
      stack_lit_add(stack_clause[clause].lit[0], clause);
    }
  else
    /*
      assert(SAT_lit_value(lit[0]) == VAL_FALSE);
      assert(SAT_lit_value(lit[1]) == VAL_FALSE);
      assert(SAT_lit_level(lit[0]) == SAT_lit_level(lit[1])); */
    {
      /* last case: clause is conflicting */
      level_backtrack(SAT_lit_level(lit[0]));
      assert(SAT_level != ROOT_LEVEL);
      /* not root level otherwise reduced to empty clause earlier */
      analyse(clause);
    }
#if defined(DEBUG_SAT) && DEBUG_LEVEL > 1
  check_consistency();
  check_consistency_heap();
#endif
  return clause;
}

/*--------------------------------------------------------------*/

Tclause
SAT_clause_new_lazy(unsigned n, Tlit * lit)
{
  /* TODO CHECK if there is no interleaving of proofs */
  unsigned i = 0, j = 0;
  Tclause clause;
#if defined(DEBUG_SAT) && DEBUG_LEVEL > 1
  fprintf(stderr, "SAT_clause_new_lazy :");
  for (i = 0; i < n; ++i)
    fprintf(stderr, " %d", lit[i]);
  fprintf(stderr, "\n");
  check_consistency();
  check_consistency_heap();
#endif
  veriT_qsort(lit, n, sizeof(Tlit), (TFcmp) SAT_lit_compare_level);
  /* checking for complementary literals, true literals,
     and eliminating duplicates */
#ifdef DEBUG
  assert(n && SAT_lit_value(lit[0]) == VAL_TRUE);
  assert(SAT_lit_reason(lit[0]) == CLAUSE_LAZY);
  for (i = 0; i < n; i++)
    assert((i == 0 || SAT_lit_value(lit[i]) == VAL_FALSE) &&
	   SAT_lit_level(lit[i]) <= SAT_lit_level(lit[0]) &&
	   (i == 0 || lit[i] != lit[i - 1]));
#endif
  for (j = 1, i = 1; i < n; ++i)
    if (SAT_lit_value(lit[i]) != VAL_FALSE ||
	SAT_lit_level(lit[i]) != ROOT_LEVEL)
      lit[j++] = lit[i];
#ifdef PROOF
    else if (SAT_proof)
      {
	if (i == j)
	  {
	    Tlit * Plit2;
	    MY_MALLOC(Plit2, n * sizeof(Tlit));
	    memcpy(Plit2, lit, n * sizeof(Tlit));
	    clause = clause_new(n, Plit2, 0, 0, 0);
#ifdef INSIDE_VERIT
	    proof_SAT_set_id(clause);
#endif
	    proof_begin(clause);
	  }
	assert(SAT_lit_reason(lit[i]) && SAT_lit_reason(lit[i]) != CLAUSE_LAZY);
	proof_resolve(lit[i], SAT_lit_reason(lit[i]));
      }
  if (SAT_proof && n != j)
    {
      n = j;
      MY_REALLOC(lit, n * sizeof(lit));
      clause = clause_new(n, lit, 0, 1, 1);
      proof_end(clause);
    }
  else
    {
      n = j;
      MY_REALLOC(lit, n * sizeof(lit));
      clause = clause_new(n, lit, 0, 1, 1);
#ifdef INSIDE_VERIT
      if (SAT_proof)
	proof_SAT_set_id(clause);
#endif
    }
#else /* PROOF */
  n = j;
  MY_REALLOC(lit, n * sizeof(lit));
  clause = clause_new(n, lit, 0, 1, 1);
#endif /* PROOF */
#ifdef EXPERIMENT_WITH_ACTIVITY
  var_decrease_activity();
  for (i = 0; i < n; i++)
    var_increase_activity(SAT_lit_var(lit[i]));
  var_decrease_activity();
  for (i = 0; i < n; i++)
    var_increase_activity(SAT_lit_var(lit[i]));
#endif
  SAT_var_set_reason(SAT_lit_var(lit[0]), clause);
#if defined(DEBUG_SAT) && DEBUG_LEVEL > 1
  check_consistency();
  check_consistency_heap();
#endif
  return clause;
}

/*--------------------------------------------------------------*/

Tclause
SAT_clause_new(unsigned n, Tlit * lit)
{
  return SAT_clause_new_aux(n, lit, 0);
}

/*--------------------------------------------------------------*/

Tclause
SAT_clause_new_conflict(unsigned n, Tlit * lit)
{
  return SAT_clause_new_aux(n, lit, 1);
}

/*--------------------------------------------------------------*/

#ifdef BACKTRACK
/**
   \author Pascal Fontaine
   \brief adds a backtrackable point */
void
SAT_push(void)
{
  STACK_RESIZE_EXP(history, history_n + 2, history_size, sizeof(Tclause));
#ifndef PRESERVE_CLAUSES
  history[history_n].history_type = SAVE_CLAUSE_FREE_LIST;
  history[history_n++].clause = first_free_clause;
  first_free_clause = CLAUSE_UNDEF;
#endif
  history[history_n].history_type = PUSH_MARKUP;
  history[history_n++].clause = stack_clause_n;
}

/*--------------------------------------------------------------*/

/**
   \author Pascal Fontaine
   \brief backtracks to the latest backtrackable point */
void
SAT_pop(void)
{
  while (history_n)
    {
      --history_n;
      switch (history[history_n].history_type)
	{
	case PUSH_MARKUP :
	  {
	    unsigned stack_clause_bt = history[history_n].clause;
	    while (stack_clause_n > stack_clause_bt)
	      clause_remove(stack_clause_n--);
	    assert(stack_clause_n == stack_clause_bt);
	  }
	  return;
	case STATUS_CHANGED :
	  assert(SAT_status == SAT_STATUS_UNSAT);
	  assert(history[history_n].clause == CLAUSE_UNDEF);
	  SAT_status = SAT_STATUS_UNDEF;
	  break;
	case CLAUSE_UNSET_WATCHED :
	  clause_set_watched(history[history_n].clause);
	  break;
#ifndef PRESERVE_CLAUSES
	case SAVE_CLAUSE_FREE_LIST :
	  first_free_clause = history[history_n].clause;
	  break;
#endif
	default :
	  my_error("internal error: strange history type\n");
	}
    }
  my_error("too many pops\n");
}

#endif

/*
  --------------------------------------------------------------
  minimal models
  --------------------------------------------------------------
*/

#define SAT_VAL_MASK (1 << 4)
#define SAT_VAL_MASK2 (1 << 5)

void
SAT_minimal_model(SAT_Tlit ** PPlit, unsigned *n, unsigned options)
{
  unsigned i, j;
  Twatch * clauses_by_lit = NULL;
  unsigned * clauses_counter = NULL;
  Tclause clause;
  TSclause * PSclause;
  assert(SAT_stack_var_n == SAT_literal_stack_n);
  /* For each literal, associate the set of non-learnt clauses */
  /* For each clause, compute the number of satisfied literals */
  i = (unsigned) (((SAT_stack_var_n + 1) * 2) * sizeof(Twatch));
  MY_MALLOC(clauses_by_lit, i);
  memset(clauses_by_lit, 0, i);
  i = (stack_clause_n + 1u) * (unsigned) sizeof(unsigned);
  MY_MALLOC(clauses_counter, i);
  memset(clauses_counter, 0, i);
  for (i = 1, PSclause = stack_clause + 1; i <= stack_clause_n;
       i++, PSclause++)
    if (!PSclause->learnt && !PSclause->deleted)
      for (j = 0; j < PSclause->n; j++)
	{
	  Twatch * Pwatch = clauses_by_lit + PSclause->lit[j];
	  if (Pwatch->n == Pwatch->size)
	    {
	      Pwatch->size += !Pwatch->size;
	      Pwatch->size <<= 1;
	      MY_REALLOC(Pwatch->Pclause, (Pwatch->size * sizeof(Tclause)));
	    }
	  Pwatch->Pclause[Pwatch->n++] = i;
	  if (SAT_lit_value_is_true(PSclause->lit[j]))
	    clauses_counter[i]++;
	}
  if (!*n)
    {
      /* Copy the literal stack */
      MY_REALLOC(*PPlit, SAT_literal_stack_n * sizeof(Tlit));
      memcpy(*PPlit, SAT_literal_stack, SAT_literal_stack_n * sizeof(Tlit));
    }
  else
    {
      assert(*n == SAT_literal_stack_n);
    }
  /* A literal whose clauses are multi-satisfied can be eliminated */
  for (i = 0; i < SAT_literal_stack_n; i++)
    {
      Twatch * Pwatch = clauses_by_lit + (*PPlit)[i];
      if ((options & SAT_MIN_SKIP_PROPAGATED) && SAT_lit_reason((*PPlit)[i]))
	continue;
      for (j = 0; j < Pwatch->n; j++)
	if (clauses_counter[Pwatch->Pclause[j]] == 1)
	  goto next_literal;
	else assert(clauses_counter[Pwatch->Pclause[j]] != 0);
      assign[SAT_lit_var((*PPlit)[i])] |= SAT_VAL_MASK;
      for (j = 0; j < Pwatch->n; j++)
	clauses_counter[Pwatch->Pclause[j]]--;
    next_literal: ;
    }
  /* Free index */
  for (i = 0; i < (SAT_stack_var_n + 1) * 2; i++)
    free(clauses_by_lit[i].Pclause);
  free(clauses_by_lit);
  free(clauses_counter);
#ifdef OLD_TAUTOLOGIC_MINIMIZE
  /* Use conflict clause (should be tautologies)  to further compress */
  if (options & SAT_MIN_USE_TAUTOLOGIES)
    for (i = 1, PSclause = stack_clause + 1; i <= stack_clause_n;
	 i++, PSclause++)
      if (PSclause->conflict)
	{
	  Tlit lit = LIT_UNDEF;
	  for (j = 0; j < PSclause->n; j++)
	    {
	      Tvalue val = SAT_lit_value(PSclause->lit[j]);
	      if (val & (SAT_VAL_MASK | SAT_VAL_MASK2))
		goto next_clause;
	      if (val == VAL_UNDEF)
		goto next_clause;
	      if (val == VAL_TRUE)
		{
		  if (lit != LIT_UNDEF)
		    goto next_clause;
		  lit = PSclause->lit[j];
		}
	      assert(val == VAL_FALSE);
	    }
	  assign[SAT_lit_var(lit)] |= SAT_VAL_MASK2;
	next_clause: ;
	}
#else
  if (options & SAT_MIN_USE_TAUTOLOGIES)
    for (i = SAT_literal_stack_n; i-- > 0; )
      if ((clause = SAT_lit_reason((*PPlit)[i])) &&
	  (PSclause = stack_clause + clause)->conflict)
	{
	  for (j = 0; j < PSclause->n; j++)
	    if (SAT_lit_value(PSclause->lit[j]) & SAT_VAL_MASK)
	      goto next_clause;
	  assign[SAT_lit_var((*PPlit)[i])] |= SAT_VAL_MASK2;
	next_clause: ;
	}
#endif
  /* Compress the literal stack */
  for (i = j = 0; i < SAT_literal_stack_n; i++)
    if (SAT_lit_value((*PPlit)[i]) & (SAT_VAL_MASK | SAT_VAL_MASK2))
      assign[SAT_lit_var((*PPlit)[i])] &= 3;
    else
      (*PPlit)[j++] = (*PPlit)[i];
  *n = j;
}

/*
  --------------------------------------------------------------
  init and done
  --------------------------------------------------------------
*/

/**
   \author Pascal Fontaine
   \brief module initialise */
void
SAT_init(void)
{
  var_inc = 1;
  var_decay = 0.95;
  SAT_status = SAT_STATUS_SAT;
  SAT_level = ROOT_LEVEL;
  SAT_empty_clause = CLAUSE_UNDEF;
  conflict_nb = 0;
  stack_lit_to_propagate = 0;
  stack_lit_hold = 0;
  stack_lit_unit = 0;
  MY_MALLOC(SAT_stack_var, sizeof(TSvar));
  SAT_stack_var_size = 1;
  MY_MALLOC(assign, sizeof(Tvalue));
  assign[0] = VAL_UNDEF;
#ifdef HINT_AS_DECISION
  hint_n = hint_p = 0;
  hint_size = 4;
  MY_MALLOC(hints, hint_size * sizeof(Tlit));
#endif
  MY_MALLOC(watch, 2 * sizeof(Twatch));
  memset(watch, 0,  2 * sizeof(Twatch));
  SAT_stack_var[VAR_UNDEF].phase_cache = 0;
  SAT_stack_var[VAR_UNDEF].seen = 0;
  SAT_stack_var[VAR_UNDEF].decide = 0;
  SAT_stack_var[VAR_UNDEF].misc = 0;
  SAT_stack_var[VAR_UNDEF].level = 0;
  SAT_stack_var[VAR_UNDEF].reason = CLAUSE_UNDEF;
  SAT_stack_var[VAR_UNDEF].activity = 0;
#ifdef SAT_SYM
  MY_MALLOC(SAT_var_orbit, sizeof(Tvar));
  SAT_var_orbit[VAR_UNDEF] = VAR_UNDEF;
#endif

#if defined(BCLAUSE) || defined(BCLAUSE_LIGHT)
  bclause_resize(0, 2);
#endif
  ON_DEBUG_SAT(check_consistency());
#if STATS_LEVEL >= 1
#ifndef INSIDE_VERIT
  stats_init();
#endif
  stat_n_conflict = stats_counter_new("SAT_n_conflict",
				      "Number of conflicts in SAT", "%9d");
  stat_n_conflict_lit = stats_counter_new("SAT_n_conflict_lit",
					  "Number of literals in conflicts in SAT", "%9d");
  stat_n_decision = stats_counter_new("SAT_n_dec",
				      "Number of decisions in SAT", "%9d");
  stat_n_tp = stats_counter_new("SAT_n_tp",
				"Number of theory propagations in SAT", "%9d");
  stat_n_delete = stats_counter_new("SAT_n_del",
				    "Number of clause deletions in SAT", "%9d");
  stat_n_restart = stats_counter_new("SAT_n_restart",
				     "Number of restarts in SAT", "%6d");
  stat_n_purge = stats_counter_new("SAT_n_purge",
				   "Number of purges in SAT", "%6d");
  stat_n_clauses = stats_counter_new("SAT_n_clauses",
				     "Number of clauses added in SAT", "%9d");
  stat_n_prop = stats_counter_new("SAT_n_prop",
				  "Number of propagation", "%9d");
#if STATS_LEVEL >= 2
  stat_n_watched = stats_counter_new("SAT_n_watch",
				     "Number of clauses examined by watched", "%9d");
  stat_prop_lit_call_nowatch =
    stats_counter_new("SAT_prop_lit_call_nowatch",
		      "Number of calls to prop_lit with no watchers", "%9d");
  stat_prop_call = 
    stats_counter_new("SAT_prop_call",
		      "Number of calls to propagate", "%9d");
  stat_prop_call_waste = 
    stats_counter_new("SAT_prop_call_waste",
		      "Number of calls to propagate with nothing to propagate", "%9d");
  stat_prop_call_noprop = 
    stats_counter_new("SAT_prop_call_noprop",
		      "Number of calls to propagate without further propagation", "%9d");
#endif
#endif
}

/*--------------------------------------------------------------*/

/**
   \author Pascal Fontaine
   \brief module release */
void
SAT_done(void)
{
  unsigned i;
#ifdef PROOF
  proof_stack_size = 0;
  free(proof_stack_lit);
  proof_stack_lit = NULL;
  free(proof_stack_clause);
  proof_stack_clause = NULL;
  proof_stack_size_2 = 0;
  free(proof_stack_lit_2);
  proof_stack_lit_2 = NULL;
  free(proof_stack_clause_2);
  proof_stack_clause_2 = NULL;
#endif
  for (i = 1; i <= SAT_stack_var_n; ++i)
    SAT_var_free(i);
  for (i = (SAT_stack_var_n + 1) << 1; i < 2 * SAT_stack_var_size; ++i)
    free(watch[i].Pclause);
  free(watch);
#if defined(BCLAUSE) || defined(BCLAUSE_LIGHT)
  for (i = 0; i < (SAT_stack_var_n + 1) << 1; i++)
    free(bclause_lit[i].prop);
  free(bclause_lit);
#endif
  free(SAT_stack_var);
  SAT_stack_var = NULL;
  SAT_stack_var_n = 0;
  SAT_stack_var_size = 0;
#ifdef SAT_SYM
  free(SAT_var_orbit);
  SAT_var_orbit = NULL;
#endif
  for (i = 1; i <= stack_clause_n; ++i)
    SAT_clause_free(i);
  free(stack_clause);
  stack_clause = NULL;
  stack_clause_n = 0;
  stack_clause_size = 0;
  free(learnts);
  learnts = NULL;
  learnts_n = 0;
  learnts_size = 0;
  heap_var_free();
  free(stack_lit);
  stack_lit = NULL;
  stack_lit_n = 0;
  stack_lit_size = 0;
  free(stack_level);
  stack_level = NULL;
  stack_level_size = 0;
#ifdef BACKTRACK
  free(history);
  history = NULL;
  history_size = 0;
  history_n = 0;
#endif
  SAT_level = ROOT_LEVEL;
  free(misc_stack);
  misc_stack = NULL;
  misc_stack_size = 0;
  misc_stack_n = 0;
  free(assign);
#ifdef HINT_AS_DECISION
  free(hints);
  hint_n = hint_p = hint_size = 0;
#endif
#ifndef INSIDE_VERIT
#if STATS_LEVEL >= 1
  stats_fprint(stdout);
  stats_done();
#endif
#endif
}

/*--------------------------------------------------------------*/

/**
   \author Pascal Fontaine
   \brief module reset */
void
SAT_reset(void)
{
  unsigned i;
#ifdef PROOF
  proof_stack_size = 0;
  free(proof_stack_lit);
  proof_stack_lit = NULL;
  free(proof_stack_clause);
  proof_stack_clause = NULL;
  proof_stack_size_2 = 0;
  free(proof_stack_lit_2);
  proof_stack_lit_2 = NULL;
  free(proof_stack_clause_2);
  proof_stack_clause_2 = NULL;
#endif
  for (i = 1; i <= SAT_stack_var_n; ++i)
    SAT_var_free(i);
  for (i = (SAT_stack_var_n + 1) << 1; i < 2 * SAT_stack_var_size; ++i)
    free(watch[i].Pclause);
  free(watch);
#if defined(BCLAUSE) || defined(BCLAUSE_LIGHT)
  for (i = 0; i < (SAT_stack_var_n + 1) << 1; i++)
    free(bclause_lit[i].prop);
  free(bclause_lit);
#endif
  free(SAT_stack_var);
  SAT_stack_var = NULL;
  SAT_stack_var_n = 0;
  SAT_stack_var_size = 0;
  for (i = 1; i <= stack_clause_n; ++i)
    SAT_clause_free(i);
  free(stack_clause);
  stack_clause = NULL;
  stack_clause_n = 0;
  stack_clause_size = 0;
  free(learnts);
  learnts = NULL;
  learnts_n = 0;
  learnts_size = 0;
  heap_var_free();
  free(stack_lit);
  stack_lit = NULL;
  stack_lit_n = 0;
  stack_lit_size = 0;
  free(stack_level);
  stack_level = NULL;
  stack_level_size = 0;
#ifdef BACKTRACK
  free(history);
  history = NULL;
  history_size = 0;
  history_n = 0;
#endif
  SAT_level = ROOT_LEVEL;
  free(misc_stack);
  misc_stack = NULL;
  misc_stack_size = 0;
  misc_stack_n = 0;
  free(assign);
#ifdef HINT_AS_DECISION
  free(hints);
#endif

  var_inc = 1;
  var_decay = 0.95;
  SAT_status = SAT_STATUS_SAT;
  SAT_level = ROOT_LEVEL;
  SAT_empty_clause = CLAUSE_UNDEF;
  conflict_nb = 0;
  stack_lit_to_propagate = 0;
  stack_lit_hold = 0;
  stack_lit_unit = 0;
  MY_MALLOC(SAT_stack_var, sizeof(TSvar));
  SAT_stack_var_size = 1;
  MY_MALLOC(assign, sizeof(Tvalue));
  assign[0] = VAL_UNDEF;
#ifdef HINT_AS_DECISION
  hint_n = hint_p = 0;
  hint_size = 4;
  MY_MALLOC(hints, hint_size * sizeof(Tlit));
#endif
  MY_MALLOC(watch, 2 * sizeof(Twatch));
  memset(watch, 0,  2 * sizeof(Twatch));
  SAT_stack_var[VAR_UNDEF].phase_cache = 0;
  SAT_stack_var[VAR_UNDEF].seen = 0;
  SAT_stack_var[VAR_UNDEF].decide = 0;
  SAT_stack_var[VAR_UNDEF].misc = 0;
  SAT_stack_var[VAR_UNDEF].level = 0;
  SAT_stack_var[VAR_UNDEF].reason = CLAUSE_UNDEF;
  SAT_stack_var[VAR_UNDEF].activity = 0;
#if defined(BCLAUSE) || defined(BCLAUSE_LIGHT)
  bclause_resize(0, 2);
#endif
  ON_DEBUG_SAT(check_consistency());
}

/*--------------------------------------------------------------*/

#ifdef DEBUG_SAT

static void
check_consistency(void)
{
  int * count_watch = NULL;
  unsigned i, j, k;
  Tvar var;
  Tlit lit;
  MY_MALLOC(count_watch, (stack_clause_n + 1) * sizeof(int));
  for (i = 0; i <= stack_clause_n; ++i)
    count_watch[i] = 0;
  for (var = 1; var <= SAT_stack_var_n; var++)
    {
      Tclause clause;
      assert(SAT_var_value(var) != VAL_UNDEF ||
	     !SAT_stack_var[var].decide ||
	     heap_var_in(var));
      clause = SAT_stack_var[var].reason;
      if (clause == CLAUSE_LAZY || clause == CLAUSE_UNDEF)
	continue;
#if defined(BCLAUSE) || defined(BCLAUSE_LIGHT)
      if (stack_clause[clause].n == 2)
	assert(SAT_lit_var(stack_clause[clause].lit[0]) == var ||
	       SAT_lit_var(stack_clause[clause].lit[1]) == var);
      else
	assert(SAT_lit_var(stack_clause[clause].lit[0]) == var);
#else
      assert(SAT_lit_var(stack_clause[clause].lit[0]) == var);
#endif
      for (i = 0, j = 0; i < stack_clause[clause].n; i++)
	if (SAT_lit_value(stack_clause[clause].lit[i]) == VAL_FALSE)
	  j++;
	else
	  assert(SAT_lit_value(stack_clause[clause].lit[i]) == VAL_TRUE &&
		 SAT_lit_var(stack_clause[clause].lit[i]) == var);
      assert(j == stack_clause[clause].n - 1);
    }
  for (lit = 2; lit < (SAT_stack_var_n + 1) << 1; lit++)
    for (j = 0; j < watch[lit].n; j++)
      {
	TSclause * PSclause = stack_clause + watch[lit].Pclause[j];
	assert(watch[lit].Pclause[j] <= stack_clause_n);
	assert(PSclause->watched);
#if defined(BCLAUSE) || defined(BCLAUSE_LIGHT)
	assert(PSclause->n >= 3);
#else
	assert(PSclause->n >= 2);
#endif
	assert(PSclause->lit[0] == lit || PSclause->lit[1] == lit);
	if (PSclause->lit[0] == lit)
	  {
	    assert(!(count_watch[watch[lit].Pclause[j]]&1));
	    count_watch[watch[lit].Pclause[j]] |= 1;
	  }
	else
	  {
	    assert(!(count_watch[watch[lit].Pclause[j]]&2));
	    count_watch[watch[lit].Pclause[j]] |= 2;
	  }
      }
  for (i = 1; i <= stack_clause_n; ++i)
    {
      assert(stack_clause[i].n != 0 || count_watch[i] == 0);
      assert(stack_clause[i].n != 1 || count_watch[i] == 0);
      assert(stack_clause[i].n < 2 ||
	     stack_clause[i].deleted ||
	     !stack_clause[i].watched ||
	     count_watch[i] == 3);
#if defined(BCLAUSE) || defined(BCLAUSE_LIGHT)
      assert(stack_clause[i].n < 3 ||
	     stack_clause[i].deleted ||
	     !stack_clause[i].watched ||
	     count_watch[i] == 3);
#else
      assert(stack_clause[i].n < 2 ||
	     stack_clause[i].deleted ||
	     !stack_clause[i].watched ||
	     count_watch[i] == 3);
#endif
    }
  free(count_watch);

  for (i = 0, k = 0; i < stack_lit_n; i++)
    {
      assert(SAT_lit_value(stack_lit_get(i)) == VAL_TRUE);
      if (SAT_lit_reason(stack_lit_get(i)) == CLAUSE_UNDEF)
	k++;
      SAT_stack_var[SAT_lit_var(stack_lit_get(i))].misc = 1;
#ifdef PROOF
      assert(SAT_proof ||
	     SAT_lit_level(stack_lit_get(i)) > ROOT_LEVEL ||
	     SAT_lit_seen(stack_lit_get(i)));
#else
      assert(SAT_lit_level(stack_lit_get(i)) > ROOT_LEVEL ||
	     SAT_lit_seen(stack_lit_get(i)));
#endif
    }
  assert (SAT_level == k + ROOT_LEVEL);
  for (var = 1; var <= SAT_stack_var_n; var++)
    assert (SAT_stack_var[var].misc || 
	    (SAT_var_value(var) == VAL_UNDEF &&
	     SAT_stack_var[var].reason == CLAUSE_UNDEF));
  for (i = 0; i < stack_lit_n; i++)
    SAT_stack_var[SAT_lit_var(stack_lit_get(i))].misc = 0;
#if defined(BCLAUSE) || defined(BCLAUSE_LIGHT)
  for (i = 0; i < (SAT_stack_var_n + 1) << 1; i++)
    if (!bclause_lit[i].updated)
      assert(bclause_lit[i].n == bclause_lit[i].direct);
#endif
}

/*--------------------------------------------------------------*/

static void
check_consistency_propagation(void)
{
  unsigned i;
  for (i = 1; i <= stack_clause_n; ++i)
    {
      unsigned count_p = 0, count_n = 0, j;
      TSclause * PSclause = stack_clause + i;
      if (!PSclause->watched)
	continue;
      for (j = 0; j < PSclause->n; j++)
	switch (SAT_lit_value(PSclause->lit[j]))
	  {
	  case VAL_FALSE: count_n++; break;
	  case VAL_TRUE: count_p++; break;
	  }
      assert (count_n < PSclause->n); /* otherwise conflicting */
      assert (count_p > 0 || count_n + 1 < PSclause->n); /* otherwise prop */
      if (!count_p)
	{
	  assert (SAT_lit_value_undef(PSclause->lit[0]));
	  assert (SAT_lit_value_undef(PSclause->lit[1]));
	}
    }
}

/*--------------------------------------------------------------*/

static void
print_clause(Tclause clause)
{
  unsigned k;
  assert(clause <= stack_clause_n);
  fprintf(stderr, "%d : ", clause);
  for (k = 0; k < stack_clause[clause].n; k++)
    fprintf(stderr, "%d ", stack_clause[clause].lit[k]);
  fprintf(stderr, "\n");
}

/*--------------------------------------------------------------*/

static void
print_stack(void)
{
  static int count = 0;
  unsigned i, k;
  for (i = 1; i <= stack_clause_n; i++)
    {
      if (stack_clause[i].deleted)
	continue;
      fprintf(stderr, "%d : ", i);
      for (k = 0; k < stack_clause[i].n; k++)
	fprintf(stderr, "%d ", stack_clause[i].lit[k]);
      fprintf(stderr, "\n");
    }

  fprintf(stderr, "stack size %u (nb var %u)\n", stack_lit_n, SAT_stack_var_n);
  for (i = 0; i < stack_lit_n; i++)
    if (SAT_lit_reason(stack_lit_get(i)) == CLAUSE_UNDEF)
      fprintf(stderr, "[%d] ", stack_lit_get(i));
    else
      fprintf(stderr, "%d (%u) ", stack_lit_get(i),
	      SAT_lit_reason(stack_lit_get(i)));
  fprintf(stderr, "\n");
  fprintf(stderr, "call %d\n", ++count);
}

/*--------------------------------------------------------------*/

static void
check_consistency_heap(void)
{
  /* check if all unassigned variables are in the heap */
  unsigned i;
  unsigned count = 0;
  for (i = 0; i < heap_var_n; ++i)
    if (SAT_var_value(heap_var[i]) == VAL_UNDEF)
      count++;
  if (count + stack_lit_n != SAT_stack_var_n)
    {
      for (i = 1; i < SAT_stack_var_n; ++i)
	assert(SAT_var_value(i) != VAL_UNDEF || heap_var_in(i));
      assert (count + stack_lit_n == SAT_stack_var_n);
    }
  for (i = 0; i < heap_var_n; ++i)
    assert(heap_index[heap_var[i]] == i);
  for (i = 0; i < heap_index_size; ++i)
    assert(heap_index[i] == HEAP_INDEX_UNDEF || heap_var[heap_index[i]] == i);  
}

/*--------------------------------------------------------------*/

static void
check_consistency_final(void)
{
  unsigned i, j, ok;
  print_stack();
  for (i = 1; i <= stack_clause_n; ++i)
    {
      if (stack_clause[i].deleted)
	continue;
      for (ok = 0, j = 0; !ok && j < stack_clause[i].n; ++j)
	ok |= SAT_lit_value(stack_clause[i].lit[j]) == VAL_TRUE;
      if (!ok)
	{
	  printf("unsatisfied clause found:");
	  print_clause(i);
	  assert(0);
	}
    }
}

#endif
