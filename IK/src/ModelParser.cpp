/* A Bison parser, made from skel.y
   by GNU bison 1.35.  */

#define YYBISON 1  /* Identify Bison output.  */

#line 1 "skel.y"

#include <malloc.h>
#include <fstream>
#include <assert.h>
#include "ParserDefs.h"

#ifndef __ARTICULATEDBODY_H__
#include "ArticulatedBody.h"
#endif	//__ARTICULATEDBODY_H__

#ifndef __TRANSFORM_H__
#include "Transform.h"
#endif	//__TRANSFORM_H__

#ifndef	__TRANSLATE_H__
#include "Translate.h"
#endif	//__TRANSLATE_H__

#ifndef	__SCALE_H__
#include "Scale.h"
#endif	//__SCALE_H__

#ifndef __ROTATEEULER_H__
#include "RotateEuler.h"
#endif	//__ROTATEEULER_H__

using namespace std;

vector<char*>* dNames;
vector<Transform*> *gChain;

//#define YYDEBUG 1

int yylex();
void yyerror(char*);
void openFile(FILE*);
void closeFile();

ArticulatedBody* gSkel;


TransformNode*	cur_node;
TransformNode*	stack[256];
int		stack_top;

int IdStack[256];
int IdStackTop;
int cur_id;

struct dof_lookup1{
	char* name;
	Dof* dof;
} dof_lookup[256];
int num_dofs;

struct node_lookup1{
	char* name;
	TransformNode* node;
} node_lookup[256];
int num_nodes;

struct mass_lookup1{
	char *name;
	double mass;
} mass_lookup[256];
int num_masses;

void __setPrimitive( vec3v, vec3v );
void __setPrimitive( vec3v, vec3v, Dof*);
void __setPrimitive( vec3v, vec3v, Dof*, Primitive*);
void __setTransform( Transform* );
//void __setTransform( vector<Transform*>* );
void __finishSkeleton();
void __endNode();
void __startNode( char*, int );
void __startChain( int );
Transform* __endChain();
//vector<Transform*>* __endChain();`
Transform* __createTranslate( vec3d );
Transform* __createTelescope( vec3v, Dof* );
Transform* __createScale( vec3v );
Transform* __createRotateEuler( Dof*, int );
Transform* __createRotateEuler( double, int );
Dof* __createDOF( double );
Dof* __createDOF( char* );
Dof* __createDOF( char*, double, double, double );
Dof* __createDOF( char*, double, double, double, double );
void __createHandle( char*, vec3v, int, char* );
void __addHandle( char*, Model*, TransformNode*, int, Vec3d, bool );
void __recordMass(char*, double);
Primitive* __setGeometry(char*, char*, vec3v);
Primitive* __setGeometry(char*, char*);
Primitive* __setGeometry(char*, vec3v);
Primitive* __setGeometry(char*);
Primitive* __setGeometry(vec3v);


#line 92 "skel.y"
#ifndef YYSTYPE
typedef union {
	double	dValue;
	int iValue;
	char* sValue;
	vec3d v3DValue;
	vec4d v4DValue;
	vec3v v3VValue;
	Transform* tValue;
	Primitive* pValue;
	Dof* dofValue;
} yystype;
# define YYSTYPE yystype
# define YYSTYPE_IS_TRIVIAL 1
#endif
#ifndef YYDEBUG
# define YYDEBUG 0
#endif



#define	YYFINAL		171
#define	YYFLAG		-32768
#define	YYNTBASE	29

/* YYTRANSLATE(YYLEX) -- Bison token number corresponding to YYLEX. */
#define YYTRANSLATE(x) ((unsigned)(x) <= 274 ? yytranslate[x] : 63)

/* YYTRANSLATE[YYLEX] -- Bison token number corresponding to YYLEX. */
static const char yytranslate[] =
{
       0,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,    23,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
      24,     2,    25,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
      26,    27,    28,    21,     2,    22,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     1,     3,     4,     5,
       6,     7,     8,     9,    10,    11,    12,    13,    14,    15,
      16,    17,    18,    19,    20
};

#if YYDEBUG
static const short yyprhs[] =
{
       0,     0,     4,     7,    12,    17,    22,    27,    29,    32,
      37,    41,    45,    50,    52,    55,    57,    59,    61,    63,
      65,    67,    69,    71,    73,    75,    77,    79,    82,    86,
      90,    95,   102,   107,   112,   119,   126,   131,   133,   136,
     141,   143,   146,   155,   157,   160,   169,   180,   187,   196,
     206,   213,   218,   223,   226,   229,   237,   247,   255,   257,
     259,   261,   263,   265,   267
};
static const short yyrhs[] =
{
      30,    34,    32,     0,    30,    34,     0,    30,    31,    34,
      32,     0,    18,    21,    53,    22,     0,    20,    21,    49,
      22,     0,    19,    21,    51,    22,     0,    34,     0,    33,
      34,     0,    35,    36,    33,    22,     0,    35,    36,    22,
       0,    35,    33,    22,     0,    16,     5,    21,     4,     0,
      37,     0,    37,    36,     0,    38,     0,    52,     0,    55,
       0,    40,     0,    42,     0,    43,     0,    44,     0,    45,
       0,    46,     0,    47,     0,    48,     0,    38,     0,    39,
      38,     0,    41,    39,    22,     0,     7,    21,     4,     0,
       8,    21,    57,    22,     0,     9,    21,    59,    23,    60,
      22,     0,    10,    21,    59,    22,     0,    11,    21,    58,
      22,     0,    13,    21,    60,    23,    61,    22,     0,    14,
      21,    62,    23,    61,    22,     0,    12,    21,    57,    22,
       0,    50,     0,    49,    50,     0,     5,    21,    62,    22,
       0,    52,     0,    51,    52,     0,     5,    21,    59,    23,
       4,    23,     5,    22,     0,    54,     0,    53,    54,     0,
       5,    21,    62,    23,    62,    23,    62,    22,     0,     5,
      21,    62,    23,    62,    23,    62,    23,    62,    22,     0,
       6,    21,    59,    23,    59,    22,     0,     6,    21,    59,
      23,    59,    23,    60,    22,     0,     6,    21,    59,    23,
      59,    23,    60,    23,    56,     0,     5,    23,     5,    23,
      59,    22,     0,     5,    23,     5,    22,     0,     5,    23,
      59,    22,     0,    59,    22,     0,     5,    22,     0,    24,
      60,    23,    60,    23,    60,    25,     0,    24,    60,    23,
      60,    23,    60,    23,    60,    25,     0,    24,    62,    23,
      62,    23,    62,    25,     0,    62,     0,     5,     0,    26,
       0,    27,     0,    28,     0,     4,     0,     3,     0
};

#endif

#if YYDEBUG
/* YYRLINE[YYN] -- source line where rule number YYN was defined. */
static const short yyrline[] =
{
       0,   139,   141,   142,   145,   149,   153,   157,   159,   162,
     164,   165,   168,   172,   174,   177,   179,   180,   183,   185,
     186,   187,   188,   189,   190,   191,   194,   196,   199,   203,
     207,   211,   215,   219,   223,   227,   231,   235,   237,   240,
     244,   246,   249,   254,   256,   259,   261,   264,   266,   267,
     270,   272,   273,   274,   275,   278,   282,   286,   290,   292,
     295,   297,   298,   301,   303
};
#endif


#if (YYDEBUG) || defined YYERROR_VERBOSE

/* YYTNAME[TOKEN_NUM] -- String name of the token TOKEN_NUM. */
static const char *const yytname[] =
{
  "$", "error", "$undefined.", "FLOAT", "INTEGER", "STRING", "PRIMITIVE", 
  "CHAIN", "TRANSLATE", "TELESCOPE", "SCALE", "ROTATE_EULER", "ROTATE_CONS", "HANDLE", "NODE", 
  "CONST", "DOFS", "HANDLES", "MASS", "'{'", "'}'", "','", "'<'", "'>'", 
  "'x'", "'y'", "'z'", "skel_file", "dof_list", "mass_list", 
  "handle_list", "nodes", "node", "node_start", "node_elements", 
  "node_element", "transform", "transforms", "chain", "chain_start", 
  "translate", "telescope", "scale", "rotate_euler", 
  "rotate_cons", "masses", "mass", "handles", "handle", 
  "dofs", "dof", "primitive", "geometry", "dof_vec3", "dof_vec4", 
  "val_vec3", "dof_val", "axis", "number", 0
};
#endif

/* YYR1[YYN] -- Symbol number of symbol that rule YYN derives. */
static const short yyr1[] =
{
       0,    29,    29,    29,    30,    31,    32,    33,    33,    34,
      34,    34,    35,    36,    36,    37,    37,    37,    38,    38,
      38,    38,    38,    38,    38,    38,    39,    39,    40,    41,
      42,    43,    44,    45,    46,    47,    48,    49,    49,    50,
      51,    51,    52,    53,    53,    54,    54,    55,    55,    55,
      56,    56,    56,    56,    56,    57,    58,    59,    60,    60,
      61,    61,    61,    62,    62
};

/* YYR2[YYN] -- Number of symbols composing right hand side of rule YYN. */
static const short yyr2[] =
{
       0,     3,     2,     4,     4,     4,     4,     1,     2,     4,
       3,     3,     4,     1,     2,     1,     1,     1,     1,     1,
       1,     1,     1,     1,     1,     1,     1,     2,     3,     3,
       4,     6,     4,     4,     6,     6,     4,     1,     2,     4,
       1,     2,     8,     1,     2,     8,    10,     6,     8,     9,
       6,     4,     4,     2,     2,     7,     9,     7,     1,     1,
       1,     1,     1,     1,     1
};

/* YYDEFACT[S] -- default rule to reduce with in state S when YYTABLE
   doesn't specify something else to do.  Zero means the default is an
   error. */
static const short yydefact[] =
{
       0,     0,     0,     0,     0,     0,     0,     2,     0,     0,
       0,    43,     0,     0,     0,     0,     1,     0,     0,     0,
       0,     0,     0,     0,     0,     0,     0,     0,     7,     0,
      13,    15,    18,     0,    19,    20,    21,    22,    23,    24,
      25,    16,    17,     0,     4,    44,     0,     0,     0,    37,
       3,     0,     0,     0,     0,     0,     0,     0,     0,     0,
       0,     0,    11,     8,    10,     0,    14,    26,     0,    64,
      63,     0,    12,     0,     5,    38,     0,    40,     0,     0,
       0,    29,     0,     0,     0,     0,     0,     0,     0,    59,
       0,    58,     0,     9,    28,    27,     0,     0,     6,    41,
       0,     0,     0,     0,    30,     0,    32,     0,    33,    36,
       0,     0,     0,    39,     0,     0,     0,     0,     0,     0,
      60,    61,    62,     0,     0,     0,     0,     0,    47,     0,
       0,    31,     0,    34,    35,     0,     0,     0,     0,     0,
       0,    45,     0,     0,    42,    48,     0,     0,     0,     0,
      57,     0,    49,     0,    55,     0,    46,    54,     0,    53,
       0,     0,     0,    56,    51,     0,    52,     0,    50,     0,
       0,     0
};

static const short yydefgoto[] =
{
     169,     2,     6,    16,    27,    28,     8,    29,    30,    31,
      68,    32,    33,    34,    35,    36,    37,    38,    39,    40,
      48,    49,    76,    41,    10,    11,    42,   152,    83,    87,
      79,    90,   123,    91
};

static const short yypact[] =
{
      27,    18,    21,    48,    56,    43,    50,    49,   108,    46,
      11,-32768,    52,    64,    49,    53,-32768,    54,    55,    58,
      59,    60,    61,    71,    74,    75,    76,    -2,-32768,    -1,
     120,-32768,-32768,   128,-32768,-32768,-32768,-32768,-32768,-32768,
  -32768,-32768,-32768,     9,-32768,-32768,    66,    80,    12,-32768,
  -32768,    72,    47,    47,    94,    78,    47,    47,    79,    78,
      19,     9,-32768,-32768,-32768,    22,-32768,-32768,    77,-32768,
  -32768,    81,-32768,     9,-32768,-32768,    13,-32768,     9,    83,
      84,-32768,    19,    86,    87,    89,    19,   101,   121,-32768,
     122,-32768,   123,-32768,-32768,-32768,     9,   125,-32768,-32768,
     126,   105,    47,   127,-32768,    19,-32768,   129,-32768,-32768,
      20,    20,   130,-32768,     9,   131,     3,    19,   133,    19,
  -32768,-32768,-32768,   134,   135,     9,   136,   139,-32768,    19,
     137,-32768,   138,-32768,-32768,    28,     9,   140,    33,    19,
      19,-32768,     9,   141,-32768,-32768,     5,   142,   145,   143,
  -32768,    35,-32768,   147,-32768,    19,-32768,-32768,     6,-32768,
     146,    37,   148,-32768,-32768,    47,-32768,   150,-32768,   151,
     158,-32768
};

static const short yypgoto[] =
{
  -32768,-32768,-32768,   149,   119,     0,-32768,   144,-32768,   -25,
  -32768,-32768,-32768,-32768,-32768,-32768,-32768,-32768,-32768,-32768,
  -32768,   116,-32768,   -44,-32768,   163,-32768,-32768,   117,-32768,
     -53,   -77,    67,   -42
};


#define	YYLAST		178


static const short yytable[] =
{
      80,    71,     7,    84,    85,   103,    14,    77,    67,   107,
     151,   161,    69,    70,     4,     4,     9,    47,    17,    92,
      62,    64,    69,    70,    89,   128,   129,    63,   118,    78,
      78,    97,    99,    44,    74,    98,   100,     4,     4,     3,
     130,     5,   132,    95,    93,     1,   120,   121,   122,   116,
     141,   142,   138,     9,   112,   145,   146,   157,   158,   164,
     165,    12,   147,   148,    13,    63,     4,    43,    15,    47,
      72,    78,   126,    46,    51,    52,    53,    17,   160,    54,
      55,    56,    57,   135,    19,    20,    21,    22,    23,    24,
      25,    26,    58,   153,   143,    59,    60,    61,    81,    94,
     149,    73,    82,    86,    96,   162,   101,   102,   104,   115,
     105,   106,   167,    17,    18,    19,    20,    21,    22,    23,
      24,    25,    26,   108,     4,    17,    18,    19,    20,    21,
      22,    23,    24,    25,    26,    19,    20,    21,    22,    23,
      24,    25,    26,   109,   137,   110,   111,   113,    65,   114,
     117,   170,   119,   125,   127,   131,   133,   134,   171,   136,
     139,   140,   144,    50,    75,   156,   150,   154,   155,   159,
     166,   163,   168,    45,    66,     0,    88,     0,   124
};

static const short yycheck[] =
{
      53,    43,     2,    56,    57,    82,     6,    51,    33,    86,
       5,     5,     3,     4,    16,    16,     5,     5,     5,    61,
      22,    22,     3,     4,     5,    22,    23,    27,   105,    24,
      24,    73,    76,    22,    22,    22,    78,    16,    16,    21,
     117,    20,   119,    68,    22,    18,    26,    27,    28,   102,
      22,    23,   129,     5,    96,    22,    23,    22,    23,    22,
      23,     5,   139,   140,    21,    65,    16,    21,    19,     5,
       4,    24,   114,    21,    21,    21,    21,     5,   155,    21,
      21,    21,    21,   125,     7,     8,     9,    10,    11,    12,
      13,    14,    21,   146,   136,    21,    21,    21,     4,    22,
     142,    21,    24,    24,    23,   158,    23,    23,    22,     4,
      23,    22,   165,     5,     6,     7,     8,     9,    10,    11,
      12,    13,    14,    22,    16,     5,     6,     7,     8,     9,
      10,    11,    12,    13,    14,     7,     8,     9,    10,    11,
      12,    13,    14,    22,     5,    23,    23,    22,    29,    23,
      23,     0,    23,    23,    23,    22,    22,    22,     0,    23,
      23,    23,    22,    14,    48,    22,    25,    25,    23,    22,
      22,    25,    22,    10,    30,    -1,    59,    -1,   111
};
/* -*-C-*-  Note some compilers choke on comments on `#line' lines.  */
#line 3 "/usr/share/bison/bison.simple"

/* Skeleton output parser for bison,

   Copyright (C) 1984, 1989, 1990, 2000, 2001, 2002 Free Software
   Foundation, Inc.

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2, or (at your option)
   any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place - Suite 330,
   Boston, MA 02111-1307, USA.  */

/* As a special exception, when this file is copied by Bison into a
   Bison output file, you may use that output file without restriction.
   This special exception was added by the Free Software Foundation
   in version 1.24 of Bison.  */

/* This is the parser code that is written into each bison parser when
   the %semantic_parser declaration is not specified in the grammar.
   It was written by Richard Stallman by simplifying the hairy parser
   used when %semantic_parser is specified.  */

/* All symbols defined below should begin with yy or YY, to avoid
   infringing on user name space.  This should be done even for local
   variables, as they might otherwise be expanded by user macros.
   There are some unavoidable exceptions within include files to
   define necessary library symbols; they are noted "INFRINGES ON
   USER NAME SPACE" below.  */

#if ! defined (yyoverflow) || defined (YYERROR_VERBOSE)

/* The parser invokes alloca or malloc; define the necessary symbols.  */

# if YYSTACK_USE_ALLOCA
#  define YYSTACK_ALLOC alloca
# else
#  ifndef YYSTACK_USE_ALLOCA
#   if defined (alloca) || defined (_ALLOCA_H)
#    define YYSTACK_ALLOC alloca
#   else
#    ifdef __GNUC__
#     define YYSTACK_ALLOC __builtin_alloca
#    endif
#   endif
#  endif
# endif

# ifdef YYSTACK_ALLOC
   /* Pacify GCC's `empty if-body' warning. */
#  define YYSTACK_FREE(Ptr) do { /* empty */; } while (0)
# else
#  if defined (__STDC__) || defined (__cplusplus)
#   include <stdlib.h> /* INFRINGES ON USER NAME SPACE */
#   define YYSIZE_T size_t
#  endif
#  define YYSTACK_ALLOC malloc
#  define YYSTACK_FREE free
# endif
#endif /* ! defined (yyoverflow) || defined (YYERROR_VERBOSE) */


#if (! defined (yyoverflow) \
     && (! defined (__cplusplus) \
	 || (YYLTYPE_IS_TRIVIAL && YYSTYPE_IS_TRIVIAL)))

/* A type that is properly aligned for any stack member.  */
union yyalloc
{
  short yyss;
  YYSTYPE yyvs;
# if YYLSP_NEEDED
  YYLTYPE yyls;
# endif
};

/* The size of the maximum gap between one aligned stack and the next.  */
# define YYSTACK_GAP_MAX (sizeof (union yyalloc) - 1)

/* The size of an array large to enough to hold all stacks, each with
   N elements.  */
# if YYLSP_NEEDED
#  define YYSTACK_BYTES(N) \
     ((N) * (sizeof (short) + sizeof (YYSTYPE) + sizeof (YYLTYPE))	\
      + 2 * YYSTACK_GAP_MAX)
# else
#  define YYSTACK_BYTES(N) \
     ((N) * (sizeof (short) + sizeof (YYSTYPE))				\
      + YYSTACK_GAP_MAX)
# endif

/* Copy COUNT objects from FROM to TO.  The source and destination do
   not overlap.  */
# ifndef YYCOPY
#  if 1 < __GNUC__
#   define YYCOPY(To, From, Count) \
      __builtin_memcpy (To, From, (Count) * sizeof (*(From)))
#  else
#   define YYCOPY(To, From, Count)		\
      do					\
	{					\
	  register YYSIZE_T yyi;		\
	  for (yyi = 0; yyi < (Count); yyi++)	\
	    (To)[yyi] = (From)[yyi];		\
	}					\
      while (0)
#  endif
# endif

/* Relocate STACK from its old location to the new one.  The
   local variables YYSIZE and YYSTACKSIZE give the old and new number of
   elements in the stack, and YYPTR gives the new location of the
   stack.  Advance YYPTR to a properly aligned location for the next
   stack.  */
# define YYSTACK_RELOCATE(Stack)					\
    do									\
      {									\
	YYSIZE_T yynewbytes;						\
	YYCOPY (&yyptr->Stack, Stack, yysize);				\
	Stack = &yyptr->Stack;						\
	yynewbytes = yystacksize * sizeof (*Stack) + YYSTACK_GAP_MAX;	\
	yyptr += yynewbytes / sizeof (*yyptr);				\
      }									\
    while (0)

#endif


#if ! defined (YYSIZE_T) && defined (__SIZE_TYPE__)
# define YYSIZE_T __SIZE_TYPE__
#endif
#if ! defined (YYSIZE_T) && defined (size_t)
# define YYSIZE_T size_t
#endif
#if ! defined (YYSIZE_T)
# if defined (__STDC__) || defined (__cplusplus)
#  include <stddef.h> /* INFRINGES ON USER NAME SPACE */
#  define YYSIZE_T size_t
# endif
#endif
#if ! defined (YYSIZE_T)
# define YYSIZE_T unsigned int
#endif

#define yyerrok		(yyerrstatus = 0)
#define yyclearin	(yychar = YYEMPTY)
#define YYEMPTY		-2
#define YYEOF		0
#define YYACCEPT	goto yyacceptlab
#define YYABORT 	goto yyabortlab
#define YYERROR		goto yyerrlab1
/* Like YYERROR except do call yyerror.  This remains here temporarily
   to ease the transition to the new meaning of YYERROR, for GCC.
   Once GCC version 2 has supplanted version 1, this can go.  */
#define YYFAIL		goto yyerrlab
#define YYRECOVERING()  (!!yyerrstatus)
#define YYBACKUP(Token, Value)					\
do								\
  if (yychar == YYEMPTY && yylen == 1)				\
    {								\
      yychar = (Token);						\
      yylval = (Value);						\
      yychar1 = YYTRANSLATE (yychar);				\
      YYPOPSTACK;						\
      goto yybackup;						\
    }								\
  else								\
    { 								\
      yyerror ("syntax error: cannot back up");			\
      YYERROR;							\
    }								\
while (0)

#define YYTERROR	1
#define YYERRCODE	256


/* YYLLOC_DEFAULT -- Compute the default location (before the actions
   are run).

   When YYLLOC_DEFAULT is run, CURRENT is set the location of the
   first token.  By default, to implement support for ranges, extend
   its range to the last symbol.  */

#ifndef YYLLOC_DEFAULT
# define YYLLOC_DEFAULT(Current, Rhs, N)       	\
   Current.last_line   = Rhs[N].last_line;	\
   Current.last_column = Rhs[N].last_column;
#endif


/* YYLEX -- calling `yylex' with the right arguments.  */

#if YYPURE
# if YYLSP_NEEDED
#  ifdef YYLEX_PARAM
#   define YYLEX		yylex (&yylval, &yylloc, YYLEX_PARAM)
#  else
#   define YYLEX		yylex (&yylval, &yylloc)
#  endif
# else /* !YYLSP_NEEDED */
#  ifdef YYLEX_PARAM
#   define YYLEX		yylex (&yylval, YYLEX_PARAM)
#  else
#   define YYLEX		yylex (&yylval)
#  endif
# endif /* !YYLSP_NEEDED */
#else /* !YYPURE */
# define YYLEX			yylex ()
#endif /* !YYPURE */


/* Enable debugging if requested.  */
#if YYDEBUG

# ifndef YYFPRINTF
#  include <stdio.h> /* INFRINGES ON USER NAME SPACE */
#  define YYFPRINTF fprintf
# endif

# define YYDPRINTF(Args)			\
do {						\
  if (yydebug)					\
    YYFPRINTF Args;				\
} while (0)
/* Nonzero means print parse trace.  It is left uninitialized so that
   multiple parsers can coexist.  */
int yydebug;
#else /* !YYDEBUG */
# define YYDPRINTF(Args)
#endif /* !YYDEBUG */

/* YYINITDEPTH -- initial size of the parser's stacks.  */
#ifndef	YYINITDEPTH
# define YYINITDEPTH 200
#endif

/* YYMAXDEPTH -- maximum size the stacks can grow to (effective only
   if the built-in stack extension method is used).

   Do not make this value too large; the results are undefined if
   SIZE_MAX < YYSTACK_BYTES (YYMAXDEPTH)
   evaluated with infinite-precision integer arithmetic.  */

#if YYMAXDEPTH == 0
# undef YYMAXDEPTH
#endif

#ifndef YYMAXDEPTH
# define YYMAXDEPTH 10000
#endif

#ifdef YYERROR_VERBOSE

# ifndef yystrlen
#  if defined (__GLIBC__) && defined (_STRING_H)
#   define yystrlen strlen
#  else
/* Return the length of YYSTR.  */
static YYSIZE_T
#   if defined (__STDC__) || defined (__cplusplus)
yystrlen (const char *yystr)
#   else
yystrlen (yystr)
     const char *yystr;
#   endif
{
  register const char *yys = yystr;

  while (*yys++ != '\0')
    continue;

  return yys - yystr - 1;
}
#  endif
# endif

# ifndef yystpcpy
#  if defined (__GLIBC__) && defined (_STRING_H) && defined (_GNU_SOURCE)
#   define yystpcpy stpcpy
#  else
/* Copy YYSRC to YYDEST, returning the address of the terminating '\0' in
   YYDEST.  */
static char *
#   if defined (__STDC__) || defined (__cplusplus)
yystpcpy (char *yydest, const char *yysrc)
#   else
yystpcpy (yydest, yysrc)
     char *yydest;
     const char *yysrc;
#   endif
{
  register char *yyd = yydest;
  register const char *yys = yysrc;

  while ((*yyd++ = *yys++) != '\0')
    continue;

  return yyd - 1;
}
#  endif
# endif
#endif

#line 315 "/usr/share/bison/bison.simple"


/* The user can define YYPARSE_PARAM as the name of an argument to be passed
   into yyparse.  The argument should have type void *.
   It should actually point to an object.
   Grammar actions can access the variable by casting it
   to the proper pointer type.  */

#ifdef YYPARSE_PARAM
# if defined (__STDC__) || defined (__cplusplus)
#  define YYPARSE_PARAM_ARG void *YYPARSE_PARAM
#  define YYPARSE_PARAM_DECL
# else
#  define YYPARSE_PARAM_ARG YYPARSE_PARAM
#  define YYPARSE_PARAM_DECL void *YYPARSE_PARAM;
# endif
#else /* !YYPARSE_PARAM */
# define YYPARSE_PARAM_ARG
# define YYPARSE_PARAM_DECL
#endif /* !YYPARSE_PARAM */

/* Prevent warning if -Wstrict-prototypes.  */
#ifdef __GNUC__
# ifdef YYPARSE_PARAM
int yyparse (void *);
# else
int yyparse (void);
# endif
#endif

/* YY_DECL_VARIABLES -- depending whether we use a pure parser,
   variables are global, or local to YYPARSE.  */

#define YY_DECL_NON_LSP_VARIABLES			\
/* The lookahead symbol.  */				\
int yychar;						\
							\
/* The semantic value of the lookahead symbol. */	\
YYSTYPE yylval;						\
							\
/* Number of parse errors so far.  */			\
int yynerrs;

#if YYLSP_NEEDED
# define YY_DECL_VARIABLES			\
YY_DECL_NON_LSP_VARIABLES			\
						\
/* Location data for the lookahead symbol.  */	\
YYLTYPE yylloc;
#else
# define YY_DECL_VARIABLES			\
YY_DECL_NON_LSP_VARIABLES
#endif


/* If nonreentrant, generate the variables here. */

#if !YYPURE
YY_DECL_VARIABLES
#endif  /* !YYPURE */

int
yyparse (YYPARSE_PARAM_ARG)
     YYPARSE_PARAM_DECL
{
  /* If reentrant, generate the variables here. */
#if YYPURE
  YY_DECL_VARIABLES
#endif  /* !YYPURE */

  register int yystate;
  register int yyn;
  int yyresult;
  /* Number of tokens to shift before error messages enabled.  */
  int yyerrstatus;
  /* Lookahead token as an internal (translated) token number.  */
  int yychar1 = 0;

  /* Three stacks and their tools:
     `yyss': related to states,
     `yyvs': related to semantic values,
     `yyls': related to locations.

     Refer to the stacks thru separate pointers, to allow yyoverflow
     to reallocate them elsewhere.  */

  /* The state stack. */
  short	yyssa[YYINITDEPTH];
  short *yyss = yyssa;
  register short *yyssp;

  /* The semantic value stack.  */
  YYSTYPE yyvsa[YYINITDEPTH];
  YYSTYPE *yyvs = yyvsa;
  register YYSTYPE *yyvsp;

#if YYLSP_NEEDED
  /* The location stack.  */
  YYLTYPE yylsa[YYINITDEPTH];
  YYLTYPE *yyls = yylsa;
  YYLTYPE *yylsp;
#endif

#if YYLSP_NEEDED
# define YYPOPSTACK   (yyvsp--, yyssp--, yylsp--)
#else
# define YYPOPSTACK   (yyvsp--, yyssp--)
#endif

  YYSIZE_T yystacksize = YYINITDEPTH;


  /* The variables used to return semantic value and location from the
     action routines.  */
  YYSTYPE yyval;
#if YYLSP_NEEDED
  YYLTYPE yyloc;
#endif

  /* When reducing, the number of symbols on the RHS of the reduced
     rule. */
  int yylen;

  YYDPRINTF ((stderr, "Starting parse\n"));

  yystate = 0;
  yyerrstatus = 0;
  yynerrs = 0;
  yychar = YYEMPTY;		/* Cause a token to be read.  */

  /* Initialize stack pointers.
     Waste one element of value and location stack
     so that they stay on the same level as the state stack.
     The wasted elements are never initialized.  */

  yyssp = yyss;
  yyvsp = yyvs;
#if YYLSP_NEEDED
  yylsp = yyls;
#endif
  goto yysetstate;

/*------------------------------------------------------------.
| yynewstate -- Push a new state, which is found in yystate.  |
`------------------------------------------------------------*/
 yynewstate:
  /* In all cases, when you get here, the value and location stacks
     have just been pushed. so pushing a state here evens the stacks.
     */
  yyssp++;

 yysetstate:
  *yyssp = yystate;

  if (yyssp >= yyss + yystacksize - 1)
    {
      /* Get the current used size of the three stacks, in elements.  */
      YYSIZE_T yysize = yyssp - yyss + 1;

#ifdef yyoverflow
      {
	/* Give user a chance to reallocate the stack. Use copies of
	   these so that the &'s don't force the real ones into
	   memory.  */
	YYSTYPE *yyvs1 = yyvs;
	short *yyss1 = yyss;

	/* Each stack pointer address is followed by the size of the
	   data in use in that stack, in bytes.  */
# if YYLSP_NEEDED
	YYLTYPE *yyls1 = yyls;
	/* This used to be a conditional around just the two extra args,
	   but that might be undefined if yyoverflow is a macro.  */
	yyoverflow ("parser stack overflow",
		    &yyss1, yysize * sizeof (*yyssp),
		    &yyvs1, yysize * sizeof (*yyvsp),
		    &yyls1, yysize * sizeof (*yylsp),
		    &yystacksize);
	yyls = yyls1;
# else
	yyoverflow ("parser stack overflow",
		    &yyss1, yysize * sizeof (*yyssp),
		    &yyvs1, yysize * sizeof (*yyvsp),
		    &yystacksize);
# endif
	yyss = yyss1;
	yyvs = yyvs1;
      }
#else /* no yyoverflow */
# ifndef YYSTACK_RELOCATE
      goto yyoverflowlab;
# else
      /* Extend the stack our own way.  */
      if (yystacksize >= YYMAXDEPTH)
	goto yyoverflowlab;
      yystacksize *= 2;
      if (yystacksize > YYMAXDEPTH)
	yystacksize = YYMAXDEPTH;

      {
	short *yyss1 = yyss;
	union yyalloc *yyptr =
	  (union yyalloc *) YYSTACK_ALLOC (YYSTACK_BYTES (yystacksize));
	if (! yyptr)
	  goto yyoverflowlab;
	YYSTACK_RELOCATE (yyss);
	YYSTACK_RELOCATE (yyvs);
# if YYLSP_NEEDED
	YYSTACK_RELOCATE (yyls);
# endif
# undef YYSTACK_RELOCATE
	if (yyss1 != yyssa)
	  YYSTACK_FREE (yyss1);
      }
# endif
#endif /* no yyoverflow */

      yyssp = yyss + yysize - 1;
      yyvsp = yyvs + yysize - 1;
#if YYLSP_NEEDED
      yylsp = yyls + yysize - 1;
#endif

      YYDPRINTF ((stderr, "Stack size increased to %lu\n",
		  (unsigned long int) yystacksize));

      if (yyssp >= yyss + yystacksize - 1)
	YYABORT;
    }

  YYDPRINTF ((stderr, "Entering state %d\n", yystate));

  goto yybackup;


/*-----------.
| yybackup.  |
`-----------*/
yybackup:

/* Do appropriate processing given the current state.  */
/* Read a lookahead token if we need one and don't already have one.  */
/* yyresume: */

  /* First try to decide what to do without reference to lookahead token.  */

  yyn = yypact[yystate];
  if (yyn == YYFLAG)
    goto yydefault;

  /* Not known => get a lookahead token if don't already have one.  */

  /* yychar is either YYEMPTY or YYEOF
     or a valid token in external form.  */

  if (yychar == YYEMPTY)
    {
      YYDPRINTF ((stderr, "Reading a token: "));
      yychar = YYLEX;
    }

  /* Convert token to internal form (in yychar1) for indexing tables with */

  if (yychar <= 0)		/* This means end of input. */
    {
      yychar1 = 0;
      yychar = YYEOF;		/* Don't call YYLEX any more */

      YYDPRINTF ((stderr, "Now at end of input.\n"));
    }
  else
    {
      yychar1 = YYTRANSLATE (yychar);

#if YYDEBUG
     /* We have to keep this `#if YYDEBUG', since we use variables
	which are defined only if `YYDEBUG' is set.  */
      if (yydebug)
	{
	  YYFPRINTF (stderr, "Next token is %d (%s",
		     yychar, yytname[yychar1]);
	  /* Give the individual parser a way to print the precise
	     meaning of a token, for further debugging info.  */
# ifdef YYPRINT
	  YYPRINT (stderr, yychar, yylval);
# endif
	  YYFPRINTF (stderr, ")\n");
	}
#endif
    }

  yyn += yychar1;
  if (yyn < 0 || yyn > YYLAST || yycheck[yyn] != yychar1)
    goto yydefault;

  yyn = yytable[yyn];

  /* yyn is what to do for this token type in this state.
     Negative => reduce, -yyn is rule number.
     Positive => shift, yyn is new state.
       New state is final state => don't bother to shift,
       just return success.
     0, or most negative number => error.  */

  if (yyn < 0)
    {
      if (yyn == YYFLAG)
	goto yyerrlab;
      yyn = -yyn;
      goto yyreduce;
    }
  else if (yyn == 0)
    goto yyerrlab;

  if (yyn == YYFINAL)
    YYACCEPT;

  /* Shift the lookahead token.  */
  YYDPRINTF ((stderr, "Shifting token %d (%s), ",
	      yychar, yytname[yychar1]));

  /* Discard the token being shifted unless it is eof.  */
  if (yychar != YYEOF)
    yychar = YYEMPTY;

  *++yyvsp = yylval;
#if YYLSP_NEEDED
  *++yylsp = yylloc;
#endif

  /* Count tokens shifted since error; after three, turn off error
     status.  */
  if (yyerrstatus)
    yyerrstatus--;

  yystate = yyn;
  goto yynewstate;


/*-----------------------------------------------------------.
| yydefault -- do the default action for the current state.  |
`-----------------------------------------------------------*/
yydefault:
  yyn = yydefact[yystate];
  if (yyn == 0)
    goto yyerrlab;
  goto yyreduce;


/*-----------------------------.
| yyreduce -- Do a reduction.  |
`-----------------------------*/
yyreduce:
  /* yyn is the number of a rule to reduce with.  */
  yylen = yyr2[yyn];

  /* If YYLEN is nonzero, implement the default value of the action:
     `$$ = $1'.

     Otherwise, the following line sets YYVAL to the semantic value of
     the lookahead token.  This behavior is undocumented and Bison
     users should not rely upon it.  Assigning to YYVAL
     unconditionally makes the parser a bit smaller, and it avoids a
     GCC warning that YYVAL may be used uninitialized.  */
  yyval = yyvsp[1-yylen];

#if YYLSP_NEEDED
  /* Similarly for the default location.  Let the user run additional
     commands if for instance locations are ranges.  */
  yyloc = yylsp[1-yylen];
  YYLLOC_DEFAULT (yyloc, (yylsp - yylen), yylen);
#endif

#if YYDEBUG
  /* We have to keep this `#if YYDEBUG', since we use variables which
     are defined only if `YYDEBUG' is set.  */
  if (yydebug)
    {
      int yyi;

      YYFPRINTF (stderr, "Reducing via rule %d (line %d), ",
		 yyn, yyrline[yyn]);

      /* Print the symbols being reduced, and their result.  */
      for (yyi = yyprhs[yyn]; yyrhs[yyi] > 0; yyi++)
	YYFPRINTF (stderr, "%s ", yytname[yyrhs[yyi]]);
      YYFPRINTF (stderr, " -> %s\n", yytname[yyr1[yyn]]);
    }
#endif

  switch (yyn) {

case 1:
#line 140 "skel.y"
{ __finishSkeleton(); YYACCEPT; ;
    break;}
case 2:
#line 141 "skel.y"
{ __finishSkeleton(); YYACCEPT; ;
    break;}
case 3:
#line 142 "skel.y"
{ __finishSkeleton(); YYACCEPT; ;
    break;}
case 9:
#line 163 "skel.y"
{ __endNode(); ;
    break;}
case 10:
#line 164 "skel.y"
{ __endNode(); ;
    break;}
case 11:
#line 165 "skel.y"
{ __endNode(); ;
    break;}
case 12:
#line 169 "skel.y"
{ __startNode(yyvsp[-2].sValue,yyvsp[0].iValue); ;
    break;}
case 15:
#line 178 "skel.y"
{ __setTransform(yyvsp[0].tValue); ;
    break;}
case 18:
#line 184 "skel.y"
{ yyval.tValue = yyvsp[0].tValue; ;
    break;}
case 19:
#line 185 "skel.y"
{ yyval.tValue = yyvsp[0].tValue; ;
    break;}
case 20:
#line 186 "skel.y"
{ yyval.tValue = yyvsp[0].tValue; ;
    break;}
case 21:
#line 187 "skel.y"
{ yyval.tValue = yyvsp[0].tValue; ;
    break;}
case 22:
#line 188 "skel.y"
{ yyval.tValue = yyvsp[0].tValue; ;
    break;}
case 23:
#line 189 "skel.y"
{ yyval.tValue = yyvsp[0].tValue; ;
    break;}
case 24:
#line 190 "skel.y"
{ yyval.tValue = yyvsp[0].tValue; ;
    break;}
case 25:
#line 191 "skel.y"
{ yyval.tValue = yyvsp[0].tValue; ;
    break;}
case 28:
#line 200 "skel.y"
{ yyval.tValue = __endChain(); ;
    break;}
case 29:
#line 204 "skel.y"
{ __startChain(yyvsp[0].iValue); ;
    break;}
case 30:
#line 208 "skel.y"
{ yyval.tValue = __createTranslate(yyvsp[-1].v3DValue); ;
    break;}
case 31:
#line 212 "skel.y"
{ yyval.tValue = __createTelescope(yyvsp[-3].v3VValue,yyvsp[-1].dofValue); ;
    break;}
case 32:
#line 216 "skel.y"
{ yyval.tValue = __createScale(yyvsp[-1].v3VValue); ;
    break;}
case 34:
#line 224 "skel.y"
{ yyval.tValue = __createRotateEuler(yyvsp[-3].dofValue,yyvsp[-1].iValue); ;
    break;}
case 35:
#line 228 "skel.y"
{ yyval.tValue = __createRotateEuler(yyvsp[-3].dValue,yyvsp[-1].iValue); ;
    break;}
case 39:
#line 241 "skel.y"
{ __recordMass(yyvsp[-3].sValue, yyvsp[-1].dValue); ;
    break;}
case 42:
#line 250 "skel.y"
{ __createHandle(yyvsp[-7].sValue,yyvsp[-5].v3VValue,yyvsp[-3].iValue,yyvsp[-1].sValue); ;
    break;}
case 45:
#line 260 "skel.y"
{ yyval.dofValue = __createDOF(yyvsp[-7].sValue,yyvsp[-5].dValue,yyvsp[-3].dValue,yyvsp[-1].dValue); ;
    break;}
case 46:
#line 261 "skel.y"
{ yyval.dofValue = __createDOF(yyvsp[-9].sValue,yyvsp[-7].dValue,yyvsp[-5].dValue,yyvsp[-3].dValue,yyvsp[-1].dValue); ;
    break;}
case 47:
#line 265 "skel.y"
{ __setPrimitive(yyvsp[-3].v3VValue,yyvsp[-1].v3VValue); ;
    break;}
case 48:
#line 266 "skel.y"
{ __setPrimitive(yyvsp[-5].v3VValue, yyvsp[-3].v3VValue, yyvsp[-1].dofValue); ;
    break;}
case 49:
#line 267 "skel.y"
{ __setPrimitive(yyvsp[-6].v3VValue, yyvsp[-4].v3VValue, yyvsp[-2].dofValue, yyvsp[0].pValue); ;
    break;}
case 50:
#line 271 "skel.y"
{ yyval.pValue = __setGeometry(yyvsp[-5].sValue, yyvsp[-3].sValue, yyvsp[-1].v3VValue); ;
    break;}
case 51:
#line 272 "skel.y"
{ yyval.pValue = __setGeometry(yyvsp[-3].sValue, yyvsp[-1].sValue); ;
    break;}
case 52:
#line 273 "skel.y"
{ yyval.pValue = __setGeometry(yyvsp[-3].sValue, yyvsp[-1].v3VValue); ;
    break;}
case 53:
#line 274 "skel.y"
{ yyval.pValue = __setGeometry(yyvsp[-1].v3VValue); ;
    break;}
case 54:
#line 275 "skel.y"
{ yyval.pValue = __setGeometry(yyvsp[-1].sValue); ;
    break;}
case 55:
#line 279 "skel.y"
{ yyval.v3DValue[0]=yyvsp[-5].dofValue; yyval.v3DValue[1]=yyvsp[-3].dofValue; yyval.v3DValue[2]=yyvsp[-1].dofValue; ;
    break;}
case 56:
#line 283 "skel.y"
{ yyval.v4DValue[0]=yyvsp[-7].dofValue; yyval.v4DValue[1]=yyvsp[-5].dofValue; yyval.v4DValue[2]=yyvsp[-3].dofValue; yyval.v4DValue[3]=yyvsp[-1].dofValue; ;
    break;}
case 57:
#line 287 "skel.y"
{ yyval.v3VValue[0]=yyvsp[-5].dValue; yyval.v3VValue[1]=yyvsp[-3].dValue; yyval.v3VValue[2]=yyvsp[-1].dValue; ;
    break;}
case 58:
#line 291 "skel.y"
{ yyval.dofValue = __createDOF(yyvsp[0].dValue); ;
    break;}
case 59:
#line 292 "skel.y"
{ yyval.dofValue = __createDOF(yyvsp[0].sValue); ;
    break;}
case 60:
#line 296 "skel.y"
{ yyval.iValue = 0; ;
    break;}
case 61:
#line 297 "skel.y"
{ yyval.iValue = 1; ;
    break;}
case 62:
#line 298 "skel.y"
{ yyval.iValue = 2; ;
    break;}
case 63:
#line 302 "skel.y"
{ yyval.dValue = (double)yyvsp[0].iValue; ;
    break;}
case 64:
#line 303 "skel.y"
{ yyval.dValue = yyvsp[0].dValue; ;
    break;}
}

#line 705 "/usr/share/bison/bison.simple"


  yyvsp -= yylen;
  yyssp -= yylen;
#if YYLSP_NEEDED
  yylsp -= yylen;
#endif

#if YYDEBUG
  if (yydebug)
    {
      short *yyssp1 = yyss - 1;
      YYFPRINTF (stderr, "state stack now");
      while (yyssp1 != yyssp)
	YYFPRINTF (stderr, " %d", *++yyssp1);
      YYFPRINTF (stderr, "\n");
    }
#endif

  *++yyvsp = yyval;
#if YYLSP_NEEDED
  *++yylsp = yyloc;
#endif

  /* Now `shift' the result of the reduction.  Determine what state
     that goes to, based on the state we popped back to and the rule
     number reduced by.  */

  yyn = yyr1[yyn];

  yystate = yypgoto[yyn - YYNTBASE] + *yyssp;
  if (yystate >= 0 && yystate <= YYLAST && yycheck[yystate] == *yyssp)
    yystate = yytable[yystate];
  else
    yystate = yydefgoto[yyn - YYNTBASE];

  goto yynewstate;


/*------------------------------------.
| yyerrlab -- here on detecting error |
`------------------------------------*/
yyerrlab:
  /* If not already recovering from an error, report this error.  */
  if (!yyerrstatus)
    {
      ++yynerrs;

#ifdef YYERROR_VERBOSE
      yyn = yypact[yystate];

      if (yyn > YYFLAG && yyn < YYLAST)
	{
	  YYSIZE_T yysize = 0;
	  char *yymsg;
	  int yyx, yycount;

	  yycount = 0;
	  /* Start YYX at -YYN if negative to avoid negative indexes in
	     YYCHECK.  */
	  for (yyx = yyn < 0 ? -yyn : 0;
	       yyx < (int) (sizeof (yytname) / sizeof (char *)); yyx++)
	    if (yycheck[yyx + yyn] == yyx)
	      yysize += yystrlen (yytname[yyx]) + 15, yycount++;
	  yysize += yystrlen ("parse error, unexpected ") + 1;
	  yysize += yystrlen (yytname[YYTRANSLATE (yychar)]);
	  yymsg = (char *) YYSTACK_ALLOC (yysize);
	  if (yymsg != 0)
	    {
	      char *yyp = yystpcpy (yymsg, "parse error, unexpected ");
	      yyp = yystpcpy (yyp, yytname[YYTRANSLATE (yychar)]);

	      if (yycount < 5)
		{
		  yycount = 0;
		  for (yyx = yyn < 0 ? -yyn : 0;
		       yyx < (int) (sizeof (yytname) / sizeof (char *));
		       yyx++)
		    if (yycheck[yyx + yyn] == yyx)
		      {
			const char *yyq = ! yycount ? ", expecting " : " or ";
			yyp = yystpcpy (yyp, yyq);
			yyp = yystpcpy (yyp, yytname[yyx]);
			yycount++;
		      }
		}
	      yyerror (yymsg);
	      YYSTACK_FREE (yymsg);
	    }
	  else
	    yyerror ("parse error; also virtual memory exhausted");
	}
      else
#endif /* defined (YYERROR_VERBOSE) */
	yyerror ("parse error");
    }
  goto yyerrlab1;


/*--------------------------------------------------.
| yyerrlab1 -- error raised explicitly by an action |
`--------------------------------------------------*/
yyerrlab1:
  if (yyerrstatus == 3)
    {
      /* If just tried and failed to reuse lookahead token after an
	 error, discard it.  */

      /* return failure if at end of input */
      if (yychar == YYEOF)
	YYABORT;
      YYDPRINTF ((stderr, "Discarding token %d (%s).\n",
		  yychar, yytname[yychar1]));
      yychar = YYEMPTY;
    }

  /* Else will try to reuse lookahead token after shifting the error
     token.  */

  yyerrstatus = 3;		/* Each real token shifted decrements this */

  goto yyerrhandle;


/*-------------------------------------------------------------------.
| yyerrdefault -- current state does not do anything special for the |
| error token.                                                       |
`-------------------------------------------------------------------*/
yyerrdefault:
#if 0
  /* This is wrong; only states that explicitly want error tokens
     should shift them.  */

  /* If its default is to accept any token, ok.  Otherwise pop it.  */
  yyn = yydefact[yystate];
  if (yyn)
    goto yydefault;
#endif


/*---------------------------------------------------------------.
| yyerrpop -- pop the current state because it cannot handle the |
| error token                                                    |
`---------------------------------------------------------------*/
yyerrpop:
  if (yyssp == yyss)
    YYABORT;
  yyvsp--;
  yystate = *--yyssp;
#if YYLSP_NEEDED
  yylsp--;
#endif

#if YYDEBUG
  if (yydebug)
    {
      short *yyssp1 = yyss - 1;
      YYFPRINTF (stderr, "Error: state stack now");
      while (yyssp1 != yyssp)
	YYFPRINTF (stderr, " %d", *++yyssp1);
      YYFPRINTF (stderr, "\n");
    }
#endif

/*--------------.
| yyerrhandle.  |
`--------------*/
yyerrhandle:
  yyn = yypact[yystate];
  if (yyn == YYFLAG)
    goto yyerrdefault;

  yyn += YYTERROR;
  if (yyn < 0 || yyn > YYLAST || yycheck[yyn] != YYTERROR)
    goto yyerrdefault;

  yyn = yytable[yyn];
  if (yyn < 0)
    {
      if (yyn == YYFLAG)
	goto yyerrpop;
      yyn = -yyn;
      goto yyreduce;
    }
  else if (yyn == 0)
    goto yyerrpop;

  if (yyn == YYFINAL)
    YYACCEPT;

  YYDPRINTF ((stderr, "Shifting error token, "));

  *++yyvsp = yylval;
#if YYLSP_NEEDED
  *++yylsp = yylloc;
#endif

  yystate = yyn;
  goto yynewstate;


/*-------------------------------------.
| yyacceptlab -- YYACCEPT comes here.  |
`-------------------------------------*/
yyacceptlab:
  yyresult = 0;
  goto yyreturn;

/*-----------------------------------.
| yyabortlab -- YYABORT comes here.  |
`-----------------------------------*/
yyabortlab:
  yyresult = 1;
  goto yyreturn;

/*---------------------------------------------.
| yyoverflowab -- parser overflow comes here.  |
`---------------------------------------------*/
yyoverflowlab:
  yyerror ("parser stack overflow");
  yyresult = 2;
  /* Fall through.  */

yyreturn:
#ifndef yyoverflow
  if (yyss != yyssa)
    YYSTACK_FREE (yyss);
#endif
  return yyresult;
}
#line 306 "skel.y"


void __createHandle( char* name, vec3v offset, int id, char* node_name ) {
	TransformNode *node = NULL;

		fprintf(stderr, "   reading handle: %s\n", name);

	for( int i=0;i<num_nodes;i++ ) {
		if( !strcmp(node_lookup[i].name,node_name) ) {
			node = node_lookup[i].node;
		}
	}

	if (node == NULL) {
	   fprintf(stderr, "Parse Error: "
		   "node %s undefined\n", node_name);
		   return;
	}

	int nodeIndex;
	if(node->mIndex == -1)
	  nodeIndex = gSkel->mLimbs.size();
	else
	  nodeIndex = node->mIndex;

	Marker *tempMarker = new Marker(name, 
offset[0],offset[1],offset[2], id, nodeIndex);  
	gSkel->mHandleList.push_back(tempMarker);
	node->AddMarker(tempMarker);
}

Dof* __createDOF( double val ) {
	Dof* d = new Dof("", 0, 0, num_dofs);
	return d;
}

Dof* __createDOF( char* name ) {
	for( int i=0;i<num_dofs;i++ ) {
		if( !strcmp(dof_lookup[i].name,name) ) {
			return dof_lookup[i].dof;
		}
	}
	fprintf(stderr, "Parse Error: "
		"dof %s undefined\n", name);
	return NULL;
}

Dof* __createDOF( char* name, double val, double lo, double hi ) {
	Dof *d = new Dof( name, lo, hi, num_dofs);
	d->mVal = val;

	dof_lookup[num_dofs].name = name;
	dof_lookup[num_dofs].dof = d;
	num_dofs++;

		fprintf(stderr, "   reading dof: %s\n", name);

	char* s = new char[strlen(name) + 1];
	strcpy( s, name );

	return d;
}

Dof* __createDOF( char* name, double val, double lo, double hi, double ease ) {
	Dof *d = new Dof( name, lo, hi, num_dofs);
	d->mVal = val;

	dof_lookup[num_dofs].name = name;
	dof_lookup[num_dofs].dof = d;
	num_dofs++;

		fprintf(stderr, "   reading dof: %s\n", name);

	char* s = new char[strlen(name) + 1];
	strcpy( s, name );

	return d;
}

void __finishSkeleton() {

	gSkel->AddChild(cur_node);
	
	if(cur_node->mPrimitive.size() > 0){
		((TransformNode*)cur_node->mPrimitive[0])->mPrimitive[0]->mIndex = gSkel->mLimbs.size() + LIMBS_MAGIC_NO;
	}else{
		double bone_length = ((Translate*)((TransformNode*)cur_node->mChildren[0])->mTransforms[0])->Get(2) * 2;

		Node *prim = new TransformNode(num_nodes, new Sphere(CLOTH, 0.0), 
								new Scale(bone_length * Vec3d(0.3, 0.3, 1.0)),
								new Translate(vl_0),
								0);

		cur_node->mPrimitive.push_back(prim);

	}

	cur_node->mIndex = gSkel->mLimbs.size();
	gSkel->mLimbs.push_back(cur_node);

}

void __endNode() {

	for( int i = 0; i < num_nodes; i++ ) {
	   if( !strcmp(node_lookup[i].name, cur_node->mName) ) {
	      fprintf(stderr, "Parse Error: "
		      "node multiply assigned to nodes %s,%s\n", 
		      cur_node->mName, node_lookup[i].name);
           }
	}

	node_lookup[num_nodes].name = cur_node->mName;
	node_lookup[num_nodes].node = cur_node;
	num_nodes++;

	if( stack_top > 0 ) {
		if(cur_node->mPrimitive.size() > 0){
			((TransformNode*)cur_node->mPrimitive[0])->mPrimitive[0]->mIndex = gSkel->mLimbs.size() + LIMBS_MAGIC_NO;
		}else{
			Vec3d bone_length = vl_0;
			Vec3d scaleFactor = Vec3d(0.1, 0.1, 0.1);
			Vec3d translateDist = vl_0;
			int  longestIndex = 0;
			if(cur_node->mChildren.size() > 0){
				for(int i = 0; i < 3; i++){
					bone_length[i] = ((Translate*)((TransformNode*)cur_node->mChildren[0])->mTransforms[0])->Get(i);
					if(fabs(bone_length[i]) > fabs(bone_length[longestIndex]))
						longestIndex = i;
				}

				scaleFactor[longestIndex] = 1.0;
				translateDist[longestIndex] = 0.5;
			}else if(strstr(cur_node->mName, "head")){
				bone_length[1] = 0.22;
				longestIndex = 1;
				scaleFactor[longestIndex] = 1.0;
				translateDist[longestIndex] = 0.5;
			}else if(strstr(cur_node->mName, "hand")){
				bone_length[1] = -0.16;
				longestIndex = 1;
				scaleFactor[longestIndex] = 1.0;
				translateDist[longestIndex] = 0.5;
			}else if(strstr(cur_node->mName, "foot")){
				bone_length[0] = -0.16;
				longestIndex = 0;
				scaleFactor[longestIndex] = 1.0;
				translateDist[longestIndex] = 0.5;
			}



			Node *prim = new TransformNode(num_nodes, new Sphere(CLOTH, 0.0), 
									new Scale(fabs(bone_length[longestIndex]) * scaleFactor),
									new Translate(translateDist * bone_length[longestIndex] / fabs(bone_length[longestIndex]) ),
									0);

			cur_node->mPrimitive.push_back(prim);
		}

		cur_node->mIndex = gSkel->mLimbs.size();

		
		if(strstr(cur_node->mName, "dummy") == NULL){
			stack[stack_top-1]->AddChild( cur_node );
			gSkel->mLimbs.push_back(cur_node);
		}

		
		cur_node = stack[--stack_top];
	}

	if(IdStackTop > 0)
		cur_id = IdStack[IdStackTop--];
}

void __startNode( char* s, int id ) {
	TransformNode*		newNode = new TransformNode( s, NULL, NULL );

	fprintf(stderr, "   reading node: %s\n", s);
	if( cur_node != NULL ){
		stack[stack_top++] = cur_node;
	}

	IdStack[++IdStackTop] = id;
	cur_node = newNode;
	cur_id = id;
	cur_node->mCurrentTransform = vl_I;
}

void __setPrimitive( vec3v s, vec3v t ) {

	//Node *prim = new TransformNode( num_nodes, new Sphere(CLOTH, 0.0),
	//						new Scale(bone_length[longestIndex] * Vec3d(s[0],s[1],s[2])),
	//						new Translate(bone_length[longestIndex] * Vec3d(t[0],t[1],t[2])),
	//						0);

//	cur_node->mPrimitive.push_back(prim);
	
}

void __setPrimitive( vec3v s, vec3v t, Dof* bone ) {
	double bone_length = bone->mVal;

	Node *prim = new TransformNode(num_nodes, new Sphere(CLOTH, 0.0), 
							new Scale(bone_length * Vec3d(s[0],s[1],s[2])),
							new Translate(Vec3d(t[0],t[1],t[2])),
							0);

	cur_node->mPrimitive.push_back(prim);
}

void __setPrimitive( vec3v s, vec3v t, Dof* bone, Primitive *geo ) {
	double bone_length = bone->mVal;

	Node *prim = new TransformNode(geo, 
							new Scale(bone_length * Vec3d(s[0],s[1],s[2])),
							new Translate(Vec3d(t[0],t[1],t[2])),
							0);

	cur_node->mPrimitive.push_back(prim);
}


void __setTransform( Transform* t ) {
//	cur_node->mTransforms.push_back(t);	
}

/*
void __setTransform( vector<Transform*> *t ) {
	for(int i = 0; i < t.size(); i++)
		cut_node->mTransforms.push_back(t->at(i));
}
*/

Transform* __endChain() {
	// return the first transform in gChain
	Transform *t = NULL;
	gChain = NULL;
	return t;
}

/*
vector<Transform*>* __endChain() {
	vector<Transform*> *t = gChain;

	gChain = NULL;
	
	return t;
}
*/

void __startChain( int n ) {

}

Transform* __createTranslate( vec3d v ) {
	
	int pos;

	char *underscore = strrchr(v[0]->GetName(), '_');

	if(underscore != NULL){
	  pos = underscore - v[0]->GetName();
	}else{
	  pos = strlen(v[0]->GetName());
	}
	char *commonName = new char[pos + 1];
	strncpy(commonName, v[0]->GetName(), pos);
	commonName[pos] = '\0';

	Translate* trans = new Translate(v[0]->mVal, v[1]->mVal, v[2]->mVal, commonName, gSkel->mDofList, Vec3d(v[0]->mUpperBound, v[1]->mUpperBound, v[2]->mUpperBound));
	
	cur_node->mTransforms.push_back((Transform*)trans);

	if( gChain != NULL )
		gChain->push_back((Transform*)trans);

	return (Transform*)trans;
}

Transform* __createTelescope( vec3v v, Dof* l ) {

	Vec3d value = Vec3d(v[0],v[1],v[2]) * l->mVal;
	Translate* tele = new Translate(value[0],value[1], value[2]);
//	if(cur_id - 1 == IdStack[IdStackTop - 1])
//		((Scale*)((TransformNode*)stack[stack_top - 1]->mPrimitive[0])->mTransforms[0])->MultScale(l->mValue);

	cur_node->mTransforms.push_back((Transform*)tele);

	if( gChain != NULL )
		gChain->push_back((Transform*)tele);

	return (Transform*)tele;
}

Transform* __createScale( vec3v v ) {
	Scale*		scale;

	scale = new Scale(v[0],v[1],v[2]);

	cur_node->mTransforms.push_back((Transform*)scale);
	
	if( gChain != NULL )
		gChain->push_back((Transform*)scale);

	return (Transform*)scale;
}


Transform* __createRotateEuler( Dof* val, int axis ) {
	RotateEuler*	rot;
	
	rot = new RotateEuler(axis,val->mVal, val->mLowerBound, val->mUpperBound, val->GetName(), gSkel->mDofList);
	
	cur_node->mTransforms.push_back((Transform*)rot);

	if( gChain != NULL )
		gChain->push_back((Transform*)rot);

	return (Transform*)rot;
}

Transform* __createRotateEuler( double val, int axis ) {
	RotateEuler*	rot;
	
	rot = new RotateEuler(axis, val);
	
	cur_node->mTransforms.push_back((Transform*)rot);

	if( gChain != NULL )
		gChain->push_back((Transform*)rot);

	return (Transform*)rot;
}

void __recordMass(char* massName, double massValue)
{
	mass_lookup[num_masses].name = massName;
	mass_lookup[num_masses].mass = massValue;
	num_masses++;
}
	
Primitive* __setGeometry(char* shape, char *massName, vec3v color)
{
	Primitive *prim;
	double massValue = 0.0;	
	for( int i = 0; i < num_masses; i++ ) {
	   if( !strcmp(mass_lookup[i].name, massName) )
		 massValue = mass_lookup[i].mass;
	}

	if(!strcmp(shape, "CUBE"))
		prim = new Cube(Vec3d(color[0], color[1], color[2]), massValue);
	else if(!strcmp(shape, "DUMMY"))
		prim = new Dummy(Vec3d(color[0], color[1], color[2]), massValue);
	else
		prim = new Sphere(Vec3d(color[0], color[1], color[2]), massValue);
	
	return prim;
}

Primitive* __setGeometry(char *shape, char *massName)
{
	Primitive *prim;
	double massValue = 0.0;	
	for( int i = 0; i < num_masses; i++ ) {
	   if( !strcmp(mass_lookup[i].name, massName) )
		 massValue = mass_lookup[i].mass;
	}
	if(!strcmp(shape, "CUBE"))
		prim = new Cube(CLOTH, massValue);
	else if(!strcmp(shape, "DUMMY"))
		prim = new Dummy(CLOTH, massValue);
	else
		prim = new Sphere(CLOTH, massValue);	

	return prim;
}

Primitive* __setGeometry(char *string, vec3v color)
{
	Primitive *prim;
	double massValue = 0.0;

	if(!strcmp(string, "CUBE"))
		prim = new Cube(Vec3d(color[0], color[1], color[2]), massValue);
	else if(!strcmp(string, "SPHERE"))
		prim = new Sphere(Vec3d(color[0], color[1], color[2]), massValue);	
	else if(!strcmp(string, "DUMMY"))
		prim = new Dummy(Vec3d(color[0], color[1], color[2]), massValue);	
	else{

	  for( int i = 0; i < num_masses; i++ ) {
	    if( !strcmp(mass_lookup[i].name, string) )
	      massValue = mass_lookup[i].mass;
	  }
	  prim = new Sphere(Vec3d(color[0], color[1], color[2]), massValue);
	}	

	return prim;
}

Primitive* __setGeometry(vec3v color)
{
	Primitive *prim;
	double massValue = 0.0;	

	prim = new Sphere(Vec3d(color[0], color[1], color[2]), massValue);	
	return prim;
}

Primitive* __setGeometry(char *string)
{
	Primitive *prim;
	double massValue = 0.0;

	if(!strcmp(string, "CUBE"))
		prim = new Cube(CLOTH, massValue);
	else if(!strcmp(string, "SPHERE"))
		prim = new Sphere(CLOTH, massValue);
	else if(!strcmp(string, "DUMMY"))
		prim = new Dummy(CLOTH, massValue);
	else{
	  for( int i = 0; i < num_masses; i++ ) {
	    if( !strcmp(mass_lookup[i].name, string) )
	      massValue = mass_lookup[i].mass;
	  }
       
	  prim = new Sphere(CLOTH, massValue);	
	}

	return prim;
}


void yyerror( char* error ) {
	printf( "%s\n", error );
}

Model* readSkelFile( FILE* file, vector<char*>* dofNames ) {
	gSkel = new ArticulatedBody();

	gChain = NULL;

	cur_node = NULL;
	cur_id = 0;
	stack_top = 0;
	IdStackTop = -1;
	num_dofs = 0;
	num_nodes = 0;
	num_masses = 0;

	dNames = dofNames;

	openFile( file );

	//yydebug = 1;
	int error = yyparse();

	//closeFile();
	
	if( error ) {
		delete gSkel;
		return NULL;
	}

	return gSkel;
}

int readSkelFile( FILE* file, ArticulatedBody* skel ) {
	gSkel = skel;
	gChain = NULL;
	cur_node = NULL;
	cur_id = 0;
	stack_top = 0;
	IdStackTop = -1;
	num_dofs = 0;
	num_nodes = 0;
	num_masses = 0;

	openFile( file );

	//yydebug = 1;
	int error = yyparse();

	//closeFile();

	return error;
}

Model* readSkelFile( char* filename, vector<char*>* dofNames ) {
	FILE* file = fopen( filename, "r" );

	if( file == NULL )
		return NULL;

	Model* skel = readSkelFile( file, dofNames );

	fclose(file);

	return skel;
}

