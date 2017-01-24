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

# define	FLOAT	257
# define	INTEGER	258
# define	STRING	259
# define	PRIMITIVE	260
# define	CHAIN	261
# define	TRANSLATE	262
# define	TELESCOPE	263
# define	SCALE	264
# define	ROTATE_QUAT	265
# define	ROTATE_EXPMAP	266
# define	ROTATE_EULER	267
# define	ROTATE_CONS	268
# define	HANDLE	269
# define	NODE	270
# define	CONST	271
# define	DOFS	272
# define	HANDLES	273
# define	MASS	274


extern YYSTYPE yylval;
