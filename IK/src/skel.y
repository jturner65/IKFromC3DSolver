%{

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


const Vec3d CLOTH(0.5, 0.5, 1.0);
ArticulatedBody* gSkel;


TransformNode*	cur_node;
TransformNode*	stack[256];
int		stack_top;

int IdStack[256];
int IdStackTop;
int cur_id;

struct {
	char* name;
	Dof* dof;
} dof_lookup[256];
int num_dofs;

struct {
	char* name;
	TransformNode* node;
} node_lookup[256];
int num_nodes;

struct {
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
Transform* __createRotateExpMap( vec3d );
Transform* __createRotateQuat( vec4d );
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

%}

%union {
	double	dValue;
	int iValue;
	char* sValue;
	vec3d v3DValue;
	vec4d v4DValue;
	vec3v v3VValue;
	Transform* tValue;
	Primitive* pValue;
	Dof* dofValue;
};

%token <dValue> FLOAT
%token <iValue> INTEGER
%token <sValue> STRING
%token PRIMITIVE
%token CHAIN TRANSLATE TELESCOPE SCALE ROTATE_QUAT ROTATE_EXPMAP ROTATE_EULER ROTATE_CONS
%token HANDLE
%token NODE
%token CONST
%token DOFS
%token HANDLES
%token MASS

%type <iValue> axis
%type <dValue> number
%type <v3DValue> dof_vec3
%type <v4DValue> dof_vec4
%type <v3VValue> val_vec3
%type <dofValue> dof dofs dof_val

%type <tValue> transform transforms
%type <tValue> chain
%type <tValue> translate
%type <tValue> telescope
%type <tValue> scale
%type <tValue> rotate_quat
%type <tValue> rotate_euler
%type <tValue> rotate_expmap
%type <tValue> rotate_cons
%type <pValue> geometry


%start skel_file

%%

skel_file:
	dof_list node handle_list							{ __finishSkeleton(); YYACCEPT; }
	| dof_list node										{ __finishSkeleton(); YYACCEPT; }
	| dof_list mass_list node handle_list				{ __finishSkeleton(); YYACCEPT; }
	;

dof_list:
	  DOFS '{' dofs '}'
	;

mass_list:
	  MASS '{' masses '}'
	;

handle_list:
	  HANDLES '{' handles '}'
	;

nodes:
	  node
	| nodes node
	;

node:
	  node_start node_elements nodes '}'				{ __endNode(); }
	| node_start node_elements '}'						{ __endNode(); }
	| node_start nodes '}'								{ __endNode(); }
	;
														
node_start:
	  NODE STRING '{' INTEGER							{ __startNode($2,$4); }
	;

node_elements:
	  node_element
	| node_element node_elements
	;

node_element:
	  transform											{ __setTransform($1); }
	| handle
	| primitive
	;

transform:
	  chain												{ $$ = $1; }
	| translate											{ $$ = $1; }
	| telescope											{ $$ = $1; }
	| scale												{ $$ = $1; }
	| rotate_quat										{ $$ = $1; }
	| rotate_euler										{ $$ = $1; }
	| rotate_cons										{ $$ = $1; }
	| rotate_expmap										{ $$ = $1; }
	;

transforms:
	  transform
	| transforms transform
	;

chain:
	  chain_start transforms '}'						{ $$ = __endChain(); }
	;

chain_start:
	  CHAIN '{' INTEGER									{ __startChain($3); }
	;

translate:
	  TRANSLATE '{' dof_vec3 '}'						{ $$ = __createTranslate($3); }
	;

telescope:
	  TELESCOPE '{' val_vec3 ',' dof_val '}'			{ $$ = __createTelescope($3,$5); }
	;

scale:
	  SCALE '{' val_vec3 '}'							{ $$ = __createScale($3); }
	;

rotate_quat:
	  ROTATE_QUAT '{' dof_vec4 '}'						{ $$ = __createRotateQuat($3); }
	;

rotate_euler:
	  ROTATE_EULER '{' dof_val ',' axis '}'				{ $$ = __createRotateEuler($3,$5); }
	;

rotate_cons:
	  ROTATE_CONS '{' number ',' axis '}'				{ $$ = __createRotateEuler($3,$5); }
	;

rotate_expmap:
	  ROTATE_EXPMAP '{' dof_vec3 '}'				{ $$ = __createRotateExpMap($3); }
	;

masses:
	  mass
	| masses mass
	;

mass:
	  STRING '{' number	'}'								{ __recordMass($1, $3); }
	;

handles:
	  handle
	| handles handle
	;

handle:
	  STRING '{' val_vec3 ',' INTEGER ',' STRING '}'	{ __createHandle($1,$3,$5,$7); }
	;


dofs:
	  dof
	| dofs dof
	;

dof:
	  STRING '{' number ',' number ',' number '}'				{ $$ = __createDOF($1,$3,$5,$7); }
	| STRING '{' number ',' number ',' number ',' number '}'	{ $$ = __createDOF($1,$3,$5,$7,$9); }
	;

primitive:
	  PRIMITIVE '{' val_vec3 ',' val_vec3 '}'								{ __setPrimitive($3,$5); }
	| PRIMITIVE '{' val_vec3 ',' val_vec3 ',' dof_val '}'					{ __setPrimitive($3, $5, $7); }
	| PRIMITIVE '{' val_vec3 ',' val_vec3 ',' dof_val ',' geometry		    { __setPrimitive($3, $5, $7, $9); }
	;

geometry:
	  STRING ',' STRING ',' val_vec3 '}'						{ $$ = __setGeometry($1, $3, $5); }
	| STRING ',' STRING '}'										{ $$ = __setGeometry($1, $3); }
	| STRING ',' val_vec3 '}'									{ $$ = __setGeometry($1, $3); }
	| val_vec3 '}'									{ $$ = __setGeometry($1); }
	| STRING '}'												{ $$ = __setGeometry($1); }			
	;

dof_vec3:
	  '<' dof_val ',' dof_val ',' dof_val '>'			{ $$[0]=$2; $$[1]=$4; $$[2]=$6; }
	;

dof_vec4:
	  '<' dof_val ',' dof_val ',' dof_val ',' dof_val '>'	{ $$[0]=$2; $$[1]=$4; $$[2]=$6; $$[3]=$8; }
	;

val_vec3:
	  '<' number ',' number ',' number '>'				{ $$[0]=$2; $$[1]=$4; $$[2]=$6; }
	;

dof_val:
	  number											{ $$ = __createDOF($1); }
	| STRING											{ $$ = __createDOF($1); }
	;

axis:
	  'x'												{ $$ = 0; }
	| 'y'												{ $$ = 1; }
	| 'z'												{ $$ = 2; }
	;

number:
	  INTEGER											{ $$ = (double)$1; }
	| FLOAT												{ $$ = $1; }
	;

%%

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
	d->mVar->mVal = val;

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
	d->mVar->mVal = val;

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
		double bone_length = ((Translate*)((TransformNode*)cur_node->mChildren[0])->mTransforms[0])->Get(0) * 2;

		Node *prim = new TransformNode(num_nodes, new Sphere(CLOTH, 0.0), 
								new Scale(bone_length * Vec3d(1.0, 0.3, 0.3)),
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
					if(abs(bone_length[i]) > abs(bone_length[longestIndex]))
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
									new Scale(abs(bone_length[longestIndex]) * scaleFactor),
									new Translate(translateDist * bone_length[longestIndex] / abs(bone_length[longestIndex]) ),
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
	cur_node->currentTransform = vl_I;
	cur_node->noGlobalTransform = vl_I;
}

void __setPrimitive( vec3v s, vec3v t ) {
  /*	Node *prim = new TransformNode( num_nodes, new Sphere(CLOTH, 0.0),
							new Scale(Vec3d(s[0],s[1],s[2])),
							new Translate(Vec3d(t[0],t[1],t[2])),
							0);

	cur_node->mPrimitive.push_back(prim);
  */
}

void __setPrimitive( vec3v s, vec3v t, Dof* bone ) {
	double bone_length = bone->mVar->mVal;

	Node *prim = new TransformNode(num_nodes, new Sphere(CLOTH, 0.0), 
							new Scale(bone_length * Vec3d(s[0],s[1],s[2])),
							new Translate(Vec3d(t[0],t[1],t[2])),
							0);

	cur_node->mPrimitive.push_back(prim);
}

void __setPrimitive( vec3v s, vec3v t, Dof* bone, Primitive *geo ) {
	double bone_length = bone->mVar->mVal;

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

	Translate* trans = new Translate(v[0]->mVar->mVal, v[1]->mVar->mVal, v[2]->mVar->mVal, commonName, gSkel->mDofList, Vec3d(v[0]->mVar->mUpperBound, v[1]->mVar->mUpperBound, v[2]->mVar->mUpperBound));
	
	cur_node->mTransforms.push_back((Transform*)trans);

	if( gChain != NULL )
		gChain->push_back((Transform*)trans);

	return (Transform*)trans;
}

Transform* __createTelescope( vec3v v, Dof* l ) {

	Vec3d value = Vec3d(v[0],v[1],v[2]) * l->mVar->mVal;
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

Transform* __createRotateExpMap( vec3d v ) {
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

	RotateExp*	expmap;

	expmap = new RotateExp(v[0]->mVar->mVal, v[1]->mVar->mVal, v[2]->mVar->mVal, Vec3d(v[0]->mVar->mUpperBound, v[1]->mVar->mUpperBound, v[2]->mVar->mUpperBound), Vec3d(v[0]->mVar->mLowerBound, v[1]->mVar->mLowerBound, v[2]->mVar->mLowerBound), commonName, gSkel->mDofList);
	
	cur_node->mTransforms.push_back((Transform*)expmap);

	if( gChain != NULL )
		gChain->push_back((Transform*)expmap);

	return (Transform*)expmap;
}

Transform* __createRotateQuat( vec4d v ) {
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

	RotateQuat*	quat;

	quat = new RotateQuat(v[0]->mVar->mVal, v[1]->mVar->mVal, v[2]->mVar->mVal, v[3]->mVar->mVal, commonName, gSkel->mDofList);
	
	cur_node->mTransforms.push_back((Transform*)quat);

	if( gChain != NULL )
		gChain->push_back((Transform*)quat);

	return (Transform*)quat;
}

Transform* __createRotateEuler( Dof* val, int axis ) {
	RotateEuler*	rot;
	
	rot = new RotateEuler(axis,val->mVar->mVal, val->mVar->mLowerBound, val->mVar->mUpperBound, val->GetName(), gSkel->mDofList);
	
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
	else if(!strcmp(shape, "HEAD"))
		prim = new Head(Vec3d(color[0], color[1], color[2]), 
massValue);
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
	else if(!strcmp(shape, "HEAD"))
		prim = new Head(CLOTH, massValue);
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
	else if(!strcmp(string, "HEAD"))
		prim = new Head(Vec3d(color[0], color[1], color[2]), 
massValue);
	else if(!strcmp(string, "SPHERE"))
		prim = new Sphere(Vec3d(color[0], color[1], color[2]), massValue);	
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
	else if(!strcmp(string, "HEAD"))
		prim = new Head(CLOTH, massValue);
	else if(!strcmp(string, "SPHERE"))
		prim = new Sphere(CLOTH, massValue);
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

