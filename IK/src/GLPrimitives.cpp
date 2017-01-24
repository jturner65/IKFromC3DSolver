
#ifndef __GLPRIMITIVES_H__
#include "GLPrimitives.h"
#endif	//__GLPRIMITIVES_H__


#include <assert.h>
const Vec3d DEFAULT_COLOR(0,1,0);
const double groundZero = 0.0;

///////////////CUBE/////////////////////////////////////////////////////////////

GLenum DRAW_STYLE;

Cube::Cube( istream& in )
{
  in >> mMass;
  assert( in );
  mColor = DEFAULT_COLOR;
}

static void DrawQuad( const Vec3d& v1, const Vec3d& v2, const Vec3d& v3, const Vec3d& v4, const Vec3d& n )
{
  glLineWidth(2.0);
  //	glBegin( GL_LINE_LOOP );
  glBegin(GL_QUADS);
  glNormal3f( n[0], n[1], n[2] );
  glVertex3f( v1[0], v1[1], v1[2] );
  glVertex3f( v2[0], v2[1], v2[2] );
  glVertex3f( v3[0], v3[1], v3[2] );
  glVertex3f( v4[0], v4[1], v4[2] );
  glEnd();
}

void Cube::Draw()
{
  if(!mDisplay)
    return;

  Vec3d v000( -0.5,-0.5,-0.5 );
  Vec3d v001( -0.5,-0.5, 0.5 );
  Vec3d v010( -0.5, 0.5,-0.5 );
  Vec3d v011( -0.5, 0.5, 0.5 );
  Vec3d v100(  0.5,-0.5,-0.5 );
  Vec3d v101(  0.5,-0.5, 0.5 );
  Vec3d v110(  0.5, 0.5,-0.5 );
  Vec3d v111(  0.5, 0.5, 0.5 );


  // top
  glColor3f( mColor[0], mColor[1], mColor[2] );
  DrawQuad( v010, v011, v111, v110, Vec3d( 0,1,0 ) );

  // bottom
  DrawQuad( v000, v001, v101, v100, Vec3d( 0,-1,0 ) );
  //right
  DrawQuad( v100, v101, v111, v110, Vec3d( 1,0,0 ) );
  //left
  DrawQuad( v000, v001, v011, v010, Vec3d( -1,0,0 ) );
  //back
  DrawQuad( v001, v011, v111, v101, Vec3d( 0,0,1 ) );
  //front
  DrawQuad( v000, v010, v110, v100, Vec3d( 0,0,-1 ) );
}


Vec3d Origin::FLOOR_COLOR = Vec3d(1.0 , 1.0, 0.0);
///////////////ORIGIN/////////////////////////////////////////////////////////////
void Origin::Draw()
{

  //draw ground
  glDisable(GL_LIGHTING);
  bool flip = true;
  for(int i = -20; i < 20; i++){
    for(int j = -20; j < 19; j++){
      if(flip == true){
	//	glColor4d(0.80, 0.80, 0.70, 1.0);
	glColor4d(0.42, 0.42, 0.42, 1.0);
	flip = false; 
      }else{
	//	glColor4d(0.84, 0.84, 0.78, 1.0);
	glColor4d(0.5, 0.5, 0.5, 1.0);
	flip = true;
      }
      glBegin(GL_QUADS);
      glNormal3d(0, 1, 0);
      glVertex3d(i, groundZero, j);
      glVertex3d(i, groundZero, j + 1);
      glVertex3d(i + 1, groundZero, j + 1);
      glVertex3d(i + 1, groundZero, j);
      glEnd();
    }
  }
  glEnable(GL_LIGHTING);


}

///////////////SPHERE/////////////////////////////////////////////////////////////

Sphere::Sphere( istream& in )
{
  in >> mMass;
  assert( in );
  mColor = DEFAULT_COLOR;
}

void Sphere::Draw()
{
  if(!mDisplay)
    return;

  int SPHERE_SLICES;
  int SPHERE_STACKS;
  if(DRAW_STYLE == GLU_FILL){
    SPHERE_SLICES = 64;
    SPHERE_STACKS = 64;
  }else{
    SPHERE_SLICES = 8;
    SPHERE_STACKS = 8;
  }

  GLUquadricObj *q;

  glColor3d( mColor[0], mColor[1], mColor[2] );
	
  q = gluNewQuadric();
	
  gluQuadricDrawStyle(q,DRAW_STYLE);
  gluQuadricNormals(q,GLU_SMOOTH);
  gluSphere(q, 0.5, SPHERE_SLICES, SPHERE_STACKS);

  gluDeleteQuadric( q );
}


///////////////CYLINDER/////////////////////////////////////////////////////////////

Cylinder::Cylinder( istream& in )
{
  in >> mMass;
  assert( in );
  mColor = DEFAULT_COLOR;
}

void Cylinder::Draw()
{
  if(!mDisplay)
    return;

  int CYLINDER_SLICES;
  int CYLINDER_STACKS;
  if(DRAW_STYLE == GLU_FILL){
    CYLINDER_SLICES = 64;
    CYLINDER_STACKS = 64;
  }else{
    CYLINDER_SLICES = 8;
    CYLINDER_STACKS = 8;
  }

  GLUquadricObj *q;

  glColor3d( mColor[0], mColor[1], mColor[2] );
	
  q = gluNewQuadric();
	
  gluQuadricDrawStyle(q,DRAW_STYLE);
  gluQuadricNormals(q,GLU_SMOOTH);
  glPushMatrix();
  glTranslated(0.0, 0.0, -0.5);
  gluCylinder(q, 0.5, 0.5, 1.0, CYLINDER_SLICES, CYLINDER_STACKS);
  glPopMatrix();
  gluDeleteQuadric( q );
}


///////////////DUMMY/////////////////////////////////////////////////////////////

Dummy::Dummy( istream& in )
{
  in >> mMass;
  assert( in );
  mColor = DEFAULT_COLOR;
}

void Dummy::Draw()
{
  if(!mDisplay)
    return;

  glColor3d( mColor[0], mColor[1], mColor[2] );

}
