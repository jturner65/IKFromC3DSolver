#ifndef __ROTATEEULER_H__
#include "RotateEuler.h"
#endif	//__ROTATEEULER_H__

#include <FL/gl.h>
#include <assert.h>
#include <string.h>

extern double sinTable[361];
extern double cosTable[361];

static double TORADIANS( double degrees ){  return (degrees * 3.14159) / 180.0;}

RotateEuler::RotateEuler( int axis, double angle, double min, double max, char* name, DofList& dofs ) : 
  mAxis(0,0,0)
{
  mIndex = 1;
  mAxis[axis] = 1.0;
  mDof = new EulerDof( name, min, max, dofs.mDofs.size(), *this);
  mAngle = angle;

  dofs.AddDof( mDof );
}

RotateEuler::RotateEuler( istream& in, DofList* dofs ) : mAxis(0,0,0)
{
  mIndex = 1;
  if ( dofs )
    {
      char buf[1024];
      in >> buf;
      assert(in);
      char* name = new char[strlen(buf)+1];
      strcpy( name, buf );
      int axis;
      in >> axis;
      assert(in);
      mAxis[axis] = 1.0;
      double min, max;
      in >> mAngle >> min >> max;
      assert(in);
      mAngle = TORADIANS( mAngle );
      mDof = new EulerDof( name, TORADIANS(min), TORADIANS(max), dofs->mDofs.size(), *this );
      dofs->AddDof( mDof );
    }
  else
    {
      int axis;
      in >> axis;
      assert(in);
      mAxis[axis] = 1.0;
      in >> mAngle;
      assert(in);
      mAngle = TORADIANS( mAngle );
      mDof = 0;
    }
}

void RotateEuler::Apply()
{
  glRotated(180.0 * mAngle / 3.14159, mAxis[0], mAxis[1], mAxis[2] ); 
}

Mat4d RotateEuler::GetTransform()
{
  Mat4d m(vl_I);		//initialize as identity
  int axis;
  double cosAngle;
  double sinAngle;
  cosAngle = cos(mAngle);
  sinAngle = sin(mAngle);
  if(mAxis[0])
    axis = 0;
  else if(mAxis[1])
    axis = 1;
  else if(mAxis[2])
    axis  = 2;

  // calculate coefficients in !row! major order
  switch(axis)
    {	case 0:
	  m[1][1] = cosAngle;
	  m[1][2] = -sinAngle;
			
	  m[2][1] = sinAngle;
	  m[2][2] = cosAngle;
	  break;

    case 1:
      m[0][0] = cosAngle;
      m[0][2] = sinAngle;
 
      m[2][0] = -sinAngle; 
      m[2][2] = cosAngle;
      break;
		
    case 2:
      m[0][0] = cosAngle;
      m[0][1] = -sinAngle;
  
      m[1][0] = sinAngle;
      m[1][1] = cosAngle;
      break;
    }
  return m;
}

//AC
Mat4d RotateEuler::GetDeriv(int dummy){
	//dummy is ignored
	Mat4d m(vl_0);			//initialize as empty
	int axis;
	double cosAngle;
	double sinAngle;
	cosAngle = cos(mAngle);
	sinAngle = sin(mAngle);
  if(mAxis[0])
    axis = 0;
  else if(mAxis[1])
    axis = 1;
  else if(mAxis[2])
    axis  = 2;

  // calculate coefficients in !row! major order
  switch(axis){
	case 0: 
		m[1][1] = -sinAngle;
		m[1][2] = -cosAngle;
		m[2][1] = cosAngle;
		m[2][2] = -sinAngle;
		break;
	case  1:
		m[0][0] = -sinAngle;
		m[0][2] = cosAngle;
		m[2][0] = -cosAngle; 
		m[2][2] = -sinAngle;
		break;
	case 2:
		m[0][0] = -sinAngle;
		m[0][1] = -cosAngle;
		m[1][0] = cosAngle;
		m[1][1] = -sinAngle;
		break;
	}
	//m[3][3] = 1.0; //need this at all times
	return m;
}

