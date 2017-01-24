#ifndef __ROTATEEULER_H__
#define __ROTATEEULER_H__

#ifndef	__TRANSFORM_H__
#include "Transform.h"
#endif	//__TRANSFORM_H__

#ifndef __DOF_H__
#include "Dof.h"
#endif	//__DOF_H__

class RotateEuler; 

class EulerDof : public Dof
{
 public:
 EulerDof(char* name, double min, double max, int id, RotateEuler& euler) : Dof(name, min, max, id), mEuler(euler){}

  virtual double GetTransformValue();
  virtual void SetTransformValue(double value);
  virtual int ReturnType(){
    return 1;
  }

  RotateEuler* GetTransform(){
    return &mEuler;
  }

 private:
  RotateEuler& mEuler;
};

class RotateEuler : public Transform
{
 public:
 RotateEuler( const Vec3d& axis, double angle ) :mAxis( axis ), mAngle( angle ), mDof(0) {
    mIndex = 1;
  }
 RotateEuler( int axis, double angle ) : mAxis(0,0,0), mAngle( angle ), mDof(0) { 
    mIndex = 1;
    mAxis[axis] = 1.0;
  }
  RotateEuler( int axis, double angle, double min, double max, char* name, DofList& dofs );
  RotateEuler( istream& in, DofList* dofs );
  virtual ~RotateEuler(){};

  virtual Mat4d GetTransform();
  virtual Mat4d GetDeriv(int d);

  // from Transform
  virtual void Apply();
  virtual bool IsDof() { return (mDof!=0); }
  virtual int GetDofCount() { return 1; }
  virtual Dof* GetDof( int dof ) { return mDof; }

  Vec3d mAxis;
  double mAngle;
  EulerDof* mDof;
};

inline double EulerDof::GetTransformValue() { return mEuler.mAngle; }
inline void EulerDof::SetTransformValue( double value ) { mEuler.mAngle = value; }

#endif
