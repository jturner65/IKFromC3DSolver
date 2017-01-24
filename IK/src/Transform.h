#ifndef __TRANSFORM_H__
#define __TRANSFORM_H__

#include "vl/VLd.h"

class Dof;

class Transform
{
 public:
  virtual void Apply() = 0;
  virtual ~Transform() {}
  virtual Mat4d GetTransform(){Mat4d m=vl_zero;return m;} // return a 4x4 affine transformation evaluated at the current dof
  virtual bool IsDof() { return false; } // if this transformation involves dof, returns true; otherwise false
  virtual int GetDofCount() { return 0; } // returns the number of dofs involved in this transformation
  virtual int GetIndex() { return mIndex; } // retursn the type of this transformation (i.e, scale, rotateEuler, or translate)
  virtual Dof* GetDof( int dof ) { return 0; } // returns a pointer to the specified dof
  int mIndex;
  virtual Mat4d GetDeriv(int dof){ Mat4d m=vl_zero; return m;} // TODO: implement this function for each subclass
};

#endif
