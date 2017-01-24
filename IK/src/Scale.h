#ifndef __SCALE_H__
#define __SCALE_H__

#ifndef	__TRANSFORM_H__
#include "Transform.h"
#endif	//__TRANSFORM_H__

#ifndef __DOF_H__
#include "Dof.h"
#endif	//__DOF_H__

#include <iostream>
#include <FL/gl.h>
#include "vl/VLd.h"

using namespace std;

class Scale;

class ScaleDof : public Dof
{
 public:
 ScaleDof( char* name, double min, double max, int dim, int id, Scale& scale ) 
   : Dof( name, min, max, id ), mDim( dim ), mScale( scale ) {}

  virtual double GetTransformValue();
  virtual void SetTransformValue( double value );
  virtual int ReturnType(){
    return 2;
  }

 private:
  int mDim;
  Scale& mScale;
};

class Scale : public Transform
{
 public:
 Scale( const Vec3d& scale ) : mScale( scale ) { 
    mIndex = 2;
    ZeroDofs();
    mBone = 1.0;	
  }
 Scale( double x, double y, double z ) : mScale(x,y,z) { 
    mIndex = 2;
    ZeroDofs();
    mBone = 1.0;
  }
  Scale( double x, double y, double z, char* name, DofList& dofs, double low_range_scalar, double high_range_scalar );
  Scale( istream& in, DofList* dofs );
  virtual ~Scale(){};

  // from Transform
  virtual void Apply() { glScaled( mScale[0], mScale[1], mScale[2] ); }
  virtual Mat4d GetTransform();
  virtual Mat4d GetDeriv(int dof); 
         
  virtual bool IsDof() { return (mDofs[0]!=0); }
  virtual int GetDofCount() { return 3; }
  virtual Dof* GetDof( int dof ) { return mDofs[dof]; }

  double Get( int index ) { return mScale[index]; }
  void Set( int index, double value ) { mScale[index] = value; }
  Vec3d GetScale(){
    return mScale;
  }
		
  void SetBoneLength(double bone){
    mBone = bone;
  }
  double GetBoneLength(){
    return mBone;
  }

 private:
  void MakeDofs( char* name, DofList& dofs, double low_range_scalar, double high_range_scalar );
  void ZeroDofs() { mDofs[0] = 0; mDofs[1] = 0; mDofs[2] = 0; }

  Vec3d mScale;
  ScaleDof* mDofs[3];
  double mBone;
};

inline double ScaleDof::GetTransformValue() { return mScale.Get( mDim ); }
inline void ScaleDof::SetTransformValue( double value ) { mScale.Set( mDim, value ); }

#endif
