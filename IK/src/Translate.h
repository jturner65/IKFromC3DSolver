#ifndef __TRANSLATE_H__
#define __TRANSLATE_H__

#ifndef	__TRANSFORM_H__
#include "Transform.h"
#endif	//__TRANSFORM_H__

#ifndef __DOF_H__
#include "Dof.h"
#endif	//__DOF_H__


class Translate;

class TranslateDof : public Dof
{
 public:
 TranslateDof( char* name, double min, double max, int dim, int id, Translate& translate ) 
   : Dof( name, min, max, id), mDim( dim ), mTranslate( translate ) {}

  virtual double GetTransformValue();
  virtual void SetTransformValue(double value);
  virtual int ReturnType(){
    return 0;
  }

 private:
  int mDim;
  Translate& mTranslate;
};

class Translate : public Transform
{
 public:
 Translate( const Vec3d& offset ) : mOffset( offset ) {
    mIndex = 0;
    ZeroDofs();
  }
 Translate( double x, double y, double z ) : mOffset(x,y,z) { 
    mIndex = 0;
    ZeroDofs();
  }
  Translate( double x, double y, double z, char* name, DofList& dofs, Vec3d range);
  Translate( istream& in, DofList* dofs );
  virtual ~Translate(){};

  // from Transform
  virtual void Apply(); 
  virtual Mat4d GetTransform();
  virtual Mat4d GetDeriv(int dof); //AC
    
  virtual bool IsDof() { return (mDofs[0]!= 0); }
  virtual int GetDofCount() { return 3; }
  virtual Dof* GetDof( int dof ) { return mDofs[dof]; }

  double Get( int index ) { return mOffset[index]; }
  void Set( int index, double value ) { mOffset[index] = value; }
  void Set( const Vec3d& offset ) { mOffset = offset; }

 private:
  void ZeroDofs() { mDofs[0] = 0; mDofs[1] = 0; mDofs[2] = 0; }
  void MakeDofs( char* name, DofList& dofs, Vec3d range);

  TranslateDof* mDofs[3];
  Vec3d mOffset;
        
};

inline double TranslateDof::GetTransformValue(){
  return mTranslate.Get(mDim);
}
inline void TranslateDof::SetTransformValue(double value){
  mTranslate.Set(mDim, value); 
}

#endif
