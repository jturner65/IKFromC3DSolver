#ifndef	__SCALE_H__
#include "Scale.h"
#endif	//__SCALE_H__

#include <assert.h>
#include <string.h>

Scale::Scale( double x, double y, double z, char* name, DofList& dofs, double low_range_scalar, double high_range_scalar )
{
  mIndex = 2;
  mBone = 1.0;
  MakeDofs( name, dofs, low_range_scalar, high_range_scalar );
}

Scale::Scale( istream& in, DofList* dofs )
{
  mIndex = 2;
  mBone = 1.0;
  if (dofs){
    char buf[1024];
    in >> buf;
    assert(in);
    char* name = new char[strlen(buf)+1];
    strcpy( name, buf );
    double x,y,z;
    double low_range_scalar, high_range_scalar;
    in >> x >> y >> z >> low_range_scalar >> high_range_scalar;
    assert(in);
    mScale = Vec3d(x,y,z);
    MakeDofs( name, *dofs, low_range_scalar, high_range_scalar );
  }else{
    ZeroDofs();
    double x,y,z;
    in >> x >> y >> z;
    assert(in);
    mScale = Vec3d(x,y,z);
  }
}

void Scale::MakeDofs( char* name, DofList& dofs, double low_range_scalar, double high_range_scalar )
{
  int name_len = 3+strlen( name );

  double x = mScale[0];
  double y = mScale[1];
  double z = mScale[2];

  char* nm = new char[name_len];
  sprintf( nm, "%s_x", name );
  mDofs[0] = new ScaleDof( nm, x*low_range_scalar, x*high_range_scalar, 0, dofs.mDofs.size(),  *this );
  dofs.AddDof( mDofs[0] );

  nm = new char[name_len];
  sprintf( nm, "%s_y", name );
  mDofs[1] = new ScaleDof( nm, y*low_range_scalar, y*high_range_scalar, 1, dofs.mDofs.size(),*this);
  dofs.AddDof( mDofs[1] );

  nm = new char[name_len];
  sprintf( nm, "%s_z", name );
  mDofs[2] = new ScaleDof( nm, z*low_range_scalar, z*high_range_scalar, 2, dofs.mDofs.size(), *this );
  dofs.AddDof( mDofs[2] );
}

Mat4d Scale::GetTransform()
{	
  Mat4d m = vl_I;
  m[0][0] = mScale[0] * mBone;
  m[1][1] = mScale[1] * mBone;
  m[2][2] = mScale[2] * mBone;
  m[3][3] = 1;
  return m;
}

Mat4d Scale::GetDeriv(int dof)
{	
  Mat4d m = vl_0;
  m[dof][dof] = mScale[dof];
  //m[3][3] = 1;
  return m;
}



