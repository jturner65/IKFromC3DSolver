#ifndef	__TRANSLATE_H__
#include "Translate.h"
#endif	//__TRANSLATE_H__

#include <assert.h>
#include <FL/gl.h>
#include <string.h>

Translate::Translate( double x, double y, double z, char* name, DofList& dofs, Vec3d range) : mOffset(x,y,z)
{
  mIndex = 0;
  MakeDofs(name, dofs, range);
}

Translate::Translate( istream& in, DofList* dofs )
{
  mIndex = 0;
  if ( dofs )
    {
      char buf[1024];
      in >> buf;
      assert(in);
      char* name = new char[strlen(buf)+1];
      strcpy( name, buf );
      double x,y,z;
      Vec3d range;
      in >> x >> y >> z >> range[0] >> range[1] >> range[2];
      assert(in);
      mOffset = Vec3d(x,y,z);
      MakeDofs( name, *dofs, range);
    }
  else
    {
      ZeroDofs();
      double x,y,z;
      in >> x >> y >> z;
      assert(in);
      mOffset = Vec3d(x,y,z);
    }
}


void Translate::MakeDofs( char* name, DofList& dofs, Vec3d range)
{
  int name_len = 3+strlen( name );

  double x = mOffset[0];
  double y = mOffset[1];
  double z = mOffset[2];

  char* nm = new char[name_len];
  sprintf( nm, "%s_x", name );
  mDofs[0] = new TranslateDof( nm, x - range[0], x + range[0], 0, dofs.mDofs.size(), *this);
  dofs.AddDof( mDofs[0] );

  nm = new char[name_len];
  sprintf( nm, "%s_y", name );
  mDofs[1] = new TranslateDof( nm, y - range[1], y + range[1], 1, dofs.mDofs.size(), *this );
  dofs.AddDof( mDofs[1] );

  nm = new char[name_len];
  sprintf( nm, "%s_z", name );
  mDofs[2] = new TranslateDof( nm, z - range[2], z + range[2], 2, dofs.mDofs.size(), *this );
  dofs.AddDof( mDofs[2] );
}

void Translate::Apply()
{
  glTranslated( mOffset[0], mOffset[1], mOffset[2] );  
}

Mat4d Translate::GetTransform()
{	
  Mat4d m = vl_I;
  m[0][3] = mOffset[0];
  m[1][3] = mOffset[1];
  m[2][3] = mOffset[2];
  m[3][3] = 1;
  return m;
}

Mat4d Translate::GetDeriv(int dof){
	//translate derivative is all 0 except for the dof in question
	Mat4d m(vl_0);
	//m[dof][3] = mOffset[dof];
	m[dof][3] = 1.0;
	return m;
}
