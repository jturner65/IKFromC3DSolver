#ifndef	__MARKER_H__
#include "Marker.h"
#endif	//__MARKER_H__
 

#include <FL/gl.h>
        
const Vec3d COLOR( 0.4, .7, .4 );
const double SCALE = 0.015;

Marker::Marker( char* name, double x, double y, double z,int markerOrderVal) : Sphere( COLOR, 0.0 ), mName( name ), mOffset(x,y,z) 
{
  mMarkerOrder = markerOrderVal;
  mNodeIndex = -1;
}

Marker::Marker( char* name, double x, double y, double z,int markerOrderVal, int nodeIndex) : Sphere( COLOR, 0.0 ), mName( name ), mOffset(x,y,z) 
{
  mMarkerOrder = markerOrderVal;
  mNodeIndex = nodeIndex;
}


void Marker::Draw()
{
  glPushMatrix();
  glTranslated( mGlobalPos[0], mGlobalPos[1], mGlobalPos[2] );
  glScaled( SCALE, SCALE, SCALE );
  glLoadName(mMarkerOrder);
  Sphere::Draw();
  glLoadName((unsigned int)(-1));
  glPopMatrix();

}
