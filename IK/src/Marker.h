#ifndef __MARKER_H__
#define __MARKER_H__

#ifndef	__GLPRIMITIVES_H__
#include "GLPrimitives.h"
#endif	//__GLPRIMITIVES_H__

class Marker : public Sphere
{
 public:
  Marker(char* name, double x, double y, double z,int markerOrder);
  Marker(char* name, double x, double y, double z,int markerOrder, int nodeIndex);


  virtual void Draw();

  ~Marker(){};
  char* mName;
  int mNodeIndex; // this marker resides on mSelectedModel->mLimbs[mNodeIndex]
  Vec3d mOffset; // the local coordinates of this marker
  int mMarkerOrder; // this marker can be accessed by mSelectedModel->mHandleList[mMarkerOrder]
  Vec3d mGlobalPos; // the world coordinates of this marker; updated by SetDofs()
};

#endif
