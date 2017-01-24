#ifdef WIN32
#pragma warning( disable : 4786 ) // disable annoying 'identifier greater that 255 characters' warning
#endif

#ifndef __ARTICULATEDBODY_H__
#define __ARTICULATEDBODY_H__

#ifndef __TRANSFORMNODE_H__
#include "TransformNode.h"
#endif	//__TRANSFORMNODE_H__

#ifndef __DOF_H__
#include "Dof.h"
#endif	//__DOF_H__

#ifndef __MODEL_H__
#include "Model.h"
#endif	//__MODEL_H__

#include <vector>
#include "vl/VLd.h"

class Marker;

class ArticulatedBody: public TransformNode, public Model
{		
 public:

  //handy pointers
  TransformNode *mRoot; // the root node of the skeleton; defined in .skel file

 protected:
  int mMarkerCount;
  int mNodeCount;

 public:
  ArticulatedBody();

  ~ArticulatedBody();

  int GetMarkerCount(){
    return mMarkerCount;
  };

  void SetMarkerCount(int m){
    mMarkerCount = m;
  };

  Marker* GetMarker(int i){
    return mHandleList[i];
  };

  virtual int GetNodeCount(){
    return mNodeCount;
  };
	//added by JT to update the skeleton
  virtual void UpdateSkeleton();
  virtual void DrawSkeleton();
  virtual void DrawSkeleton(int frameNum);

  virtual void DrawVisualization();

  virtual void SetDofs(Vecd &dQ);
  virtual void SetDofs(Matd &frames, int frameNum);
		
  virtual void InitModel();
  void ParentPointer(TransformNode *parent);

};

#endif
