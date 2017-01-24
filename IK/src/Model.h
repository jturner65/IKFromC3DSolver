#ifndef __MODEL_H__
#define __MODEL_H__

#ifndef __DOF_H__
#include "Dof.h"
#endif //__DOF_H__

class C3dFileInfo;
class TransformNode;
class Marker;

#define LIMBS_MAGIC_NO	1000

class Model
{
 public:
  Model(){
    mOpenedC3dFile = NULL;
  };
  ~Model(){};

  virtual void SetDofs(Vecd &dQ){};
  virtual void SetDofs(Matd &frames, int frameNum){};

  int GetDofCount(){
    return mDofList.mDofs.size();
  };
  virtual int GetNodeCount(){
    return 0;
  };
  int GetHandleCount(){
    return mHandleList.size();
  };
  
  virtual void DrawSkeleton(){};
  virtual void UpdateSkeleton(){};
  virtual void DrawSkeleton(int frame){};
  virtual void DrawVisualization(){};

  virtual void InitModel(){};

  DofList mDofList; // the list of dofs; updated by SetDofs()
  std::vector<TransformNode*> mLimbs; // the list of body nodes that comprises the skeleton
  std::vector<Marker*> mHandleList; // the list of markers on the skeleton
  C3dFileInfo* mOpenedC3dFile; // the pointer to C3D file
};
#endif
