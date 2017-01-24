#ifndef __TRANSFORMNODE_H__
#define __TRANSFORMNODE_H__

#ifndef __NODE_H__
#include "Node.h"
#endif	//__NODE_H__

#ifndef __MARKER_H__
#include "Marker.h"
#endif	//__MARKER_H__

#include <stdarg.h>

class Transform;
class DofList;

class TransformNode : public Node
{
 public:
  TransformNode() : mName(0){
    mCurrentTransform = vl_I; 
  }
  // the other args should be Transform*'s, followed by 0
  TransformNode( Node* child, Transform* t1=0 ... );
  TransformNode( int index, Node* child, Transform* t1=0 ... );
  TransformNode( char* name, Node* child, Transform* t1=0 ... );
  TransformNode(char* name);
		
  void AddChild( Node* child ){ 
    mChildren.push_back(child); 
    std::vector<int> *newVec = new std::vector<int>; 
  }
  void AddMarker(Marker *marker) {mHandles.push_back(marker);}
  void AddTransform( Transform* t ) { mTransforms.push_back( t ); }
  void AddPrimitive( Node* p ) { mPrimitive.push_back( p ); }
  int  GetSize(){return mTransforms.size(); }
  virtual void UpdateUpMatrix(Mat4d currTransform, Mat4d InvHeadMatrix); // update all the transformation chains
  char* GetName() { return mName; }

  // from Node
  virtual void Draw();
  void Draw(int numFrame);
  void DrawHandles();
  virtual ~TransformNode();
  double GetMass(){
    return *mMass;
  }

  char* mName;
  Mat4d mParentTransform;  // transformation chain from the root to the parent node; updated by SetDofs()
  Mat4d mLocalTransform; // local transformations (including fixed translation from the parent node); updated by SetDofs()
  Mat4d mCurrentTransform; // mParentTransform * mLocalTransform; updated by SetDofs()
  std::vector<Transform*> mTransforms; // list of pointers to transformations associated with this node
  std::vector<Node*> mChildren; // list of pointers to children nodes
  std::vector<Node *> mPrimitive; 
  std::vector<Marker *> mHandles; // list of handles on this node
  double *mMass;
  TransformNode *mParentNode; // handy pointer to the parent node
};

#endif
