#include "ArticulatedBody.h"
#include "Transform.h"

#include <vector>
#include <fstream>
#include <assert.h>

using namespace std;

const double EPSILON = 0.0000001;

const Vec4d G(0.0, 9.8, 0.0, 0.0);
 
ArticulatedBody::ArticulatedBody()
{
  mMarkerCount = 0;
  mNodeCount = 0;
  mRoot = NULL;
}

ArticulatedBody::~ArticulatedBody()
{ 
  for(int i = 0; i < mNodeCount; i++)
    delete mLimbs[i];
}

void ArticulatedBody::SetDofs(Matd &frames, int frameNum)
{
  int nDof = GetDofCount();
  Vecd tempVec(nDof);
  for(int i = 0; i < nDof; i++)
    tempVec[i] = frames[frameNum][i];

  mDofList.SetDofs(tempVec);
  Mat4d invHeadMatrix = vl_I;
  for(int i = 0; i < ((TransformNode*)mChildren[0])->mTransforms.size(); i++)
    invHeadMatrix *= ((TransformNode*)mChildren[0])->mTransforms[i]->GetTransform();
	
  invHeadMatrix = inv(invHeadMatrix);
  UpdateUpMatrix(Mat4d(vl_I), invHeadMatrix);
}

void ArticulatedBody::SetDofs(Vecd &Q)
{
  mDofList.SetDofs(Q);
  Mat4d invHeadMatrix = vl_I;
  for(int i = 0; i < ((TransformNode*)mChildren[0])->mTransforms.size(); i++)
    invHeadMatrix *= ((TransformNode*)mChildren[0])->mTransforms[i]->GetTransform();
  invHeadMatrix = inv(invHeadMatrix);

  UpdateUpMatrix(Mat4d(vl_I), invHeadMatrix);
}

void ArticulatedBody::DrawSkeleton()
{
  TransformNode::Draw();
  
  Mat4d invHeadMatrix = vl_I;
  for(int i = 0; i < ((TransformNode*)mChildren[0])->mTransforms.size(); i++)
    invHeadMatrix *= ((TransformNode*)mChildren[0])->mTransforms[i]->GetTransform();
	
  invHeadMatrix = inv(invHeadMatrix);
  UpdateUpMatrix(Mat4d(vl_I), invHeadMatrix);

}

//same as draw skeleton, but without the drawing. heh.  used to update the skeleton once we calculate new values
void ArticulatedBody::UpdateSkeleton(){
  
  Mat4d invHeadMatrix = vl_I;
  for(int i = 0; i < ((TransformNode*)mChildren[0])->mTransforms.size(); i++)
    invHeadMatrix *= ((TransformNode*)mChildren[0])->mTransforms[i]->GetTransform();
	
  invHeadMatrix = inv(invHeadMatrix);
  UpdateUpMatrix(Mat4d(vl_I), invHeadMatrix);

}



void ArticulatedBody::DrawSkeleton(int frameNum)
{
  TransformNode::Draw(frameNum);
}

void ArticulatedBody::DrawVisualization()
{
    TransformNode::DrawHandles();
}

void ArticulatedBody::InitModel()
{
  mMarkerCount = mHandleList.size();	
  mNodeCount = mLimbs.size();
 
  Mat4d invHeadMatrix = vl_I;
  for(int i = 0; i < ((TransformNode*)mChildren[0])->mTransforms.size(); i++)
    invHeadMatrix *= ((TransformNode*)mChildren[0])->mTransforms[i]->GetTransform();
  invHeadMatrix = inv(invHeadMatrix);
  UpdateUpMatrix(vl_I, invHeadMatrix);  

  mRoot = (TransformNode*)mChildren[0];
  
  int nDof = GetDofCount();
  Vecd dofVec;
  dofVec.SetSize(nDof);
  mDofList.GetDofs(&dofVec);

  SetDofs(dofVec);
  mRoot->mParentNode = NULL;
  ParentPointer(mRoot);
}

void ArticulatedBody::ParentPointer(TransformNode *node)
{
    int nChild = node->mChildren.size();
    for(int i = 0; i < nChild; i++){
        ((TransformNode*)node->mChildren[i])->mParentNode = node;
        ParentPointer((TransformNode*)node->mChildren[i]);
    }
}
