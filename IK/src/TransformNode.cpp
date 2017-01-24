#ifndef	__TRANSFORMNODE_H__
#include "TransformNode.h"
#endif	//__TRANSFORMNODE_H__

#ifndef __TRANSFORM_H__
#include "Transform.h"
#endif	//__TRANSFORM_H__

#ifndef __GLPRIMITIVES_H__
#include "GLPrimitives.h"
#endif	//__GLPRIMITIVES_H__


#include <FL/gl.h>
#include <assert.h>

TransformNode::TransformNode(char* name)
{
  mName = name;
  mIndex = -1;

  mCurrentTransform = vl_I;
}

TransformNode::TransformNode( char* name, Node* child,Transform* t1 ... )
{
  mName = name;
  mIndex = -1;
  if ( child ){
    //mChildren.push_back( child );
    mPrimitive.push_back(child);
  }

  if(t1){
    mTransforms.push_back( t1 );
    va_list ap;
    va_start( ap, t1 );
    while(1){
      Transform* t = va_arg( ap, Transform* );      
      if(t)
	mTransforms.push_back( t );
      else
	break;
    }
    va_end(ap);
  }
  mCurrentTransform = vl_I;
}

TransformNode::TransformNode( Node* child, Transform* t1 ... )
{
  mName = 0;
  mIndex = -1;
  mPrimitive.push_back(child);

  if (t1){
    mTransforms.push_back( t1 );
    va_list ap;
    va_start( ap, t1 );
    while(1){
      Transform* t = va_arg(ap, Transform*);
      if(t)
	mTransforms.push_back(t);
      else
	break;
    }
    va_end( ap );
  }
  mCurrentTransform = vl_I;
}

TransformNode::TransformNode( int index, Node* child, Transform* t1 ... )
{
  mName = 0;
  mIndex = index;
  mPrimitive.push_back(child);

  if(t1){
    mTransforms.push_back(t1);
    va_list ap;
    va_start( ap, t1 );
    while(1){
      Transform* t = va_arg( ap, Transform* );
      if(t)
	mTransforms.push_back( t );
      else
	break;
    }
    va_end(ap);
  }
  mCurrentTransform = vl_I;
}

void TransformNode::Draw()
{
  glPushMatrix();
	
  for( int i = 0; i < mTransforms.size(); i++ )
    mTransforms[i]->Apply();
 
  for(int i = 0; i < mPrimitive.size(); i++){
    mPrimitive[i]->Draw();
  }
  
  for(int i = 0; i < mChildren.size(); i++ )
    mChildren[i]->Draw();

  glPopMatrix();
}	

void TransformNode::Draw(int frameNum)
{
  glPushMatrix();
	
  for( int i = 0; i < mTransforms.size(); i++ )
    mTransforms[i]->Apply();
	
  for(int i=0;i<mPrimitive.size();i++){
    mPrimitive[i]->Draw();
  }

  for(int i = 0; i < mChildren.size(); i++ )
    ((TransformNode*)mChildren[i])->Draw(frameNum);

  glPopMatrix();
}	

void TransformNode::DrawHandles()
{	
    for(int i = 0; i < mHandles.size(); i++)
        mHandles[i]->Draw();

    for(int i = 0; i < mChildren.size(); i++ )
        ((TransformNode*)mChildren[i])->DrawHandles();
}	

TransformNode::~TransformNode()
{
  int size = mTransforms.size();
  for(int i = 0; i < size; i++)
    delete mTransforms[i];
	
  size = mPrimitive.size();
  for(int i = 0; i < size; i++)
    delete mPrimitive[i];

  size = mHandles.size();
  for(int i = 0; i < size; i++)
    delete mHandles[i];
	
}

void TransformNode::UpdateUpMatrix(Mat4d currTransform, Mat4d invHeadMatrix)
{
  mParentTransform = currTransform;
  mLocalTransform = vl_I;
  for( int i = 0; i < mTransforms.size(); i++ )
    mLocalTransform *= mTransforms[i]->GetTransform();  
   
  currTransform *= mLocalTransform;

  mCurrentTransform = currTransform;
  for(int i = 0; i < mHandles.size(); i++){
    Vec3d tempVec3;
    tempVec3 = mHandles[i]->mOffset;

    Vec4d tempVec4 = Vec4d(tempVec3[0], tempVec3[1], tempVec3[2], 1);
    tempVec4 = currTransform * tempVec4;
		
    mHandles[i]->mGlobalPos = Vec3d(tempVec4[0], tempVec4[1], tempVec4[2]);
  }

  for(int i = 0; i < mChildren.size(); i++ )
    mChildren[i]->UpdateUpMatrix(currTransform, invHeadMatrix);

  for(int i = 0; i < mPrimitive.size(); i++ )
    mPrimitive[i]->UpdateUpMatrix(currTransform, invHeadMatrix);  
}
