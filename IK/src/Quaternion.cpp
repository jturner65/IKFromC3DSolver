#ifndef __QUATERNION_H__
#include "Quaternion.h"
#endif	//__QUATERNION_H__

#include <FL/gl.h>
#include <assert.h>
#include <string.h>

static double TORADIANS( double degrees ){  return (degrees * 3.14159) / 180.0;}

//build a quaternion from 3 euler angles
Quaternion::Quaternion(double zdeg, double ydeg, double xdeg){
	//if reverse order, change zdeg to xdeg and xdeg to zdeg
	//3-2-1 
	mQuatVal[0] = (cos(xdeg/2.0) * cos(ydeg/2.0) * cos(zdeg/2.0)) + (sin(xdeg/2.0) * sin(ydeg/2.0) * sin(zdeg/2.0));
	mQuatVal[1] = (sin(xdeg/2.0) * cos(ydeg/2.0) * cos(zdeg/2.0)) - (cos(xdeg/2.0) * sin(ydeg/2.0) * sin(zdeg/2.0));
	mQuatVal[2] = (cos(xdeg/2.0) * sin(ydeg/2.0) * cos(zdeg/2.0)) + (sin(xdeg/2.0) * cos(ydeg/2.0) * sin(zdeg/2.0));
	mQuatVal[3] = (cos(xdeg/2.0) * cos(ydeg/2.0) * sin(zdeg/2.0)) - (sin(xdeg/2.0) * sin(ydeg/2.0) * cos(zdeg/2.0));

	normalise(mQuatVal);
	mW = mQuatVal[0];
	mVec = Vec3d(mQuatVal[1],mQuatVal[2],mQuatVal[3]);
	//angle always must be accurate - used for deriv
	mAngle = 2.0 * acos(mQuatVal[0]);
	double sinRes =  sin(mAngle * .5);
	if(sinRes != 0 ) {//means theta is 0 or a multiple of 0 - only way theta/2 will have sin 0, so no rotation
		mAxis = mVec * (1.0 / sin(mAngle * .5));
	} else {//if theta == 0 then no rotation, so doesn't matter
		mAxis = Vec3d(vl_0);
		mAxis[0] = 1;		//arbitrary rotation - no rot needed if angle ==0
	}
}//

//multiply this quat and b -> b x this
Quaternion Quaternion::mult(Quaternion& b){
	Quaternion result = Quaternion(b);
	result.mW = (b.mW * this->mW) - (dot(b.mVec,this->mVec));
	result.mVec = cross(b.mVec, this->mVec);
	result.mVec += (b.mW * this->mVec);
	result.mVec += (this->mW * b.mVec);

	result.mQuatVal[0] = result.mW;
	result.mQuatVal[1] = result.mVec[0];
	result.mQuatVal[2] = result.mVec[1];
	result.mQuatVal[3] = result.mVec[2];
	normalise(mQuatVal);
	result.mW      = result.mQuatVal[0];
	result.mVec[0] = result.mQuatVal[1];
	result.mVec[1] = result.mQuatVal[2];
	result.mVec[2] = result.mQuatVal[3];

	//angle always must be accurate - used for deriv
	result.mAngle = 2.0 * acos(result.mQuatVal[0]);
	double sinRes =  sin(result.mAngle * .5);
	if(sinRes != 0 ) {//means theta is 0 or a multiple of 0 - only way theta/2 will have sin 0, so no rotation
		result.mAxis = result.mVec * (1.0 / sin(result.mAngle * .5));
	} else {//if theta == 0 then no rotation, so doesn't matter
		result.mAxis = Vec3d(vl_0);
		result.mAxis[0] = 1;		//arbitrary rotation - no rot needed if angle ==0
	}

	result.normaliseQ();
	return result;
}//mult


void Quaternion::Apply()//need to have mAxis calculated by this point
{
  glRotated(180.0 * mAngle / 3.14159, mAxis[0], mAxis[1], mAxis[2] ); 
}

void Quaternion::buildQuatMat(Vec4d& _mQ){
	normalise(_mQ);//must be normalized to preserve resulting rotational matrix integrity

	//row major (m[row][col]) - eq from slides
	//row 0
	quatMat[0][0] = 1 - (2 * _mQ[2] * _mQ[2]) - (2 * _mQ[3] * _mQ[3]);
	quatMat[0][1] = (2 * _mQ[1] * _mQ[2]) + (2 * _mQ[0] * _mQ[3]);
	quatMat[0][2] = (2 * _mQ[1] * _mQ[3]) - (2 * _mQ[0] * _mQ[2]);
	quatMat[0][3] = 0.0;

	//row 1
	quatMat[1][0] = (2 * _mQ[1] * _mQ[2]) - (2 * _mQ[0] * _mQ[3]);
	quatMat[1][1] = 1 - (2 * _mQ[3] * _mQ[3]) - (2 * _mQ[1] * _mQ[1]);
	quatMat[1][2] = (2 * _mQ[2] * _mQ[3]) + (2 * _mQ[0] * _mQ[1]);
	quatMat[1][3] = 0.0;

	//row 2
	quatMat[2][0] = (2 * _mQ[1] * _mQ[3]) + (2 * _mQ[0] * _mQ[2]);
	quatMat[2][1] = (2 * _mQ[2] * _mQ[3]) - (2 * _mQ[0] * _mQ[1]);
	quatMat[2][2] = 1 - (2 * _mQ[1] * _mQ[1]) - (2 * _mQ[2] * _mQ[2]);
	quatMat[2][3] = 0.0;
	
	//row 3
	quatMat[3][0] = 0.0;
	quatMat[3][1] = 0.0;
	quatMat[3][2] = 0.0;
	quatMat[3][3] = 1.0; 
}//buildQuatMat


//find the index of the largest value to use in conversion



//build quaternion vector from passed 4x4 matrix
void Quaternion::buildQuatVal(Mat4d& _mQmat){
	//first want to make sure we use term that is furthest from zero
//	int idx = 

}


Mat4d Quaternion::GetTransform(){//treat this like a rotation matrix in transformation space
	//mQuatVal always should be normalized by here
	normalise(mQuatVal);
	Mat4d m(vl_I);		//init as identity
	//row major (m[row][col]) - eq from slides
	//row 0
	m[0][0] = 1 - (2 * mQuatVal[2] * mQuatVal[2]) - (2 * mQuatVal[3] * mQuatVal[3]);
	m[0][1] = (2 * mQuatVal[1] * mQuatVal[2]) + (2 * mQuatVal[0] * mQuatVal[3]);
	m[0][2] = (2 * mQuatVal[1] * mQuatVal[3]) - (2 * mQuatVal[0] * mQuatVal[2]);

	//row 1
	m[1][0] = (2 * mQuatVal[1] * mQuatVal[2]) - (2 * mQuatVal[0] * mQuatVal[3]);
	m[1][1] = 1 - (2 * mQuatVal[3] * mQuatVal[3]) - (2 * mQuatVal[1] * mQuatVal[1]);
	m[1][2] = (2 * mQuatVal[2] * mQuatVal[3]) + (2 * mQuatVal[0] * mQuatVal[1]);

	//row 2
	m[2][0] = (2 * mQuatVal[1] * mQuatVal[3]) + (2 * mQuatVal[0] * mQuatVal[2]);
	m[2][1] = (2 * mQuatVal[2] * mQuatVal[3]) - (2 * mQuatVal[0] * mQuatVal[1]);
	m[2][2] = 1 - (2 * mQuatVal[1] * mQuatVal[1]) - (2 * mQuatVal[2] * mQuatVal[2]);

	//row 3

	return m;}//GetTransform

//quat has 4 dof - take deriv for each degree of freedom
Mat4d Quaternion::GetDeriv(int dof){
	Mat4d m(vl_0);
	//mQuatVal always should be normalized by here
	normalise(mQuatVal);
	switch(dof){
	case 0: {//w/respect to w - idx 0 - no diag terms
		//row 0
		m[0][1] = (2 * mQuatVal[3]);
		m[0][2] = - (2 * mQuatVal[2]);

		//row 1
		m[1][0] = - (2 * mQuatVal[3]);
		m[1][2] = (2 *  mQuatVal[1]);

		//row 2
		m[2][0] = (2 * mQuatVal[2]);
		m[2][1] = -(2 * mQuatVal[1]);

		//row 3 all 0
		return m;}
	case  1: {//w/respect to x - idx 1
		//row 0
		m[0][1] = (2 * mQuatVal[2]);
		m[0][2] = (2 * mQuatVal[3]);

		//row 1
		m[1][0] = (2 * mQuatVal[2]);
		m[1][1] = -(4 * mQuatVal[1]);
		m[1][2] =  (2 * mQuatVal[0]);

		//row 2
		m[2][0] = (2 * mQuatVal[3]);
		m[2][1] = - (2 * mQuatVal[0]);
		m[2][2] = - (4 * mQuatVal[1]);

		//row 3 all 0
		return m;}
	case  2: {//w/respect to y - idx 2
		//row 0
		m[0][0] = - (4 * mQuatVal[2]);
		m[0][1] = (2 * mQuatVal[1]);
		m[0][2] = - (2 * mQuatVal[0]);

		//row 1
		m[1][0] = (2 * mQuatVal[1]); 
		m[1][2] = (2 * mQuatVal[3]);

		//row 2
		m[2][0] = (2 * mQuatVal[0]);
		m[2][1] = (2 * mQuatVal[3]);
		m[2][2] = - (4 * mQuatVal[2]);

		//row 3 all 0
		return m;}
	case  3: {//w/respect to z - idx 3
		//row 0
		m[0][0] = - (4 * mQuatVal[3]);
		m[0][1] = (2 * mQuatVal[0]);
		m[0][2] = (2 * mQuatVal[1]);

		//row 1
		m[1][0] = - (2 * mQuatVal[0]);
		m[1][1] = - (4 * mQuatVal[3]);
		m[1][2] = (2 * mQuatVal[2]);

		//row 2
		m[2][0] = (2 * mQuatVal[1]);
		m[2][1] = (2 * mQuatVal[2]);

		//row 3 all 0
		return m;}
	}//switch
	return m;
}

