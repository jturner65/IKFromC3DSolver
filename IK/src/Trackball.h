#ifndef __TRACKBALL_H__
#define __TRACKBALL_H__

#include "vl/VLd.h"
#include <FL/gl.h>
#include <FL/glu.h>

//Trackball Routine

class Trackball
{
	public:
		Trackball(Vec3d center, double radius){mCenter = center;mRadius=radius; mCurrQuat = Vec4d(0, 0, 0, 1);}
		void Ball_Place(Vec3d center, double radius){mCenter = center;mRadius=radius;}
		void Ball_ComputeRotationMatrix(Mat4d &);
		void StartBall(double x, double y);
		void UpdateBall(double x, double y);
		void GetCurrentRotation(double *returnRot);
		void GetCurrentRotation(Mat4d & returnRot);
		void Draw(int, int);
		void SetCenter(Vec3d cent){ mCenter = cent;}
		void SetCurrQuat(Vec4d q){ mCurrQuat = q; }
		double mRadius;
		
	private:	
		void Qt_ToMatrix(Vec4d q, Mat4d &out);
		Vec3d MouseOnSphere(double mouseX, double mouseY);
		Vec4d Qt_FromBallPoints(Vec3d from, Vec3d to);
		Vec4d Qt_Mul(Vec4d qL, Vec4d qR);
		Vec4d Qt_Conj(Vec4d q);
		
		Vec3d mCenter;
		Vec3d mStartPos;
		Vec4d mCurrQuat;
};

#endif
