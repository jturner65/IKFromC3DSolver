#ifndef __QUATERNION_H__
#define __QUATERNION_H__

#ifndef	__TRANSFORM_H__
#include "Transform.h"
#endif	//__TRANSFORM_H__

#ifndef __DOF_H__
#include "Dof.h"
#endif	//__DOF_H__

class Quaternion; 

class QuatDof : public Dof {

	public:
		QuatDof(char* name, double min, double max, int dim, int id, Quaternion& quat) : Dof(name, min, max, id), mQuat(quat){}
		//cpyCnstrctr
		QuatDof(QuatDof& dof) : Dof(dof.GetName(), dof.mLowerBound, dof.mUpperBound, dof.mId), mDim(dof.mDim), mQuat(dof.mQuat){}

		virtual double GetTransformValue();
		virtual void SetTransformValue(double value);
		virtual int ReturnType(){   return 3; }			//quat dof set to be 3 - 0 is translate, 1 is rotate, 2 is scale

		Quaternion* GetTransform(){	return &mQuat;}

	private:
		int mDim;
		Quaternion& mQuat;
};

class Quaternion : public Transform	{
	public:
		Quaternion( const Vec3d& axis, double angle ) : mAxis( axis ), mAngle( angle ), mQuatVal(1,0,0,0), mW(0), mVec(0,0,0) {
			//this cnstrctr builds a quat rotation around an arbitrary axis in R3
			//calculate quaternion value here using euler formula : exp(theta/2 * r) = cos theta/2 + rsin theta/2, r is vec
			normalise(mAxis);
			buildQuat(mAxis, mAngle);
		}

		Quaternion(double zdeg, double ydeg, double xdeg);

		Quaternion( int axis, double angle ) : mAxis(0,0,0), mAngle( angle ), mQuatVal(1,0,0,0),  mW(0), mVec(0,0,0)  { 
			mAxis[axis] = 1.0;				//in this cnstrctr, int axis value means which axis (x,y,z) is unit, all others are 0
			buildQuat(mAxis, mAngle);
		}
		//cpyCnstrctr
		Quaternion(Quaternion& quat) : mAxis(quat.mAxis), mAngle(quat.mAngle), mQuatVal(quat.mQuatVal),  mW(quat.mW), mVec(quat.mVec){
			for(int i = 0; i < 4; ++i){					mDofs[i] = quat.mDofs[i];		}
		}
		
		virtual ~Quaternion(){};

		Quaternion GetConjugate(){	Quaternion res = Quaternion(mAxis * -1, mAngle);  return res; } //same cos term, negative sin term
		Quaternion GetInverse(){	Quaternion res = GetConjugate();	return res;	}

		void normaliseQ(){		normalise(mQuatVal);				normalise(mAxis);	}

		void buildQuat(Vec3d& _axis, double _angle){
			mW = cos(_angle/2.0);
			mVec = (sin(_angle/2.0) * _axis);
			mQuatVal = Vec4d(mW,mVec[0], mVec[1], mVec[2]);
			normalise(mQuatVal);//keep normalized
			mIndex = 3;
		}//buildQuat

		//convert back and forth between 4vec and mat versions
		void buildQuatMat(Vec4d& mQ);
		void buildQuatVal(Mat4d& _mQmat);		

		//will return index of largest diag computation, to use when building quat from rot mat
		int findIDXlargestDiag(Vec3d& diag);
		
		//multiply this quat and b :=> b x this
		Quaternion mult(Quaternion& b);

		//calculate new w,x,y,z values if angle changed by delTheta
		void CalcNewVals(float delTheta);

		virtual Mat4d GetTransform();
		virtual Mat4d GetDeriv(int dof);

	  // from Transform
		virtual void Apply();
		virtual bool IsDof() { return (mDofs[0]!=0); }
		virtual int GetDofCount() { return 4; }
		virtual Dof* GetDof( int dof ) { return mDofs[dof]; }
		double Get( int index ) { return mQuatVal[index]; }
		void Set( int index, double value ) { mQuatVal[index] = value; }
		void Set( const Vec4d& offset ) { mQuatVal = offset; }

	private:
		void ZeroDofs() { mDofs[0] = 0; mDofs[1] = 0; mDofs[2] = 0; mDofs[3] = 0; }
		void MakeDofs( char* name, DofList& dofs, Vec3d range);

	public : 

		//rotation-related values
		Vec3d mAxis;
		double mAngle;

		//quaternion values
		Vec4d mQuatVal;			//actual quaternion
		double mW;				//mQuatVal[0]
		Vec3d mVec;				//vector of mQuatVal[1],mQuatVal[2],mQuatVal[3]

		Mat4d quatMat;			//matrix form of quaternion

		QuatDof* mDofs[4];

};

inline double QuatDof::GetTransformValue() { return mQuat.mQuatVal[mDim]; }
inline void QuatDof::SetTransformValue( double value ) { mQuat.mQuatVal[mDim] = value; }

#endif