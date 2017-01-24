#ifndef __IKSOLVER_H__
#define __IKSOLVER_H__

#include <fstream>
#include "vl/VLd.h"
#include <vector>

#include "Transform.h"
#include "ArticulatedBody.h"
#include "RealTimeIKui.h"
#include "PhowardData.h"
#include "Model.h"
#include "C3dFileInfo.h"
//#include "IK_ConCompSolver.h"

namespace IKsolverNS{
	class IKSolver{
		public:
			IKSolver() : eps(0), prtrb(0), numIters(0), alphaIters(0), numHandles(0), numDofs(0), runSolver(false), moCapLoaded(false), playBackAnim(false), useCC_COM(false){} 
			~IKSolver(){}

			//functions
			void Solve(double fq, int frame);
			void calcJacobianRow(int i); //global Jacobian Solver for 1 row (constraint - actually a column in Jt)
			void buildWeights();	//vector of constraint/handle - based weights, to determine which handle-marker pairs should be considered more important and which are less important

			void setNumHandles(int _num){numHandles = _num;}
			void setCurModel(Model* _curModel){curModel = _curModel;}
			void setNumIters(int _num){numIters = _num;}
			void setPerturb(float _prt){prtrb = _prt;}
			void setEpsVal(float _eps){eps = _eps;}
			void setAlphaIters(int _aNum){alphaIters = _aNum;}
			void setMoCapLoaded(bool _mcLoaded);
			void setPlayBackAnim(bool _pbA){playBackAnim = _pbA;}
			void setCurGFunc(int _g){curGFunc = (_g) % maxGFunc;}
			double getDofMass(int i);
			double calcFq(int frame);

			void mapMocapToHandles();

			void animPlayBack(int frame);
			void initSolver(Model* _curModel, bool _mcLoaded);
			void resetSolver();
			bool HandlesMatchMocap();

			Vecd gradQ() { return Vecd(newq - orig_q); }
			Vecd calcDofMass();
			double calcGq(int frame);

			double calcSqrDistHandleToMocap(int frame, int hndlIDX, Vec3d& comp);
			Vec3d calcVecHandleToMocap(int frame, int hndlIDX, Vec3d& comp);


			void DisplayWeights();	//debug
			void solverDebugPrintout(int frame);//called from command

			//void applyTransforms();//apply modified transforms on model

			//variables
			Matd Jt;			//Transpose Jacobian
			Vecd C;				//constraint vectors - vector from each handle to each marker, in x,y,z
			Vecd weights;		//weights for each constraint
			float eps;			//Epsilon (allowed error term)
			float prtrb;		//perturbation amount - amount we jostle the partial of the objective function
			float minPrtrb;		//minimum perturbation amount we will allow for any single iteration
			int numIters;		//number of iterations to run per draw cycle
			int alphaIters;		//number of iterations to check for optimal adaptive time step
			int numHandles;		//number of handles the model for this solver has
			int numMarkers;		//number of mocap markers
			int numDofs;		//number of dofs for model this solver is solving for
			bool runSolver;		//whether to run the solver or not
			Model* curModel;
			bool moCapLoaded;
			bool playBackAnim;
			int curGFunc;
			static const int maxGFunc = 2;
			Vecd orig_q,newq;
			vector<Vecd> animDofs;			//ptrs to recorded dofs for each frame
			bool useCC_COM;					//whether or not to use the connected component COM algorithm
			Vecd massVals, transNodeMass;

//			IK_ConCompSolver compSolver;	//object to be used to correlate mismatched mocap to skeleton data
	};
}

#endif