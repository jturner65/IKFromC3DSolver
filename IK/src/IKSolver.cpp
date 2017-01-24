#include "IKSolver.h"

using namespace std;

extern RealTimeIKUI *UI;

namespace IKsolverNS{
double IKSolver::calcGq(int frame){
	double Gq = 0.0;
	Vecd Ggrade = gradQ();

	switch(curGFunc){
	case 0 : break;		//no gq
	case 1 : {//move heavy linkages the least

		double totMass = dot(Vecd(vl_1), transNodeMass);
		//cout<<"Tot mass : "<<totMass<<endl;
		totMass = (totMass == 0 ? 1 : totMass);
		for(int incr = 0; incr<numDofs; ++incr){
			Gq += abs(Ggrade[incr]);// * (massVals[incr])/totMass;			
		}
		break;
	}
	case 2 : 
	case 3 : 
	case 4 : 
	default : {//
		break;
		}
	}//switch
	return Gq;
}//calcGq

//calculate the mass moving from every dof
Vecd IKSolver::calcDofMass(){
	vector<TransformNode*> &limbs = UI->mData->mSelectedModel->mLimbs;
	massVals = Vecd(numDofs, vl_0);		//mass of each linkage corresponding to each dof
	int numLimbs = limbs.size();
	transNodeMass = Vecd(numLimbs,vl_0);

	//use magnitude of grad for each element of Ggrade for amount of change
	for(int i=0; i<numLimbs; ++i){
		TransformNode *t = limbs[i];
		vector<Transform*> &transforms = t->mTransforms;
		for(int j=0; j<transforms.size(); ++j){
			Transform* tf = transforms[j];
			if(tf->IsDof()){					
				for(int k=0; k < tf->GetDofCount(); ++k){
					Dof* d = tf->GetDof(k);
					double objMass =  getDofMass(d->mId);
					transNodeMass[i] = objMass;
					massVals[d->mId] = objMass;
				}//k
			}//if dof
		}//for j
	}//for limbs
    return massVals;
}//

double IKSolver::getDofMass(int i){
	//TransformNode* tf = 

    return 0.0;
}

double IKSolver::calcFq(int frame){
	double fq = 0, tmp = 0;
	double gQ = (orig_q == newq) ? 0 : calcGq(frame);
	for(int i=0; i<numHandles; ++i){
		//add the squared distance between a handle and its respective marker
		Vec3d comp = curModel->mOpenedC3dFile->GetMarkerPos(frame, i);
		if (comp == Vec3d(vl_0)) { tmp = 0; }
		else { tmp = weights[i] * weights[i] * calcSqrDistHandleToMocap(frame, i, comp); }	//sqrlen(curModel->mHandleList[i]->mGlobalPos - curModel->mOpenedC3dFile->GetMarkerPos(frame, i));	}
		fq += tmp;
	}
	fq += gQ;
	return fq;
}//calcFq

double IKSolver::calcSqrDistHandleToMocap(int frame, int hndlIDX, Vec3d& comp){
	if(useCC_COM){	return 0;	} //calc dist between newly calculated handles and mocap data
	else {			return sqrlen(curModel->mHandleList[hndlIDX]->mGlobalPos - comp);	}
}//calcSqrDistHandleToMocap

//take mislabeled or out-of-order mocap data and link it to the most likely positions on the skeleton - should be called if numHandles != numMarkers, with the ability to handle other criteria for mismatch
void IKSolver::mapMocapToHandles(){
//	compSolver = IK_ConCompSolver(18,numMarkers,numHandles, curModel);
	
}

void IKSolver::initSolver(Model* _curModel, bool _mcLoaded){
	curModel = _curModel;
	numHandles = curModel->GetHandleCount();
	numDofs = curModel->GetDofCount();
	orig_q = Vecd(numDofs, vl_0);
	newq=Vecd(numDofs, vl_0);
	prtrb = .5;
	minPrtrb = .01;
	eps = .00001;		
	numIters = max(2,(80 - numHandles)/2);
	alphaIters = min(400, 40 + 4*numHandles);
	setMoCapLoaded(_mcLoaded);
	massVals = Vecd(numDofs,vl_0);
	transNodeMass = Vecd(UI->mData->mSelectedModel->mLimbs.size(),vl_0);

	if((moCapLoaded) && (!HandlesMatchMocap())){
		cout<<"Mapping mismatched mocap data to handles : # mocap markers : "<<numMarkers<<" | # skel IK handles : "<<numHandles<<endl;
		useCC_COM = true;
		mapMocapToHandles();
		cout<<"Finished mapping mocap to skel"<<endl;
	} else {
		useCC_COM = false;
	}
	Jt = Matd(numDofs, numHandles * 3, vl_0);					//reset the Jacobian based on the number of dofs and x,y,z for each handle
	C = Vecd(numHandles * 3, vl_0);

	cout<<"num handles : "<<numHandles<<endl;
	cout<<"num markers : "<<numMarkers<<endl;
}

//whether or not the loaded mocap markers matches the loaded skeleton handles - right now only works off # of handles/markers
bool IKSolver::HandlesMatchMocap(){		
	return (numHandles == numMarkers);
}

void IKSolver::setMoCapLoaded(bool _mcLoaded){
	moCapLoaded = _mcLoaded;
	if(moCapLoaded){	
        animDofs.resize(curModel->mOpenedC3dFile->GetFrameCount());	
		numMarkers = curModel->mOpenedC3dFile->GetMarkerCount();
	} 	
	else {				
		animDofs.resize(1);		
		numMarkers = 0;
	}	
}//

void IKSolver::resetSolver(){
	eps = 0;
	prtrb = 0;	
	numIters = 0;	
	numHandles = 0;	
	numDofs = 0;
	alphaIters = 0;
	runSolver = false; 
	moCapLoaded = false;
	playBackAnim = false;
}//initsolver

//executed for each existing handle in skeleton - calculate vector from handle to corresponding mocap data
Vec3d IKSolver::calcVecHandleToMocap(int frame, int hndlIDX, Vec3d& comp){
	if(useCC_COM){	return vl_0;	}		//TODO: 																				//return vector between new handle coms and new marker coms - need same number of handles in new system as in old
	else {	return curModel->mHandleList[hndlIDX]->mGlobalPos - comp; }//Ci vector = handle - constraint
}

//entering this from main.cpp - fq is being calcuated in main, and the loop is being controlled there.  
void IKSolver::Solve(double fq, int frame){//loop removed to main, for external control
	Jt.MakeZero();
	C.MakeZero();
	//Jt = Matd(numDofs, numHandles * 3, vl_0);					//reset the Jacobian based on the number of dofs and x,y,z for each handle
	//C = Vecd(numHandles*3,vl_0);

	for(int i=0; i<numHandles; ++i){
		Vec3d Ci = Vec3d(0,0,0);
		Vec3d comp = curModel->mOpenedC3dFile->GetMarkerPos(frame, i);
		if (comp != Vec3d(vl_0)) {//ignore markers at 0,0,0 as these are void markers
			calcJacobianRow(i);
			Ci = calcVecHandleToMocap(frame, i, comp);//curModel->mHandleList[i]->mGlobalPos - curModel->mOpenedC3dFile->GetMarkerPos(frame, i); //Ci vector = handle - constraint
		}
		C[i*3] = Ci[0];
		C[i*3 + 1] = Ci[1];
		C[i*3 + 2] = Ci[2];
	}

	Vecd partialF = 2.0 * (Jt * C);	
	Vecd q(numDofs);
	curModel->mDofList.GetDofs(&q);//populate dofs vector
	//previous iteration's q
	orig_q = q;

	q -= (minPrtrb * partialF);		//apply purturbed partial to dofs
	double newPrtrb = prtrb;
	//Calculate addaptive timestep to avoid unnecessary Jacobian calculations
	double fqOld = fq;
	bool improving = true;
	newq = q;
	int count = 0;
	while (improving && (count < alphaIters)) {//was 100
		count++;
		curModel->mDofList.SetDofs(q);
		curModel->UpdateSkeleton(); //applyTransforms();
		double fqNew = calcFq(frame);
		if ((fqNew > fqOld) || (fqOld < eps)) {
			if (newPrtrb <= minPrtrb) {
				improving = false;
			//	cout << "f:" << frame << "\tcnt:" << count << "\tfqNew:" << fqNew << "\tfqOld:" << fqOld << "\t:Not Improving at all anymore" << endl;
			}
			else {
				newPrtrb *= .5;
		//		cout << "f:" << frame << "\tcnt:" << count << "\tfqNew:" << fqNew << "\tfqOld:" << fqOld << "\t:Not Improving : decrease new prtrb:" << newPrtrb << endl;
			}
		}		//start with a bigger prtrbation, halve it until it gets smaller than min prtrbation.  hopefully this will speed this process
		else {
			//improved
		//	cout << "f:" << frame << "\tcnt:" << count << "\tfqNew:" << fqNew << "\tfqOld:" << fqOld << "\t:Improving" << endl;
			fqOld = fqNew;
			newq = q;
		}
		if (improving) { q -= (newPrtrb * partialF); }		//try better perturbation
	}
	curModel->mDofList.SetDofs(newq);
	animDofs[frame] = newq;					//save a copy of dof being set for animation
	curModel->UpdateSkeleton(); //applyTransforms();						//need to apply transforms to their nodes, to move skeleton for iteration
	////previous iteration's q
	//oldq = Vecd(q);

	//q -= (minPrtrb * partialF);		//apply purturbed partial to dofs
	//double newPrtrb = prtrb;
	////Calculate addaptive timestep to avoid unnecessary Jacobian calculations
	//double fqOld = fq;
	//bool improving = true;
	//newq = q;
	//int count = 0;
	//while(improving && (count < alphaIters)){//was 100
	//	count++;
	//	curModel->mDofList.SetDofs(q);
	//	curModel->UpdateSkeleton(); //applyTransforms();
	//	double fqNew = calcFq(frame);
	//	if((fqNew > fqOld) || (fqOld < eps)){ 
	//		if(newPrtrb <= minPrtrb){
	//			improving = false;
	//			cout << "f:" << frame << "\tcnt:" << count << "\tfqNew:" << fqNew << "\tfqOld:" << fqOld <<"\t:Not Improving at all anymore"<<endl;
	//		}
	//		else {
	//			newPrtrb *= .5;
	//			cout << "f:" << frame << "\tcnt:" << count << "\tfqNew:" << fqNew << "\tfqOld:" << fqOld << "\t:Not Improving : decrease new prtrb:" <<newPrtrb<<endl;
	//		}
	//	}		//start with a bigger prtrbation, halve it until it gets smaller than min prtrbation.  hopefully this will speed this process
	//	else{
	//		cout << "f:" << frame << "\tcnt:" << count << "\tfqNew:" << fqNew << "\tfqOld:" << fqOld << "\t:Improving" << endl;
	//		fqOld = fqNew;
	//		newq = q;
	//		q -= (newPrtrb * partialF);
	//	}
	//}
	//curModel->mDofList.SetDofs(newq);		
	//animDofs[frame] = newq;					//save a copy of dof being set for animation
	//curModel->UpdateSkeleton(); //applyTransforms();						//need to apply transforms to their nodes, to move skeleton for iteration
}//Solve

//need to update the skeleton
//void IKSolver::applyTransforms(){	curModel->UpdateSkeleton();		}

//plays back recorded dofs settings
void IKSolver::animPlayBack(int frame){
	//check if valid frame in anim dofs ara, if vec at frame is not null and if vec has enough dofs for this model
	if((frame < animDofs.size()) && (animDofs[frame] != NULL) && (animDofs[frame].Elts() == numDofs)){
		curModel->mDofList.SetDofs(animDofs[frame]);
		curModel->UpdateSkeleton(); //applyTransforms();		
	}
}//IKSolver::animPlayBack

//build weight vector - eventually make ui to handle this
void IKSolver::buildWeights(){
	//make initial weights be related to masses of transform nodes that handles are connected to
	weights = Vecd(numHandles, vl_1); //weights array - set to all 1's now - can change to weigh more distant handles from their markers higher than those that are closer
}

//Calculate the Jacobian for the current state (stored in a class variable) - actually builds transpose, so it's per column
void IKSolver::calcJacobianRow(int i){//each row (column of transpose) is a single handle/marker pair
	Marker* handle = curModel->mHandleList[i];
	TransformNode* curNode = curModel->mLimbs[handle->mNodeIndex]; //get the most distal node for this constraint
	Vec4d tail = Vec4d(handle->mOffset,1.0); //set the tail as the handle vector
	while(curNode){//while not past the root
		for(int j = 0; j < curNode->mTransforms.size(); ++j){ //for all transforms related to this node
			Transform* t = curNode->mTransforms[j]; 
			if(t->IsDof()){ //if the transform is a dof
				for(int d = 0; d < t->GetDofCount(); ++d){ //for each dof within the transform
					Mat4d pre(vl_I);
					if(curNode->mParentNode){		pre = curNode->mParentTransform;	}
					Dof* pDof = t->GetDof(d);
					int dofNum = pDof->mId; //which index is this dof in the list
					for(int k = 0; k < curNode->mTransforms.size(); ++k){ //for each transform
						if(k == j){										pre = pre * t->GetDeriv(d);}							//append the appropriate partial
						else{											pre = pre * curNode->mTransforms[k]->GetTransform();}	//append the transform
					}
					Vec4d partialC = weights[i] * pre * tail; 					
					//build jacobian transpose outta da box -- including weightings
					for(int n = 0; n < 3; ++n){					Jt[dofNum][i*3 + n ] = partialC[n];			}
				}//for each dof of transform
			}//if dof
		}//for each transform
		tail = curNode->mLocalTransform * tail;
		curNode = curNode->mParentNode;
	}//end while

}//calcJacobian

//debug method called from command
void IKSolver::solverDebugPrintout(int frame){

	//print dof info
	vector<Dof*> &dofs = UI->mData->mSelectedModel->mDofList.mDofs;
	vector<Marker*> &handles = UI->mData->mSelectedModel->mHandleList;
	C3dFileInfo* c3d = UI->mData->mSelectedModel->mOpenedC3dFile;
	vector<TransformNode*> &limbs = UI->mData->mSelectedModel->mLimbs;
	cout << "mDofList.mDofs:" << endl;
	for(int i=0; i<dofs.size(); i++){
		Dof* d = dofs[i];		
		cout << i << " " << d->mId << " " << d->GetName() << " value: " << d->mVal << " type: " << d->ReturnType() << endl;
	}

	////print out indidual data for dofs
	//int _numDofs = 0;
	//cout << endl << "mlimbs:" << endl;
	//for(int i=0; i<limbs.size(); i++){
	//	TransformNode *t = limbs[i];
	//	cout << i << " " << t->GetName() << " " << endl;
	//	vector<Transform*> &transforms = t->mTransforms;
	//	for(int j=0; j<transforms.size(); j++){
	//		Transform* tf = transforms[j];
	//		cout << "      " << j << " dof: ";
	//		if (tf->IsDof()){					cout <<"true"<<endl; } 
	//		else {								cout<<"false"<<endl;		}
	//		if(tf->IsDof()){
	//			cout << " numDofs: " << tf->GetDofCount();
	//			for(int k=0; k<tf->GetDofCount(); k++){
	//				Dof* d = tf->GetDof(k);
	//					++_numDofs;	
	//				cout << "            " << k << " " << d->GetName() << " ind: " << d->mId << " type: " << d->ReturnType() << endl;
	//			}
	//		}
	//		cout << endl;
	//	}//for j
	//}//for limbs
	//cout<<"Total num dofs : "<<_numDofs<<endl;

	//print out relations between handles and markers
	cout << endl << "handles matching constraints :" << endl;
	for(int i=0; i<handles.size(); i++){
		Marker* h = handles[i];
		double hdist, cdist, h2cdist;
		if(handles.size() > 1){
			hdist = sqrt(sqrlen(handles[(i == handles.size()-1 ? 0 : i+1)]->mGlobalPos - h->mGlobalPos));
			cdist = sqrt(sqrlen(c3d->GetMarkerPos(frame,(i == handles.size()-1 ? 0 : i+1)) - c3d->GetMarkerPos(frame, i)));
		} else {
			hdist = 0;
			cdist = 0;
		}
		h2cdist = len(h->mGlobalPos -c3d->GetMarkerPos(frame, i));
		cout<<endl;
		cout << i << "\t: " << h->mName <<"\t h : "<<h->mGlobalPos<<"\t c : "<<c3d->GetMarkerPos(frame, i)<<endl;
		cout<<endl;
		cout <<"\t"<< i << ": \tdist nxt h:"<<hdist<<"\t dist nxt c: "<<cdist<<"\t%diff : "<<((hdist - cdist)/(hdist !=0 ? hdist : 1) * 100) <<endl;
		cout<<endl;
		cout <<"\t"<< i << ": \tdist from handle to marker : "<<h2cdist<<endl;
		cout<<"___________________________________________________________________________________________"<<endl;
		cout<<endl;
	}

}//solverdebugprintout

void IKSolver::DisplayWeights(){	for(int i=0; i<numHandles; ++i){			cout<<"weights["<<i<<"] = "<<weights[i]<<endl;			}}
}//namespace
