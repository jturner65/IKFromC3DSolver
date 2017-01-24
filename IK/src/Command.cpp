#ifndef __COMMAND_H__
#include "Command.h"
#endif //__COMMAND_H__

#ifndef __C3DFILEINFO_H__
#include "C3dFileInfo.h"
#endif	//__C3DFILEINFO_H__

#ifndef __ARTICULATEDBODY_H__
#include "ArticulatedBody.h"
#endif	//__ARTICULATEDBODY_H__

#ifndef RealTimeIKui_h
#include "RealTimeIKui.h"
#endif //RealTimeIKui_h

#ifndef __PHYLTERFLGLWINDOW_H__
#include "PhylterGLWindow.h"
#endif	//__PHYLTERFLGLWINDOW_H__

#ifndef	__PHOWARDDATA_H__
#include "PhowardData.h"
#endif

#ifndef __TRANSFORM_H__
#include "Transform.h"
#endif	//__TRANSFORM_H__

#ifndef __IKSOLVER_H__
#include "IKSolver.h" //AC
#endif


int readSkelFile( FILE* file, ArticulatedBody* skel );

extern RealTimeIKUI *UI;

using namespace IKsolverNS;

void LoadModel(void *v)
{
  char *params = (char*)v;
  if(!params){
    params = (char*)fl_file_chooser("Open File?", "{*.skel}", "../IK/src/skels" );
  }

  if(!params)
    return;

  FILE *file = fopen(params, "r");
    
  if(file == NULL){
    cout << "Skel file does not exist" << endl;
    return;
  }

  ArticulatedBody *mod = new ArticulatedBody();
  UI->mData->mModels.push_back(mod);
  UI->mData->mSelectedModel = mod;

  readSkelFile(file, mod);
  UI->CreateDofSliderWindow();
  UI->CreateWeightSliderWindow();

  mod->InitModel();
  UI->mGLWindow->mShowModel = true;
  UI->mShowModel_but->value(1);
  UI->mGLWindow->refresh();
  
  cout << "number of dofs in model: " << UI->mData->mModels[0]->GetDofCount() << endl;
 
  //prep solver for initial run
	extern IKSolver solver;
	//set these as solver variables to minimize lookups

	solver.initSolver(UI->mData->mSelectedModel, false);
	solver.buildWeights();
}

void TerminateSolver(void *v){
	extern IKSolver solver;
    cout << "terminate solver duhduh duh duhduh" << endl;
 	solver.runSolver = false;	
}

void Solution(void *v)
{
	extern IKSolver solver;
	if(solver.moCapLoaded){	solver.runSolver = true;	}
    std::cout<<"solver go!"<<std::endl;
	//solver.solverDebugPrintout(UI->mFrameCounter_cou->value());
}

void AnimPlayback(void *v){
	extern IKSolver solver;
	solver.setPlayBackAnim( !solver.playBackAnim);//toggle playing animation back
}

void SwitchObjFunc(void *v){
	extern IKSolver solver;
	solver.setCurGFunc(solver.curGFunc + 1);
	cout << "switchOBJ: " << solver.curGFunc << endl;
}

void Exit(void *v)
{
  exit(0);
}

void LoadC3d(void *v)
{
  if(!UI->mData->mSelectedModel){
    cout << "Load skeleton first";
    return;
  }
  char *params = (char*)v;
  if(!params){
    params = fl_file_chooser("Open File?", "{*.c3d}", "../IK/src/mocap/" );
  }

  if(!params)
    return;
  
  char *c3dFilename = new char[80];
  
  // load single c3d file
 
  C3dFileInfo *openFile = new C3dFileInfo(params);
  openFile->LoadFile();
  UI->mData->mSelectedModel->mOpenedC3dFile = openFile;
  cout << "number of frames in c3d: " << openFile->GetFrameCount() << endl;

  UI->InitControlPanel();
  UI->mGLWindow->mShowConstraints = true;
  UI->mShowConstr_but->value(1);
  //init solver variables upon loading of mocap data
  extern IKSolver solver;
  solver.setMoCapLoaded(true);
  solver.initSolver(UI->mData->mSelectedModel, true);
}
