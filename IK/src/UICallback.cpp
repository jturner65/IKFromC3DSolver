// Only commands that can be scripted will be processed as command lines

#ifndef __UICALLBACK_H__
#include "UICallback.h"
#endif	//__UICALLBACK_H__

#ifndef __PHYLTERFLGLWINDOW_H__
#include "PhylterGLWindow.h"
#endif	//__PHYLTERFLGLWINDOW_H__

#ifndef __MODEL_H__
#include "Model.h"
#endif	//__HMODEL_H__

#ifndef	__MARKER_H__
#include "Marker.h"
#endif	//__MARKER_H__

#ifndef __COMMAND_H__
#include "Command.h"
#endif //__COMMAND_H__

#ifndef RealTimeIKui_h
#include "RealTimeIKui.h"
#endif //RealTimeIKui_h

#ifndef	__PHOWARDDATA_H__
#include "PhowardData.h"
#endif

#ifndef __IKSOLVER_H__
#include "IKSolver.h" 
#endif

using namespace IKsolverNS;

extern GLenum DRAW_STYLE;
extern RealTimeIKUI *UI;

void LoadModel_cb(Fl_Widget *o, void *v)
{
  LoadModel(NULL);
}

void Motion_cb(Fl_Widget *o, void *v)
{
}

void LoadMocap_cb(Fl_Widget *o, void *v)
{
  LoadC3d(NULL);
}

void SaveMotion_cb(Fl_Widget *o, void *v)
{
}


void Exit_cb(Fl_Widget *o, void *v)
{
  Exit(NULL);
}

void PlayStop_cb(Fl_Widget *o, void *v)
{
  if(UI->mPlay_but->value()){
    UI->mPlay_but->label("STOP");
  }else{
    UI->mPlay_but->label("@>");
  }
}

void Frame_scr_cb(Fl_Widget *o, void *v)
{
  Fl_Value_Slider* slider = (Fl_Value_Slider*)o;
  int currentFrame = int(slider->value());
  UI->mFrameCounter_cou->value(currentFrame);
}

void Speed_cb(Fl_Widget *o, void *v)
{
  UI->mData->mHowFast = UI->mSpeed_rol->value();

}

void FrameCounter_cb(Fl_Widget *o, void *v)
{
  double val = UI->mFrameCounter_cou->value();
  int beginFrame = int(UI->mBegin_sli->value());
  int endFrame = int(UI->mEnd_sli->value());
  bool looping = UI->mLoop_but->value();

  if(val < beginFrame){
    if(looping)
      val = beginFrame;
  }else if( val > endFrame){
    if(looping)
      val = beginFrame;
  }

  // val is unwarped time
  int tw = val;
    
  UI->mFrame_sli->value(val);
  UI->mFrameCounter_cou->value(val);
}

void RecordMotion_cb(Fl_Widget *o, void *v)
{
}

void StillShot_cb(Fl_Widget *o, void *v)
{
}

void ShowModel_cb(Fl_Widget *o, void *v)
{
  if(UI->mData->mModels.size() <= 0){
    UI->mShowModel_but->value(0);
    return;
  }

  int status = ((Fl_Button*)o)->value();
  UI->mGLWindow->mShowModel = status;

  UI->mGLWindow->refresh();

}

void ShowConstr_cb(Fl_Widget *o, void *v)
{
  if(UI->mData->mModels.size() <= 0){
    UI->mShowConstr_but->value(0);
    return;
  }

  int status = ((Fl_Button*)o)->value();
  UI->mGLWindow->mShowConstraints = status;

  UI->mGLWindow->refresh();

}

void ShowViz_cb(Fl_Widget *o, void *v)
{
  if(UI->mData->mModels.size() == 0){
    UI->mShowViz_but->value(0);
    return;
  }
  int status = ((Fl_Button*)o)->value();
  UI->mGLWindow->mShowViz = status;
  if(status){
    DRAW_STYLE = GLU_SILHOUETTE;
  }else{
    DRAW_STYLE = GLU_FILL;
  }
  UI->mGLWindow->refresh();
}

void ShowModel_ite_cb(Fl_Widget *o, void *v)
{
  if(UI->mData->mModels.size() <= 0){
    return;
  }
  int status = !UI->mGLWindow->mShowModel;
  UI->mGLWindow->mShowModel = status;

  UI->mShowModel_but->value(status);
  UI->mGLWindow->refresh();

}

void ShowConstr_ite_cb(Fl_Widget *o, void *v)
{
  if(UI->mData->mModels.size() <= 0){
    return;
  }
  int status = !UI->mGLWindow->mShowConstraints;
  UI->mGLWindow->mShowConstraints = status;

  UI->mShowConstr_but->value(status);
  UI->mGLWindow->refresh();
}

void ShowViz_ite_cb(Fl_Widget *o, void *v)
{
  if(UI->mData->mModels.size() <= 0){
    return;
  }
  int status = !UI->mGLWindow->mShowViz;
  UI->mGLWindow->mShowViz = status;
  UI->mShowViz_but->value(status);
  if(status){
    DRAW_STYLE = GLU_SILHOUETTE;
  }else{
    DRAW_STYLE = GLU_FILL;
  }
  UI->mGLWindow->refresh();
}

void Terminate_cb(Fl_Widget *o, void *v){
  UI->mData->mTermination = true;
	TerminateSolver(NULL);
}

void About_cb(Fl_Widget *o, void *v)
{
}

void PlaybackAnim_cb(Fl_Widget *o, void *v){
	AnimPlayback(NULL);
}

void SwitchObjFunc_cb(Fl_Widget *o, void *v){
	SwitchObjFunc(NULL);
}

void ShowSliders_cb(Fl_Widget *o, void *v)
{
  if(UI->mData->mModels.size() <= 0){
  }

  if(!UI->mDofSliderWindow->shown()){	
    int nDof = UI->mData->mSelectedModel->GetDofCount();
    for(int i = 0; i < nDof; i++){
	Dof *currDof = UI->mData->mSelectedModel->mDofList.mDofs[i];
	UI->mDofs_sli[i]->maximum(currDof->mUpperBound);
	UI->mDofs_sli[i]->minimum(currDof->mLowerBound);
	UI->mDofs_sli[i]->value(currDof->mVal);
    }
    UI->mDofSliderWindow->show();
  }else{
    UI->mDofSliderWindow->hide();
  }

}

//weights window
void ShowWeightSliders_cb(Fl_Widget *o, void *v){		//i just love generated code :/
	if(UI->mData->mModels.size() <= 0){}
	extern IKSolver solver;

	if(!UI->mWeightSliderWindow->shown()){	
		int nWeights = UI->mData->mSelectedModel->GetHandleCount();
			for(int i = 0; i < nWeights; i++){
				float curVal = solver.weights[i];
				UI->mWeights_sli[i]->maximum(1);
				UI->mWeights_sli[i]->minimum(0);
				UI->mWeights_sli[i]->value(curVal);
			}
		UI->mWeightSliderWindow->show();
	}else{
		UI->mWeightSliderWindow->hide();
	}

}//ShowWeightSliders_cb callback


void DofSliders_cb(Fl_Widget *o, void *v)
{
  int index = (long int)v;
  Fl_Value_Slider* slider = (Fl_Value_Slider*)o;
  UI->mData->mSelectedModel->mDofList.SetDof(index, slider->value());
}


void WeightsSliders_cb(Fl_Widget *o, void *v)
{
  int index = (long int)v;
  Fl_Value_Slider* slider = (Fl_Value_Slider*)o;
  extern IKSolver solver;
  solver.weights[index] = slider->value();
}

void Solve_cb(Fl_Widget *o, void *v)
{
 
  Solution(NULL);
}
