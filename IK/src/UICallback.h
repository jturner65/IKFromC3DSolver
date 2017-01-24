#ifndef __UICALLBACK_H__
#define __UICALLBACK_H__
#include <FL/Fl.H>

class Fl_Widget;

void LoadModel_cb(Fl_Widget *o, void *v);
void Motion_cb(Fl_Widget *o, void *v);
void LoadMocap_cb(Fl_Widget *o, void *v);
void SaveMotion_cb(Fl_Widget *o, void *v);
void SaveFeatures_cb(Fl_Widget *o, void *v);
void Exit_cb(Fl_Widget *o, void *v);

void PlayStop_cb(Fl_Widget *o, void *v);
void Speed_cb(Fl_Widget *o, void *v);
void FrameCounter_cb(Fl_Widget *o, void *v);
void Frame_scr_cb(Fl_Widget *o, void *v);

void RecordMotion_cb(Fl_Widget *o, void *v);
void StillShot_cb(Fl_Widget *o, void *v);

void ShowModel_cb(Fl_Widget *o, void *v);
void ShowConstr_cb(Fl_Widget *o, void *v);
void ShowViz_cb(Fl_Widget *o, void *v);

void Terminate_cb(Fl_Widget *o, void *v);
void About_cb(Fl_Widget *o, void *v);
void PlaybackAnim_cb(Fl_Widget *o, void *v);
void SwitchObjFunc_cb(Fl_Widget *o, void *v);
void ShowSliders_cb(Fl_Widget *o, void *v);
void ShowWeightSliders_cb(Fl_Widget *o, void *v);

void ShowModel_ite_cb(Fl_Widget *o, void *v);
void ShowConstr_ite_cb(Fl_Widget *o, void *v);
void ShowViz_ite_cb(Fl_Widget *o, void *v);

void DofSliders_cb(Fl_Widget *o, void *v);
void WeightsSliders_cb(Fl_Widget *o, void *v);
void Return_abt_but(Fl_Widget *o, void *v);
void Solve_cb(Fl_Widget *o, void *v);

#endif
