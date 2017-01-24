#ifndef __PHYLTERFLGLWINDOW_H__
#define __PHYLTERFLGLWINDOW_H__

#ifndef __TRACKBALL_H__
#include "Trackball.h"
#endif	//__TRACKBALL_H__

#include <FL/Fl.H>
#include <FL/Fl_Gl_Window.H>
#include <FL/fl_draw.H>
#include <FL/Fl_File_Chooser.H>
#include <FL/gl.h>
#include <FL/glut.H>
#include <FL/glu.h>
#include <stdlib.h>
#include "vl/VLd.h"
#include <vector>

class Phylter_Fl_Gl_Window : public Fl_Gl_Window
{
 public:
  Phylter_Fl_Gl_Window(int x, int y, int w, int h);

  void draw();
  int handle(int event);
  void refresh();
  void resizeWindow(int width, int height);
  void resize( int x, int y, int w, int h );

  void DrawC3dMarkers();
	
  bool mShowConstraints;
  bool mShowModel;
  bool mShowViz;

  int mWinWidth, mWinHeight;
  Trackball mTrackball;

 private:
  void ResetCamera();

  Vec3d mMouse;
  float mXstart, mYstart, mXangle, mYangle, mZoom, mTranslateX, mTranslateY;
  bool mControlState, mShiftState, mAltState;
  Vec3d mFixedTrans;
  bool mRotatingXY, mZooming, mTranslating;
  int mWhichClick;
};

#endif
