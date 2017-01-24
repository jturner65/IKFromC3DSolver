#ifndef __PHYLTERFLGLWINDOW_H__
#include "PhylterGLWindow.h"
#endif	//__PHYLTERFLGLWINDOW_H__

#ifndef RealTimeIKui_h
#include "RealTimeIKui.h"
#endif //RealTimeIKui_h

#ifndef	__PHOWARDDATA_H__
#include "PhowardData.h"
#endif

#ifndef __ARTICULATEDBODY_H__
#include "ArticulatedBody.h"
#endif	//__ARTICULATEDBODY_H__

#ifndef __C3DFILEINFO_H__
#include "C3dFileInfo.h"
#endif	//__C3DFILEINFO_H__

#define BUFSIZE 512

extern RealTimeIKUI *UI;


void LightInit(){
	static float ambient[]             = {0.2, 0.2, 0.2, 1.0};
	static float diffuse[]             = {0.6, 0.6, 0.6, 1.0};
	static float front_mat_shininess[] = {60.0};
	static float front_mat_specular[]  = {0.2, 0.2,  0.2,  1.0};
	static float front_mat_diffuse[]   = {0.5, 0.28, 0.38, 1.0};
	static float lmodel_ambient[]      = {0.2, 0.2,  0.2,  1.0};
	static float lmodel_twoside[]      = {GL_FALSE};

	GLfloat position[] = {1.0,0.0,0.0,0.0};
	GLfloat position1[] = {-1.0,0.0,0.0,0.0};
	GLfloat position2[] = {0.0,0.0,1.0,0.0};
	GLfloat position3[] = {0.0,0.0,-1.0,0.0};

	glEnable(GL_LIGHT0);
	glLightfv(GL_LIGHT0, GL_AMBIENT,  ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE,  diffuse);
	glLightfv(GL_LIGHT0, GL_POSITION, position);
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT,  lmodel_ambient);
	glLightModelfv(GL_LIGHT_MODEL_TWO_SIDE, lmodel_twoside);
	glEnable( GL_LIGHT1);
	glLightfv(GL_LIGHT1,GL_DIFFUSE, diffuse);
	glLightfv(GL_LIGHT1,GL_POSITION, position1);

	glEnable(GL_LIGHTING);
	glEnable(GL_COLOR_MATERIAL);

	glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, front_mat_shininess);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,  front_mat_specular);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE,   front_mat_diffuse);

	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glDisable(GL_CULL_FACE);
	glEnable(GL_NORMALIZE);
}

void inits(void){
	glClearColor(0.7,0.7,0.7,0.0);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
	glShadeModel(GL_SMOOTH);
	LightInit();
}
	
Phylter_Fl_Gl_Window::Phylter_Fl_Gl_Window(int x, int y, int w, int h): Fl_Gl_Window(x,y,w,h,NULL),mTrackball(Vec3d(0,0,0), h/2.5){
	mWinWidth = w;
	mWinHeight = h;

	mZoom = mTranslateX = mTranslateY = 0;

	mXangle = mXstart = 0;
	mYangle = mYstart = 180;
	mControlState = mShiftState = mAltState = false;
	mRotatingXY = mZooming = mTranslating = false;
	mFixedTrans = vl_0;

	mShowConstraints = false;
	mShowModel =  false;
	mShowViz = false;
}


void Phylter_Fl_Gl_Window::draw(){  
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(30.0, 1.33, 0.3, 600.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable( GL_DEPTH_TEST );
	gluLookAt(0, 0, 10, 0, 0, -100, 0, 1, 0);			//<--!!! set where the eye starts
	
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	inits();

	double *rotationMat = new double[16];
	mTrackball.GetCurrentRotation(rotationMat);
	glMultMatrixd( rotationMat );
	
	// draw axes
	glDisable( GL_LIGHTING );
	if(mRotatingXY || mZooming || mTranslating){
		glLineWidth(4.0);
		glColor3f( 1.0f, 0.0f, 0.0f );
		glBegin( GL_LINES );
		glVertex3f( -0.40f, 0.0f, -0.0f );
		glVertex3f( 0.4f, 0.0f, -0.0f );
		glEnd();

		glColor3f( 0.0f, 1.0f, 0.0f );
		glBegin( GL_LINES );
		glVertex3f( 0.0f, -0.4f, 0.0f );
		glVertex3f( 0.0f, 0.4f, 0.0f );
		glEnd();

		glColor3f( 0.0f, 0.0f, 1.0f );
		glBegin( GL_LINES );
		glVertex3f( 0.0f, 0.0f, -0.4f );
		glVertex3f( 0.0f, 0.0f, 0.4f );
		glEnd();
	}
	glEnable( GL_LIGHTING );
	
	glTranslatef(0.0f, -0.3f, 0);
	glTranslatef(mFixedTrans[0] / 30.0, mFixedTrans[1] / 30, mFixedTrans[2] / 10);
	//  glRotatef(-90, 0, 1, 0);

	Origin o;
	o.Draw();
  
	if(mShowViz){ for (int i = 0; i < UI->mData->mModels.size(); i++) {  UI->mData->mModels[i]->DrawVisualization(); } }
	if(mShowModel){ for (int i = 0; i < UI->mData->mModels.size(); i++) { UI->mData->mModels[i]->DrawSkeleton(); }  }
	if(mShowConstraints){    DrawC3dMarkers();  }
      
// Draw the globe
	if(mRotatingXY){
		glPushMatrix();
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glViewport( 0, 0, mWinWidth, mWinHeight );
		gluOrtho2D(0.0, (GLdouble)mWinWidth, 0.0, (GLdouble)mWinHeight);

		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		glDisable( GL_LIGHTING );
		glLineWidth(4.0);
		glColor3f( 1.0f, 1.0f, 0.0f );
		double r = mTrackball.mRadius;
		glBegin( GL_LINE_LOOP);		
		for(int i = 0; i < 360; i+=4){
			double theta = i / 180.0 * 3.14159;
			double x = r * cos(theta);
			double y = r * sin(theta);
			glVertex2d( (GLdouble)((mWinWidth >> 1) + x), (GLdouble)((mWinHeight >> 1) + y));
		}
		glEnd();
		glPopMatrix();
		glEnable( GL_LIGHTING );
	}

	delete [] rotationMat;
	glFlush();
}

int Phylter_Fl_Gl_Window::handle(int event){  
	switch (event) {

		case FL_PUSH:
			Fl::focus(UI->mGLWindow);
			mMouse[0] = Fl::event_x() - (mWinWidth >> 1);
			mMouse[1] = (mWinHeight >> 1) - Fl::event_y();

			mTrackball.StartBall(mMouse[0], mMouse[1]);

			mShiftState = (Fl::get_key(FL_Shift_L) || Fl::get_key(FL_Shift_R));
			mControlState = (Fl::get_key(FL_Control_L) || Fl::get_key(FL_Control_R));
			mAltState = (Fl::get_key(FL_Alt_L) || Fl::get_key(FL_Alt_R));

			if (Fl::event_button() == 1) {
				if (mControlState){		mWhichClick = 1; }
				else if (mAltState) {	mWhichClick = 2; }
				else {					mWhichClick = 0; }
			}

			if (mWhichClick == 0) {
				if (mShiftState) { mRotatingXY = true; }

				mXstart = mMouse[0];
				mYstart = mMouse[1];

			}
			else if (Fl::event_button() == 2 || mWhichClick == 1) {
				if (mShiftState) {
					mTranslating = true;
					mXstart = mMouse[0];
					mYstart = mMouse[1];
				}
			}
			else if (Fl::event_button() == 3 || mWhichClick == 2) {
				if (mShiftState) {	mZooming = true;}
				mXstart = mMouse[0];
				mYstart = mMouse[1];
			}
			break;

		case FL_DRAG:{
			if (mRotatingXY) {
				mYangle += (mMouse[0] - mXstart);
				mXangle += (mMouse[1] - mYstart);
				if (mYangle > 360)
					mYangle -= 360;
				else if (mYangle < 0)
					mYangle += 360;
				if (mXangle > 360)
					mXangle -= 360;
				else if (mXangle < 0)
					mXangle += 360;
				mXstart = mMouse[0];
				mYstart = mMouse[1];

				redraw();

				mTrackball.UpdateBall(mMouse[0], mMouse[1]);
			}
			else if (mZooming) {
				float xVal = mMouse[0] - mXstart;
				float yVal = mMouse[1] - mYstart;
				double shift;

				if (xVal != 0)
					shift = sqrt(xVal * xVal + yVal * yVal) * (xVal / fabs(xVal));
				else
					shift = sqrt(xVal * xVal + yVal * yVal);

				mZoom += shift;
				Mat4d tempRot;
				mTrackball.GetCurrentRotation(tempRot);
				tempRot = inv(tempRot);
				Vec4d tempTrans = tempRot * Vec4d(0, 0, shift, 1);
				mFixedTrans[0] += tempTrans[0];
				mFixedTrans[1] += tempTrans[1];
				mFixedTrans[2] += tempTrans[2];

				mXstart = mMouse[0];
				mYstart = mMouse[1];
				redraw();

			}
			else if (mTranslating) {
				mTranslateX += (mMouse[0] - mXstart);
				mTranslateY += (mMouse[1] - mYstart);
				Mat4d tempRot;
				mTrackball.GetCurrentRotation(tempRot);
				tempRot = inv(tempRot);
				Vec4d tempTrans = tempRot * Vec4d(mMouse[0] - mXstart, mMouse[1] - mYstart, 0, 1);
				mFixedTrans[0] += tempTrans[0];
				mFixedTrans[1] += tempTrans[1];
				mFixedTrans[2] += tempTrans[2];
				mXstart = mMouse[0];
				mYstart = mMouse[1];
				redraw();

			}

			mMouse[0] = Fl::event_x() - (mWinWidth >> 1);
			mMouse[1] = (mWinHeight >> 1) - Fl::event_y();

			break;}

		case FL_RELEASE:{
			mMouse[0] = Fl::event_x() - (mWinWidth >> 1);
			mMouse[1] = (mWinHeight >> 1) - Fl::event_y();

			mRotatingXY = mZooming = mTranslating = false;
			redraw();
			break; }

		default:    return 0;
	}
  return 1;
}

void Phylter_Fl_Gl_Window::refresh(){  redraw();}
void Phylter_Fl_Gl_Window::resizeWindow(int width, int height){  resize(x(), y(), width, height);}
void Phylter_Fl_Gl_Window::resize(int x, int y, int w, int h){
	mWinWidth = w;
	mWinHeight = h;
	ResetCamera();
	Fl_Gl_Window::resize(x, y, w, h);
}

void Phylter_Fl_Gl_Window::ResetCamera(){
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glViewport(0, 0, mWinWidth, mWinHeight);
	gluPerspective(30.0, 1.33, 0.3, 600.0);
}


void Phylter_Fl_Gl_Window::DrawC3dMarkers(){
	if (!UI->mData->mSelectedModel->mOpenedC3dFile) {	return;}
	int currFrame = UI->mFrameCounter_cou->value();
	int nHandle = UI->mData->mSelectedModel->mOpenedC3dFile->GetMarkerCount();
	for(int j = 0; j < nHandle; j++){
		glPushMatrix();
		Sphere temp(YELLOW, 0.0);
		Vec3d pos = UI->mData->mSelectedModel->mOpenedC3dFile->GetMarkerPos(currFrame, j);
		glTranslated(pos[0], pos[1], pos[2]);
		glScaled(0.03, 0.03, 0.03);
		temp.Draw();
		glPopMatrix();
	}
}

