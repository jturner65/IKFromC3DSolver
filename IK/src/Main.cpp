#ifndef RealTimeIKui_h
#include "RealTimeIKui.h"
#endif	//RealTimeIKui_h

#ifndef __PHYLTERFLGLWINDOW_H__
#include "PhylterGLWindow.h"
#endif	//__PHYLTERFLGLWINDOW_H__
#include <time.h>
#include "IKSolver.h" 

using namespace std;
using namespace IKsolverNS;

IKSolver solver; 

static double TICK = 0.03;

RealTimeIKUI *UI;
int solverFrames = 0, maxSolverFrames = 0;//DEBUG
bool showMax = true;					//DEBUG

void onTimer(void *){

	if(UI->mPlay_but->value())   {
		UI->Increment();
		//anim playback
		if((solver.moCapLoaded) && (!solver.runSolver) && (solver.playBackAnim)){//attempt to play back animations already calculated, if any exist
			solver.animPlayBack(UI->mFrameCounter_cou->value());
		}
	}

	if(solver.runSolver){
		solver.setPlayBackAnim(false);//turn off anim playback - redundancy is redundantly safe
		showMax = true;
		//need to run a few iterations per timer click
		int frame = UI->mFrameCounter_cou->value();
		int iters = 0;
		double fq = solver.calcFq(frame);
		//while((fq  > solver.eps * solver.numHandles) && (iters < solver.numIters)){  	
		while(iters < solver.numIters){  	//ignore eps calculation - dists will never exactly  match
			solver.Solve(fq, frame);
			fq = solver.calcFq(frame);
			//cout<<"Fq : "<<fq<<" iters : "<<iters<<endl;
			++iters;
		}//while
		UI->mGLWindow->refresh();
		//if(fq<solver.eps){solver.runSolver = false;}
		++solverFrames;
		//solver.runSolver = false;
	}//if run solver
	else {//not running solver
		solverFrames = 0;
		showMax = false;
		UI->mGLWindow->refresh();
 	}

	Fl::add_timeout(TICK, onTimer);
	maxSolverFrames = max(maxSolverFrames,solverFrames);
	if((showMax) && (!solver.runSolver) && (maxSolverFrames != 0)){cout<<"frames to solve : "<<maxSolverFrames<<endl;  showMax =false;}

}



int main(int argc, char **argv) 
{
  UI = new RealTimeIKUI();
  
  Fl::visual(FL_DOUBLE|FL_INDEX);
  UI->Show();
  Fl::add_timeout(TICK, onTimer);
  return Fl::run();
}
