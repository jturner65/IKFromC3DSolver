#ifndef __COMMAND_H__
#define __COMMAND_H__

#include <fstream>
#include "vl/VLd.h"
#include <vector>

typedef void (*Command)(void*);

void LoadModel(void*);
void LoadC3d(void*);
void Exit(void*);
void Solution(void*);
void TerminateSolver(void*);
void AnimPlayback(void *v);
void SwitchObjFunc(void *v);

#endif

