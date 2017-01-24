#ifndef __C3DFILEINFO_H__
#define __C3DFILEINFO_H__

#include "vl/vld.h"
#include <vector>

#define C3D_REC_SIZE   512

typedef struct c3d_head_t {
	unsigned char	prec_start;
	unsigned char	key;
	short	pnt_cnt;
	short	a_channels;
	short	start_frame;
	short	end_frame;
	short	int_gap;
	float	scale;
	short	rec_start;
	short	a_frames;
	float	freq;
	short	stuff[244];	
} c3d_head;

typedef struct c3d_param_t {
	unsigned char	reserved[2];
	unsigned char	pblocks;
	unsigned char	ftype;
	char stuff[C3D_REC_SIZE-4];	
} c3d_param;

typedef struct c3d_frameSI_t {
	short	x, y, z;
	unsigned char	cam_byte;
	unsigned char	residual;
} c3d_frameSI;

typedef struct c3d_frame_t {
	float	x, y, z;
	float	residual;
} c3d_frame;


class C3dFileInfo
{
 public:
  C3dFileInfo(char* name);
  virtual ~C3dFileInfo(){};

  char *mFileName;

  int GetFrameCount(){
    return mNumFrames;	
  }

  int GetMarkerCount(){ 
    return mNumMarkers;
  }

  void LoadFile();
  Vec3d GetMarkerPos(int frameIndex, int markerIndex){
    return mMarkerPos[frameIndex * mNumMarkers + markerIndex];
  }
  void SetMarkerPos(int frameIndex, int markerIndex, Vec3d pos){
    mMarkerPos[frameIndex * mNumMarkers + markerIndex] = pos;
  }

  std::vector<Vec3d> mMarkerPos;

 private:
  int mNumFrames;
  int mNumMarkers;		//correspond to mocap markers? - renamed by JT for clarity
  float ConvertDecToFloat(char bytes[4]);
  void ConvertFloatToDec(float f, char* bytes);
};

#endif
