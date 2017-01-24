#ifndef __C3DFILEINFO_H__
#include "C3dFileInfo.h"
#endif	//__C3DFILEINFO_H__

#ifndef RealTimeIKui_h
#include "RealTimeIKui.h"
#endif //RealTimeIKui_h

#ifndef	__PHOWARDDATA_H__
#include "PhowardData.h"
#endif // __PHOWARDDATA_H__

using namespace std;

extern RealTimeIKUI *UI;

C3dFileInfo::C3dFileInfo(char *filename)
{
  int len = strlen(filename);
  mFileName = new char[len + 1];
  strcpy(mFileName, filename);	
}

void C3dFileInfo::LoadFile()
{ 
  std::vector<Vec3d> data;
  int nFrame;
  int nMarker;
  int freq;

  char buf[C3D_REC_SIZE];
  FILE *file;
  int i, j;
  Vec3d v;
  c3d_head hdr;
  c3d_param param;
  c3d_frameSI frameSI;
  c3d_frame frame;
  bool bDecFmt = true;

  if ( (file = fopen(mFileName, "rb" )) == NULL ){
    cout << "c3d open failed" << endl;
    return; 
  }
	 
  //get the header
  if (!fread(buf, C3D_REC_SIZE, 1, file)){
    cout << "c3d open failed" << endl;
    return;
  }
  memcpy(&hdr, buf, sizeof(hdr));

  //get number format
  if (hdr.rec_start > 2)
    {
      if (!fread(buf, C3D_REC_SIZE, 1, file))
	return;
      memcpy(&param, buf, sizeof(param));
      if (param.ftype == 84)
	bDecFmt = false;
    }
  
  //convert if in dec format
  if (bDecFmt)
    {
      hdr.freq = ConvertDecToFloat((char*)&hdr.freq);
      hdr.scale = ConvertDecToFloat((char*)&hdr.scale);
    }

  //byte switch
/*	SWITCHBYTES(hdr.pnt_cnt);
	SWITCHBYTES(hdr.start_frame);
	SWITCHBYTES(hdr.end_frame);
	SWITCHBYTES(hdr.rec_start);
	//hdr.freq = (int) hdr.freq % 256 * 256 + (int) hdr.freq / 256;
        hdr.freq = 66;
*/

  nFrame = hdr.end_frame - hdr.start_frame + 1;
  nMarker = hdr.pnt_cnt;
  double mC3DScale = hdr.scale;
  freq = hdr.freq;

  float pntScale = (hdr.scale < 0) ? 1 : hdr.scale;

  //eat parameter records
  for (i = 3; i < hdr.rec_start; i++)
    if (!fread(buf, C3D_REC_SIZE, 1, file))
      return;
	
  // start retrieving data
  data.resize(nFrame * nMarker);

  int iRecSize;
  if (mC3DScale < 0)
    iRecSize = sizeof(c3d_frame) + ( hdr.a_channels * hdr.a_frames * sizeof(float));
  else
    iRecSize = sizeof(c3d_frameSI) + ( hdr.a_channels * hdr.a_frames * sizeof(short));
  
  for( i=0;i< nFrame;i++ ){
    for( j=0;j < nMarker;j++ )
      {
	if (!fread(buf, iRecSize, 1, file))
	  return;
	if (mC3DScale < 0)
	  {
	    memcpy(&frame, buf, sizeof(frame));
	    //frame.x = ByteSwitch((char*)&frame.x);
	    //frame.y = ByteSwitch((char*)&frame.y);
	    //frame.z = ByteSwitch((char*)&frame.z);
	    if(bDecFmt){
	      frame.y = ConvertDecToFloat((char*)&frame.y);
	      frame.z = ConvertDecToFloat((char*)&frame.z);
	      frame.x = ConvertDecToFloat((char*)&frame.x);
	    }
	    v[2] = frame.x / 1000.0;
	    v[0] = frame.y / 1000.0;
	    v[1] = frame.z / 1000.0;
	  }
	else
	  {
	    memcpy(&frameSI, buf, sizeof(frameSI));
	    if(bDecFmt){
	      frame.y = ConvertDecToFloat((char*)&frameSI.y);
	      frame.z = ConvertDecToFloat((char*)&frameSI.z);
	      frame.x = ConvertDecToFloat((char*)&frameSI.x);
	    }
	    v[0] = (float)frameSI.y * pntScale / 1000.0;
	    v[1] = (float)frameSI.z * pntScale / 1000.0;
	    v[2] = (float)frameSI.x * pntScale / 1000.0;
	  }
	data[i * nMarker + j] = v;
      }
  }
  
  fclose(file);

  char *pch = strrchr(mFileName, '\\');  //clip leading path
  if (pch) mFileName = pch+1;

  int sampleRate = freq / UI->mData->mFPS;

  mMarkerPos.clear();
  for(int i = 0; i < nFrame; i++){
    if(i % sampleRate != 0)
      continue;
    for(int j = 0; j < nMarker; j++){
      Vec3d tempMarkerPos = data[i * nMarker + j];
      
      mMarkerPos.push_back(tempMarkerPos);
    }
  }

  mNumFrames = ceil(nFrame / (double)sampleRate);
  mNumMarkers = nMarker;
}

float C3dFileInfo::ConvertDecToFloat(char bytes[4]) 
{
    char p[4]; 
    p[0] = bytes[2]; 
    p[1] = bytes[3]; 
    p[2] = bytes[0]; 
    p[3] = bytes[1]; 
    if (p[0] || p[1] || p[2] || p[3]) 
        --p[3];          // adjust exponent 
    return *(float*)p; 
} 
  

void C3dFileInfo::ConvertFloatToDec(float f, char* bytes) 
{
    char* p = (char*)&f; 
    bytes[0] = p[2]; 
    bytes[1] = p[3]; 
    bytes[2] = p[0]; 
    bytes[3] = p[1]; 
    if (bytes[0] || bytes[1] || bytes[2] || bytes[3]) 
        ++bytes[1];      // adjust exponent 
}
