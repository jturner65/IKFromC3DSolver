#ifndef __DOF_H__
#define __DOF_H__

#include <vector>
#include "vl/VLd.h"
#include <iostream>

using namespace std;

class Dof
{
 public:
 Dof( char* name, double min, double max, int id )
   : mName( name ), mId( id ){
    mVal = 0.0;
    mUpperBound = max;
    mLowerBound = min;
  }

  ~Dof(){
    delete [] mName;
  }

  char* GetName() { return mName; }
        

  virtual double GetTransformValue(){
    return 0.0;
  }
  virtual void SetTransformValue(double value){};
  
  virtual int ReturnType(){
    return -1;
  }
  		
  int mId;  // the order of this dof in mSelectedModel->mDofList.mDofs
  double mVal; // current value of this dof
  double mUpperBound;
  double mLowerBound;

 private:
  char* mName;
};

class DofList{
 public:
  DofList(){  }
  ~DofList(){    for(int i = 0; i < mDofs.size(); i++){      delete mDofs[i];    }  }
  void AddDof( Dof* dof ) { mDofs.push_back( dof ); }
  void SetDofs( Vecd & data );
  void SetDof(int index, double value);
  void GetDofs( Vecd *data );
  double GetDof(int index);
  void UpdateDofs();
     
  std::vector<Dof*> mDofs;
};

#endif
