#ifndef __PRIMITIVE_H__
#define __PRIMITIVE_H__

#ifndef __NODE_H__
#include "Node.h"
#endif	//__NODE_H__

class Primitive : public Node
{
 public:
  virtual double GetMass() = 0;
  virtual void SetMass(double m) = 0;
  void SetDisplay(bool dis){
    mDisplay = dis;
  }

  bool mDisplay;
  double mMass;
};

#endif
