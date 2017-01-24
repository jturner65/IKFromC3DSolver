#ifndef __GLPRIMITIVES_H__
#define __GLPRIMITIVES_H__

#ifndef __PRIMITIVE_H__
#include "Primitive.h"
#endif	//__PRIMITIVE_H__

#include <FL/gl.h>
#include <FL/glu.h>
#include "vl/VLd.h"
#include <FL/glut.H>

const Vec3d PINK(1.0, 0.5, 0.75);
const Vec3d RED(0.703, 0.0117, 0.0625);
const Vec3d BLUE(0.5, 0.5, 1.0);
const Vec3d PURPLE(0.5977,0.3047, 0.8359);
const Vec3d BABY_BLUE(0.5, 0.835, 1.0);
const Vec3d DARK_BLUE(0.2, 0.2, 1.0);
const Vec3d YELLOW(0.9375, 0.9648, 0.2492);
const Vec3d PEACH(1.0, 0.0, 0.5);
const Vec3d GREEN(0.0, 1.0, 0.0);
const Vec3d ORANGE(0.9, 0.55, 0.35);
const Vec3d GREENISH_BLUE(0.3058, 0.6015, 0.6015);
const Vec3d WALL1(0.0, 0.5, 0.5);
const Vec3d WALL2(0.543, 0.0, 0.7461);
const Vec3d WALL3(0.5, 0.5, 0.75);
const Vec3d WALL4(0.0938, 0.9336, 0.7461);
const Vec3d CLOTH(0.5, 0.5, 1.0);

class Cube : public Primitive
{
 public:
  Cube( const Vec3d& color, double mass ){
    mDisplay = true;
    mColor = color;
    mMass = mass;
  }
  Cube( const Vec3d& color, double mass, bool display ){
    mDisplay = display;
    mColor = color;
    mMass = mass;
  }
  Cube( istream& in );

  // from Node
  virtual void Draw();

  virtual Node* Clone() { return new Cube( mColor, mMass ); }

  // from Primitive
  virtual double GetMass() { return mMass; }
  virtual void SetMass(double m){ mMass = m; }

};

class Origin : public Node
{
 public:
  // from Node
  static Vec3d FLOOR_COLOR;
  virtual void Draw();
  virtual Node* Clone() { return new Origin(); }
};

class Sphere : public Primitive
{
 public:
  Sphere( const Vec3d& color, double mass ){
    mDisplay = true;
    mColor = color;
    mMass = mass;
  }
  Sphere( const Vec3d& color, double mass, bool display){
    mDisplay = display;
    mColor = color;
    mMass = mass;
  }
  Sphere( istream& in );

  // from Node
  virtual void Draw();
  virtual Node* Clone() { return new Sphere( mColor, mMass ); }

  // from Primitive
  virtual double GetMass() { return mMass; }
  virtual void SetMass(double m){ mMass = m; }

};

class Cylinder : public Primitive
{
 public:
  Cylinder( const Vec3d& color, double mass ){
    mDisplay = true;
    mColor = color;
    mMass = mass;
  }
  Cylinder( const Vec3d& color, double mass, bool display){
    mDisplay = display;
    mColor = color;
    mMass = mass;
  }
  Cylinder( istream& in );

  // from Node
  virtual void Draw();
  virtual Node* Clone() { return new Cylinder( mColor, mMass ); }

  // from Primitive
  virtual double GetMass() { return mMass; }
  virtual void SetMass(double m){ mMass = m; }
};


class Dummy : public Primitive
{
 public:
  Dummy( const Vec3d& color, double mass ){
    mDisplay = true;
    mColor = color;
    mMass = mass;
  }
  Dummy( const Vec3d& color, double mass, bool display){
    mDisplay = display;
    mColor = color;
    mMass = mass;
  }
  Dummy( istream& in );

  // from Node
  virtual void Draw();
  virtual Node* Clone() { return new Sphere( mColor, mMass ); }

  // from Primitive
  virtual double GetMass() { return mMass; }
  virtual void SetMass(double m){ mMass = m; }

};

#endif
