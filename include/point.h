#ifndef _VOROPOINT_H_
#define _VOROPOINT_H_

#define INTPOINT IntPoint

/*! A light-weight integer point with fields x,y */
class IntPoint {
public:
  IntPoint() : x(0), y(0) {}
  IntPoint(int _x, int _y) : x(_x), y(_y) {}
  int x,y;
};

#define INTPOINT3 IntPoint3

/*! A light-weight integer point with fields x,y,z */
class IntPoint3 {
public:
  IntPoint3() : x(0), y(0), z(0) {}
  IntPoint3(int _x, int _y, int _z) : x(_x), y(_y), z(_z) {}
  int x,y,z;
};

#endif
