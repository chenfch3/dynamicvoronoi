#ifndef _DYNAMICVORONOI3D_H_
#define _DYNAMICVORONOI3D_H_

#include <stdlib.h>
#include <stdio.h>
#include <limits.h>
#include <queue>
#include <vector>

#include "bucketedqueue3d.h"

//! A DynamicVoronoi3D object computes and updates a distance map and Voronoi diagram on a 3D grid.
class DynamicVoronoi3D {

public:
  DynamicVoronoi3D();
  ~DynamicVoronoi3D();

  //! Initialization with an empty 3D map
  void initializeEmpty(int _sizeX, int _sizeY, int _sizeZ, bool initGridMap=true);
  //! Initialization with a given binary 3D map (false==free, true==occupied)
  void initializeMap(int _sizeX, int _sizeY, int _sizeZ, bool*** _gridMap);

  //! Add an obstacle at the specified cell coordinate
  void occupyCell(int x, int y, int z);
  //! Remove an obstacle at the specified cell coordinate
  void clearCell(int x, int y, int z);
  //! Remove old dynamic obstacles and add the new ones
  void exchangeObstacles(std::vector<INTPOINT3> newObstacles);

  //! Update distance map and Voronoi diagram to reflect the changes
  void update(bool updateRealDist=true);
  //! Prune the Voronoi diagram to a minimal surface
  void prune();

  //! Returns the obstacle distance at the specified location
  float getDistance(int x, int y, int z);
  //! Returns whether the specified cell is part of the (pruned) Voronoi graph
  bool isVoronoi(int x, int y, int z);
  //! Checks whether the specified location is occupied
  bool isOccupied(int x, int y, int z);
  //! Write the current distance map and Voronoi diagram as a series of per-slice PPM files
  void visualize(const char* filenameBase="result");

  //! Returns the size in X
  unsigned int getSizeX() { return sizeX; }
  //! Returns the size in Y
  unsigned int getSizeY() { return sizeY; }
  //! Returns the size in Z
  unsigned int getSizeZ() { return sizeZ; }

private:
  struct dataCell {
    float dist;
    char voronoi;
    char queueing;
    int obstX;
    int obstY;
    int obstZ;
    bool needsRaise;
    int sqdist;
  };

  typedef enum {voronoiKeep=-4, freeQueued=-3, voronoiRetry=-2, voronoiPrune=-1, free=0, occupied=1} State;
  typedef enum {fwNotQueued=1, fwQueued=2, fwProcessed=3, bwQueued=4, bwProcessed=1} QueueingState;
  typedef enum {invalidObstData = SHRT_MAX/2} ObstDataState;
  typedef enum {pruned, keep, retry} markerMatchResult;

  // Internal methods
  void setObstacle(int x, int y, int z);
  void removeObstacle(int x, int y, int z);
  inline void checkVoro(int x, int y, int z, int nx, int ny, int nz, dataCell& c, dataCell& nc);
  void commitAndColorize(bool updateRealDist=true);
  inline void reviveVoroNeighbors(int& x, int& y, int& z);

  inline bool isOccupied(int& x, int& y, int& z, dataCell& c);
  inline markerMatchResult markerMatch(int x, int y, int z);

  // Queues
  BucketPrioQueue3D open;
  std::queue<INTPOINT3> pruneQueue;

  std::vector<INTPOINT3> removeList;
  std::vector<INTPOINT3> addList;
  std::vector<INTPOINT3> lastObstacles;

  // Map dimensions
  int sizeX;
  int sizeY;
  int sizeZ;
  dataCell*** data;
  bool*** gridMap;
};

#endif
