#ifndef _PRIORITYQUEUE3D_H_
#define _PRIORITYQUEUE3D_H_

#define MAXDIST3D 500
#define RESERVE3D 64

#include <vector>
#include <queue>
#include <assert.h>
#include "point.h"

//! Priority queue for 3D integer coordinates with squared distances as priority.
/** A priority queue that uses buckets to group elements with the same priority.
 *  The individual buckets are unsorted, which increases efficiency if these groups are large.
 *  The elements are assumed to be integer coordinates, and the priorities are assumed
 *  to be squared euclidean distances (integers) in 3D space.
 */
class BucketPrioQueue3D {

public:
  //! Standard constructor
  /** Standard constructor. When called for the first time it creates a look-up table
   *  that maps squared distances to bucket numbers, which might take some time...
   */
  BucketPrioQueue3D();
  //! Checks whether the Queue is empty
  bool empty();
  //! push an element
  void push(int prio, INTPOINT3 t);
  //! return and pop the element with the lowest squared distance
  INTPOINT3 pop();

private:
  static void initSqrIndices();
  static std::vector<int> sqrIndices;
  static int numBuckets;
  int count;
  int nextBucket;

  std::vector<std::queue<INTPOINT3> > buckets;
};

#endif
