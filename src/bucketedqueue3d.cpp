#include "bucketedqueue3d.h"

#include "limits.h"
#include <stdio.h>
#include <stdlib.h>

std::vector<int> BucketPrioQueue3D::sqrIndices;
int BucketPrioQueue3D::numBuckets;


BucketPrioQueue3D::BucketPrioQueue3D() {
  if (sqrIndices.size() == 0) initSqrIndices();
  nextBucket = INT_MAX;

  buckets = std::vector<std::queue<INTPOINT3> >(numBuckets);

  count = 0;
}

bool BucketPrioQueue3D::empty() {
  return (count == 0);
}

void BucketPrioQueue3D::push(int prio, INTPOINT3 t) {
  if (prio >= (int)sqrIndices.size()) {
    fprintf(stderr, "error: priority %d exceeds maximum squared distance for MAXDIST3D=%d.\n", prio, MAXDIST3D);
    exit(-1);
  }
  int id = sqrIndices[prio];
  if (id < 0) {
    fprintf(stderr, "error: priority %d is not a valid squared distance x*x+y*y+z*z for MAXDIST3D=%d.\n", prio, MAXDIST3D);
    exit(-1);
  }
  buckets[id].push(t);
  if (id < nextBucket) nextBucket = id;
  count++;
}

INTPOINT3 BucketPrioQueue3D::pop() {
  assert(count > 0);
  int i;
  for (i = nextBucket; i < (int)buckets.size(); i++) {
    if (!buckets[i].empty()) break;
  }
  assert(i < (int)buckets.size());
  nextBucket = i;
  count--;
  INTPOINT3 p = buckets[i].front();
  buckets[i].pop();
  return p;
}

void BucketPrioQueue3D::initSqrIndices() {
  // Allocate index array to cover all possible squared distances: x*x+y*y+z*z
  // with 0 <= x,y,z <= MAXDIST3D. Maximum value = 3 * MAXDIST3D * MAXDIST3D.
  sqrIndices = std::vector<int>(3 * MAXDIST3D * MAXDIST3D + 1, -1);

  int count = 0;
  // Iterate over all ordered triples (x >= y >= z >= 0) to enumerate every
  // unique value of x*x+y*y+z*z reachable within MAXDIST3D.
  for (int x = 0; x <= MAXDIST3D; x++) {
    for (int y = 0; y <= x; y++) {
      for (int z = 0; z <= y; z++) {
        int sqr = x*x + y*y + z*z;
        if (sqrIndices[sqr] == -1) {
          sqrIndices[sqr] = count++;
        }
      }
    }
  }
  numBuckets = count;
}
