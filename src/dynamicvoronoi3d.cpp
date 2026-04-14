#include "dynamicvoronoi3d.h"

#include <math.h>
#include <iostream>
#include <string.h>

DynamicVoronoi3D::DynamicVoronoi3D() {
  data = NULL;
  gridMap = NULL;
}

DynamicVoronoi3D::~DynamicVoronoi3D() {
  if (data) {
    for (int x = 0; x < sizeX; x++) {
      for (int y = 0; y < sizeY; y++) delete[] data[x][y];
      delete[] data[x];
    }
    delete[] data;
  }
  if (gridMap) {
    for (int x = 0; x < sizeX; x++) {
      for (int y = 0; y < sizeY; y++) delete[] gridMap[x][y];
      delete[] gridMap[x];
    }
    delete[] gridMap;
  }
}

void DynamicVoronoi3D::initializeEmpty(int _sizeX, int _sizeY, int _sizeZ, bool initGridMap) {
  sizeX = _sizeX;
  sizeY = _sizeY;
  sizeZ = _sizeZ;

  if (data) {
    for (int x = 0; x < sizeX; x++) {
      for (int y = 0; y < sizeY; y++) delete[] data[x][y];
      delete[] data[x];
    }
    delete[] data;
  }
  data = new dataCell**[sizeX];
  for (int x = 0; x < sizeX; x++) {
    data[x] = new dataCell*[sizeY];
    for (int y = 0; y < sizeY; y++) data[x][y] = new dataCell[sizeZ];
  }

  if (initGridMap) {
    if (gridMap) {
      for (int x = 0; x < sizeX; x++) {
        for (int y = 0; y < sizeY; y++) delete[] gridMap[x][y];
        delete[] gridMap[x];
      }
      delete[] gridMap;
    }
    gridMap = new bool**[sizeX];
    for (int x = 0; x < sizeX; x++) {
      gridMap[x] = new bool*[sizeY];
      for (int y = 0; y < sizeY; y++) gridMap[x][y] = new bool[sizeZ];
    }
  }

  dataCell c;
  c.dist = INFINITY;
  c.sqdist = INT_MAX;
  c.obstX = invalidObstData;
  c.obstY = invalidObstData;
  c.obstZ = invalidObstData;
  c.voronoi = free;
  c.queueing = fwNotQueued;
  c.needsRaise = false;

  for (int x = 0; x < sizeX; x++)
    for (int y = 0; y < sizeY; y++)
      for (int z = 0; z < sizeZ; z++) data[x][y][z] = c;

  if (initGridMap) {
    for (int x = 0; x < sizeX; x++)
      for (int y = 0; y < sizeY; y++)
        for (int z = 0; z < sizeZ; z++) gridMap[x][y][z] = false;
  }
}

void DynamicVoronoi3D::initializeMap(int _sizeX, int _sizeY, int _sizeZ, bool*** _gridMap) {
  gridMap = _gridMap;
  initializeEmpty(_sizeX, _sizeY, _sizeZ, false);

  for (int x = 0; x < sizeX; x++) {
    for (int y = 0; y < sizeY; y++) {
      for (int z = 0; z < sizeZ; z++) {
        if (gridMap[x][y][z]) {
          dataCell c = data[x][y][z];
          if (!isOccupied(x, y, z, c)) {
            // Check if fully surrounded by occupied cells; if so, skip wavefront seeding.
            bool isSurrounded = true;
            for (int dx = -1; dx <= 1 && isSurrounded; dx++) {
              int nx = x + dx;
              if (nx <= 0 || nx >= sizeX - 1) continue;
              for (int dy = -1; dy <= 1 && isSurrounded; dy++) {
                int ny = y + dy;
                if (ny <= 0 || ny >= sizeY - 1) continue;
                for (int dz = -1; dz <= 1 && isSurrounded; dz++) {
                  if (dx == 0 && dy == 0 && dz == 0) continue;
                  int nz = z + dz;
                  if (nz <= 0 || nz >= sizeZ - 1) continue;
                  if (!gridMap[nx][ny][nz]) isSurrounded = false;
                }
              }
            }
            if (isSurrounded) {
              c.obstX = x;
              c.obstY = y;
              c.obstZ = z;
              c.sqdist = 0;
              c.dist = 0;
              c.voronoi = occupied;
              c.queueing = fwProcessed;
              data[x][y][z] = c;
            } else {
              setObstacle(x, y, z);
            }
          }
        }
      }
    }
  }
}

void DynamicVoronoi3D::occupyCell(int x, int y, int z) {
  gridMap[x][y][z] = true;
  setObstacle(x, y, z);
}

void DynamicVoronoi3D::clearCell(int x, int y, int z) {
  gridMap[x][y][z] = false;
  removeObstacle(x, y, z);
}

void DynamicVoronoi3D::setObstacle(int x, int y, int z) {
  dataCell c = data[x][y][z];
  if (isOccupied(x, y, z, c)) return;

  addList.push_back(INTPOINT3(x, y, z));
  c.obstX = x;
  c.obstY = y;
  c.obstZ = z;
  data[x][y][z] = c;
}

void DynamicVoronoi3D::removeObstacle(int x, int y, int z) {
  dataCell c = data[x][y][z];
  if (!isOccupied(x, y, z, c)) return;

  removeList.push_back(INTPOINT3(x, y, z));
  c.obstX = invalidObstData;
  c.obstY = invalidObstData;
  c.obstZ = invalidObstData;
  c.queueing = bwQueued;
  data[x][y][z] = c;
}

void DynamicVoronoi3D::exchangeObstacles(std::vector<INTPOINT3> points) {
  for (unsigned int i = 0; i < lastObstacles.size(); i++) {
    int x = lastObstacles[i].x;
    int y = lastObstacles[i].y;
    int z = lastObstacles[i].z;
    if (gridMap[x][y][z]) continue;
    removeObstacle(x, y, z);
  }
  lastObstacles.clear();

  for (unsigned int i = 0; i < points.size(); i++) {
    int x = points[i].x;
    int y = points[i].y;
    int z = points[i].z;
    if (gridMap[x][y][z]) continue;
    setObstacle(x, y, z);
    lastObstacles.push_back(points[i]);
  }
}

void DynamicVoronoi3D::update(bool updateRealDist) {
  commitAndColorize(updateRealDist);

  while (!open.empty()) {
    INTPOINT3 p = open.pop();
    int x = p.x;
    int y = p.y;
    int z = p.z;
    dataCell c = data[x][y][z];

    if (c.queueing == fwProcessed) continue;

    if (c.needsRaise) {
      // RAISE: propagate invalidation to neighbors.
      // The boundary check nx<=0 || nx>=sizeX-1 (and analogously for y,z)
      // intentionally excludes the outermost cell layer from wavefront
      // processing, matching the convention of the 2D implementation.
      for (int dx = -1; dx <= 1; dx++) {
        int nx = x + dx;
        if (nx <= 0 || nx >= sizeX - 1) continue;
        for (int dy = -1; dy <= 1; dy++) {
          int ny = y + dy;
          if (ny <= 0 || ny >= sizeY - 1) continue;
          for (int dz = -1; dz <= 1; dz++) {
            if (dx == 0 && dy == 0 && dz == 0) continue;
            int nz = z + dz;
            if (nz <= 0 || nz >= sizeZ - 1) continue;
            dataCell nc = data[nx][ny][nz];
            if (nc.obstX != invalidObstData && !nc.needsRaise) {
              if (!isOccupied(nc.obstX, nc.obstY, nc.obstZ, data[nc.obstX][nc.obstY][nc.obstZ])) {
                open.push(nc.sqdist, INTPOINT3(nx, ny, nz));
                nc.queueing = fwQueued;
                nc.needsRaise = true;
                nc.obstX = invalidObstData;
                nc.obstY = invalidObstData;
                nc.obstZ = invalidObstData;
                if (updateRealDist) nc.dist = INFINITY;
                nc.sqdist = INT_MAX;
                data[nx][ny][nz] = nc;
              } else {
                if (nc.queueing != fwQueued) {
                  open.push(nc.sqdist, INTPOINT3(nx, ny, nz));
                  nc.queueing = fwQueued;
                  data[nx][ny][nz] = nc;
                }
              }
            }
          }
        }
      }
      c.needsRaise = false;
      c.queueing = bwProcessed;
      data[x][y][z] = c;
    }
    else if (c.obstX != invalidObstData &&
             isOccupied(c.obstX, c.obstY, c.obstZ, data[c.obstX][c.obstY][c.obstZ])) {

      // LOWER: propagate shorter distances to neighbors
      c.queueing = fwProcessed;
      c.voronoi = occupied;

      for (int dx = -1; dx <= 1; dx++) {
        int nx = x + dx;
        if (nx <= 0 || nx >= sizeX - 1) continue;
        for (int dy = -1; dy <= 1; dy++) {
          int ny = y + dy;
          if (ny <= 0 || ny >= sizeY - 1) continue;
          for (int dz = -1; dz <= 1; dz++) {
            if (dx == 0 && dy == 0 && dz == 0) continue;
            int nz = z + dz;
            if (nz <= 0 || nz >= sizeZ - 1) continue;
            dataCell nc = data[nx][ny][nz];
            if (!nc.needsRaise) {
              int distx = nx - c.obstX;
              int disty = ny - c.obstY;
              int distz = nz - c.obstZ;
              int newSqDistance = distx*distx + disty*disty + distz*distz;
              bool overwrite = (newSqDistance < nc.sqdist);
              if (!overwrite && newSqDistance == nc.sqdist) {
                if (nc.obstX == invalidObstData ||
                    !isOccupied(nc.obstX, nc.obstY, nc.obstZ, data[nc.obstX][nc.obstY][nc.obstZ]))
                  overwrite = true;
              }
              if (overwrite) {
                open.push(newSqDistance, INTPOINT3(nx, ny, nz));
                nc.queueing = fwQueued;
                if (updateRealDist) nc.dist = sqrt((double)newSqDistance);
                nc.sqdist = newSqDistance;
                nc.obstX = c.obstX;
                nc.obstY = c.obstY;
                nc.obstZ = c.obstZ;
              } else {
                checkVoro(x, y, z, nx, ny, nz, c, nc);
              }
              data[nx][ny][nz] = nc;
            }
          }
        }
      }
    }
    data[x][y][z] = c;
  }
}

float DynamicVoronoi3D::getDistance(int x, int y, int z) {
  if (x > 0 && x < sizeX && y > 0 && y < sizeY && z > 0 && z < sizeZ)
    return data[x][y][z].dist;
  return -INFINITY;
}

bool DynamicVoronoi3D::isVoronoi(int x, int y, int z) {
  dataCell c = data[x][y][z];
  return (c.voronoi == free || c.voronoi == voronoiKeep);
}

bool DynamicVoronoi3D::isOccupied(int x, int y, int z) {
  dataCell c = data[x][y][z];
  return (c.obstX == x && c.obstY == y && c.obstZ == z);
}

bool DynamicVoronoi3D::isOccupied(int& x, int& y, int& z, dataCell& c) {
  return (c.obstX == x && c.obstY == y && c.obstZ == z);
}

void DynamicVoronoi3D::commitAndColorize(bool updateRealDist) {
  // ADD NEW OBSTACLES
  for (unsigned int i = 0; i < addList.size(); i++) {
    INTPOINT3 p = addList[i];
    int x = p.x, y = p.y, z = p.z;
    dataCell c = data[x][y][z];
    if (c.queueing != fwQueued) {
      if (updateRealDist) c.dist = 0;
      c.sqdist = 0;
      c.obstX = x;
      c.obstY = y;
      c.obstZ = z;
      c.queueing = fwQueued;
      c.voronoi = occupied;
      data[x][y][z] = c;
      open.push(0, INTPOINT3(x, y, z));
    }
  }

  // REMOVE OLD OBSTACLES
  for (unsigned int i = 0; i < removeList.size(); i++) {
    INTPOINT3 p = removeList[i];
    int x = p.x, y = p.y, z = p.z;
    dataCell c = data[x][y][z];
    if (isOccupied(x, y, z, c)) continue; // reinserted, skip
    open.push(0, INTPOINT3(x, y, z));
    if (updateRealDist) c.dist = INFINITY;
    c.sqdist = INT_MAX;
    c.needsRaise = true;
    data[x][y][z] = c;
  }
  removeList.clear();
  addList.clear();
}

void DynamicVoronoi3D::checkVoro(int x, int y, int z,
                                  int nx, int ny, int nz,
                                  dataCell& c, dataCell& nc) {
  if ((c.sqdist > 1 || nc.sqdist > 1) && nc.obstX != invalidObstData) {
    // The two nearest obstacles must be separated by more than 1 cell in any axis
    if (abs(c.obstX - nc.obstX) > 1 || abs(c.obstY - nc.obstY) > 1 || abs(c.obstZ - nc.obstZ) > 1) {
      // Distance from (x,y,z) to nc's obstacle
      int dxy_x = x - nc.obstX;
      int dxy_y = y - nc.obstY;
      int dxy_z = z - nc.obstZ;
      int sqdxy = dxy_x*dxy_x + dxy_y*dxy_y + dxy_z*dxy_z;
      int stability_xy = sqdxy - c.sqdist;
      if (stability_xy < 0) return;

      // Distance from (nx,ny,nz) to c's obstacle
      int dnxy_x = nx - c.obstX;
      int dnxy_y = ny - c.obstY;
      int dnxy_z = nz - c.obstZ;
      int sqdnxy = dnxy_x*dnxy_x + dnxy_y*dnxy_y + dnxy_z*dnxy_z;
      int stability_nxy = sqdnxy - nc.sqdist;
      if (stability_nxy < 0) return;

      // Mark the more stable cell (or both) as part of the Voronoi surface
      if (stability_xy <= stability_nxy && c.sqdist > 2) {
        if (c.voronoi != free) {
          c.voronoi = free;
          reviveVoroNeighbors(x, y, z);
          pruneQueue.push(INTPOINT3(x, y, z));
        }
      }
      if (stability_nxy <= stability_xy && nc.sqdist > 2) {
        if (nc.voronoi != free) {
          nc.voronoi = free;
          reviveVoroNeighbors(nx, ny, nz);
          pruneQueue.push(INTPOINT3(nx, ny, nz));
        }
      }
    }
  }
}

void DynamicVoronoi3D::reviveVoroNeighbors(int& x, int& y, int& z) {
  for (int dx = -1; dx <= 1; dx++) {
    int nx = x + dx;
    if (nx <= 0 || nx >= sizeX - 1) continue;
    for (int dy = -1; dy <= 1; dy++) {
      int ny = y + dy;
      if (ny <= 0 || ny >= sizeY - 1) continue;
      for (int dz = -1; dz <= 1; dz++) {
        if (dx == 0 && dy == 0 && dz == 0) continue;
        int nz = z + dz;
        if (nz <= 0 || nz >= sizeZ - 1) continue;
        dataCell nc = data[nx][ny][nz];
        if (nc.sqdist != INT_MAX && !nc.needsRaise &&
            (nc.voronoi == voronoiKeep || nc.voronoi == voronoiPrune)) {
          nc.voronoi = free;
          data[nx][ny][nz] = nc;
          pruneQueue.push(INTPOINT3(nx, ny, nz));
        }
      }
    }
  }
}

void DynamicVoronoi3D::prune() {
  // Phase 1: Expand freeQueued state into the open queue and attempt to fill
  // thin non-Voronoi corridors that would otherwise disconnect the surface.
  while (!pruneQueue.empty()) {
    INTPOINT3 p = pruneQueue.front();
    pruneQueue.pop();
    int x = p.x, y = p.y, z = p.z;

    if (data[x][y][z].voronoi == occupied) continue;
    if (data[x][y][z].voronoi == freeQueued) continue;

    data[x][y][z].voronoi = freeQueued;
    open.push(data[x][y][z].sqdist, p);

    // For each of the 6 face directions, attempt to "fill" a thin non-Voronoi
    // cell that is sandwiched between Voronoi regions, preventing disconnection.
    // +x
    if (x + 2 < sizeX && data[x+1][y][z].voronoi == occupied) {
      dataCell& r  = data[x+1][y][z];
      if (data[x+1][y+1][z].voronoi != occupied &&
          data[x+1][y-1][z].voronoi != occupied &&
          data[x+1][y][z+1].voronoi != occupied &&
          data[x+1][y][z-1].voronoi != occupied &&
          data[x+2][y][z].voronoi   != occupied) {
        r.voronoi = freeQueued;
        open.push(r.sqdist, INTPOINT3(x+1, y, z));
        data[x+1][y][z] = r;
      }
    }
    // -x
    if (x - 2 >= 0 && data[x-1][y][z].voronoi == occupied) {
      dataCell& l  = data[x-1][y][z];
      if (data[x-1][y+1][z].voronoi != occupied &&
          data[x-1][y-1][z].voronoi != occupied &&
          data[x-1][y][z+1].voronoi != occupied &&
          data[x-1][y][z-1].voronoi != occupied &&
          data[x-2][y][z].voronoi   != occupied) {
        l.voronoi = freeQueued;
        open.push(l.sqdist, INTPOINT3(x-1, y, z));
        data[x-1][y][z] = l;
      }
    }
    // +y
    if (y + 2 < sizeY && data[x][y+1][z].voronoi == occupied) {
      dataCell& t  = data[x][y+1][z];
      if (data[x+1][y+1][z].voronoi != occupied &&
          data[x-1][y+1][z].voronoi != occupied &&
          data[x][y+1][z+1].voronoi != occupied &&
          data[x][y+1][z-1].voronoi != occupied &&
          data[x][y+2][z].voronoi   != occupied) {
        t.voronoi = freeQueued;
        open.push(t.sqdist, INTPOINT3(x, y+1, z));
        data[x][y+1][z] = t;
      }
    }
    // -y
    if (y - 2 >= 0 && data[x][y-1][z].voronoi == occupied) {
      dataCell& b  = data[x][y-1][z];
      if (data[x+1][y-1][z].voronoi != occupied &&
          data[x-1][y-1][z].voronoi != occupied &&
          data[x][y-1][z+1].voronoi != occupied &&
          data[x][y-1][z-1].voronoi != occupied &&
          data[x][y-2][z].voronoi   != occupied) {
        b.voronoi = freeQueued;
        open.push(b.sqdist, INTPOINT3(x, y-1, z));
        data[x][y-1][z] = b;
      }
    }
    // +z
    if (z + 2 < sizeZ && data[x][y][z+1].voronoi == occupied) {
      dataCell& f  = data[x][y][z+1];
      if (data[x+1][y][z+1].voronoi != occupied &&
          data[x-1][y][z+1].voronoi != occupied &&
          data[x][y+1][z+1].voronoi != occupied &&
          data[x][y-1][z+1].voronoi != occupied &&
          data[x][y][z+2].voronoi   != occupied) {
        f.voronoi = freeQueued;
        open.push(f.sqdist, INTPOINT3(x, y, z+1));
        data[x][y][z+1] = f;
      }
    }
    // -z
    if (z - 2 >= 0 && data[x][y][z-1].voronoi == occupied) {
      dataCell& bk = data[x][y][z-1];
      if (data[x+1][y][z-1].voronoi != occupied &&
          data[x-1][y][z-1].voronoi != occupied &&
          data[x][y+1][z-1].voronoi != occupied &&
          data[x][y-1][z-1].voronoi != occupied &&
          data[x][y][z-2].voronoi   != occupied) {
        bk.voronoi = freeQueued;
        open.push(bk.sqdist, INTPOINT3(x, y, z-1));
        data[x][y][z-1] = bk;
      }
    }
  }

  // Phase 2: Use markerMatch to decide which cells to keep, prune, or retry.
  while (!open.empty()) {
    INTPOINT3 p = open.pop();
    dataCell c = data[p.x][p.y][p.z];
    int v = c.voronoi;
    if (v != freeQueued && v != voronoiRetry) continue;

    markerMatchResult r = markerMatch(p.x, p.y, p.z);
    if (r == pruned)     c.voronoi = voronoiPrune;
    else if (r == keep)  c.voronoi = voronoiKeep;
    else {               // retry
      c.voronoi = voronoiRetry;
      pruneQueue.push(p);
    }
    data[p.x][p.y][p.z] = c;

    if (open.empty()) {
      while (!pruneQueue.empty()) {
        INTPOINT3 q = pruneQueue.front();
        pruneQueue.pop();
        open.push(data[q.x][q.y][q.z].sqdist, q);
      }
    }
  }
}

// Determine whether a Voronoi cell should be kept or pruned.
//
// In 3D the GVD is a 2D surface (not a 1D curve as in 2D), so we thin only
// the "solid interior" of thick surface regions.  A cell is pruned only when
// ALL 6 face-adjacent neighbors are also Voronoi — meaning it is strictly
// interior to a thick Voronoi block and can be removed without changing the
// surface topology.  All other cells (surface cells, edge cells, endpoints)
// are kept.
DynamicVoronoi3D::markerMatchResult DynamicVoronoi3D::markerMatch(int x, int y, int z) {
  static const int faceDirs[6][3] = {
    {-1,0,0},{1,0,0},{0,-1,0},{0,1,0},{0,0,-1},{0,0,1}
  };

  int voroCount6 = 0;
  for (int d = 0; d < 6; d++) {
    int nx = x + faceDirs[d][0];
    int ny = y + faceDirs[d][1];
    int nz = z + faceDirs[d][2];
    if (nx <= 0 || nx >= sizeX-1 || ny <= 0 || ny >= sizeY-1 || nz <= 0 || nz >= sizeZ-1)
      continue;
    int v = data[nx][ny][nz].voronoi;
    if (v <= free && v != voronoiPrune) voroCount6++;
  }

  // Keep endpoints, surface cells, and edge cells.
  // Only prune cells whose removal cannot expose a "hole" in the surface —
  // i.e., cells that are surrounded on all 6 faces by other Voronoi cells.
  if (voroCount6 < 6) return keep;
  return pruned;
}

void DynamicVoronoi3D::visualize(const char* filenameBase) {
  // Write one PPM file per z-slice: <filenameBase>_z<zzz>.ppm
  // Color scheme: red = Voronoi surface, black = obstacle, gray gradient = distance
  char filename[512];
  for (int z = 0; z < sizeZ; z++) {
    snprintf(filename, sizeof(filename), "%s_z%03d.ppm", filenameBase, z);
    FILE* F = fopen(filename, "w");
    if (!F) {
      std::cerr << "could not open '" << filename << "' for writing!\n";
      continue;
    }
    fprintf(F, "P6\n%d %d 255\n", sizeX, sizeY);
    for (int y = sizeY - 1; y >= 0; y--) {
      for (int xv = 0; xv < sizeX; xv++) {
        if (isVoronoi(xv, y, z)) {
          fputc(255, F); fputc(0, F); fputc(0, F);
        } else if (data[xv][y][z].sqdist == 0) {
          fputc(0, F); fputc(0, F); fputc(0, F);
        } else {
          float f = 80 + data[xv][y][z].dist * 5;
          if (f > 255) f = 255;
          if (f < 0)   f = 0;
          unsigned char c = (unsigned char)f;
          fputc(c, F); fputc(c, F); fputc(c, F);
        }
      }
    }
    fclose(F);
  }
}
