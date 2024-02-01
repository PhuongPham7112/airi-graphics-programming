/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code

*/

#include "jello.h"
#include "physics.h"
#include <vector>
/* Computes acceleration to every control point of the jello cube, 
   which is in state given by 'jello'.
   Returns result in array 'a'. */
void computeAcceleration(struct world * jello, struct point a[8][8][8])
{
  // 512 acceleration results for each point
  // a_ith at point ith = (F hook at ith + F damp at ith + F ext at ith) / mass 
  /* for you to implement ... */
    int maxIdx = 7;
    int minIdx = 0;
    for (int i = 0; i <= maxIdx; i++) {
        for (int j = 0; j <= maxIdx; j++) {
            for (int k = 0; k <= maxIdx; k++) { // at point [i][j][k]                
                // find sum
                point fTotal = point(0.0, 0.0, 0.0);

                point currentPoint = jello->p[i][j][k];
                point currentPointVelocity = jello->v[i][j][k];

                // find structural neightbors: 6 immediate neighbors
                std::vector<indexStruct> neighborIdx;
                if (i > minIdx) neighborIdx.push_back(indexStruct(i - 1, j, k));
                if (i < maxIdx) neighborIdx.push_back(indexStruct(i + 1, j, k));
                if (j > minIdx) neighborIdx.push_back(indexStruct(i, j - 1, k));
                if (j < maxIdx) neighborIdx.push_back(indexStruct(i, j + 1, k));
                if (k > minIdx) neighborIdx.push_back(indexStruct(i, j, k - 1));
                if (k < maxIdx) neighborIdx.push_back(indexStruct(i, j, k + 1));
                for (indexStruct idx : neighborIdx) {
                    point neighbor = jello->p[idx.x][idx.y][idx.z];
                    point neighborVelocity = jello->v[idx.x][idx.y][idx.z];

                    // calculate hook force
                    pSUM(fTotal, HookLaw(jello->kElastic, jello->unrestLength, currentPoint, neighbor), fTotal);
                    // calculate damp force
                    pSUM(fTotal, Damping(jello->dElastic, currentPoint, neighbor, currentPointVelocity, neighborVelocity), fTotal);
                }
                neighborIdx.clear();

                // find shear neighbors
                if (i < maxIdx) {
                    if (j < maxIdx) neighborIdx.push_back(indexStruct(i + 1, j + 1, k));
                    if (j > minIdx) neighborIdx.push_back(indexStruct(i + 1, j - 1, k));
                    if (k > minIdx) neighborIdx.push_back(indexStruct(i + 1, j, k - 1));
                    if (k < maxIdx) neighborIdx.push_back(indexStruct(i + 1, j, k + 1));
                }
                if (i > minIdx) {
                    if (j < maxIdx) neighborIdx.push_back(indexStruct(i - 1, j + 1, k));
                    if (j > minIdx) neighborIdx.push_back(indexStruct(i - 1, j - 1, k));
                    if (k > minIdx) neighborIdx.push_back(indexStruct(i - 1, j, k - 1));
                    if (k < maxIdx) neighborIdx.push_back(indexStruct(i - 1, j, k + 1));
                }
                if (j < maxIdx) {
                    if (k < maxIdx) neighborIdx.push_back(indexStruct(i, j + 1, k + 1));
                    if (k > minIdx) neighborIdx.push_back(indexStruct(i, j + 1, k - 1));
                }
                if (j > minIdx) {
                    if (k < maxIdx) neighborIdx.push_back(indexStruct(i, j - 1, k + 1));
                    if (k > minIdx) neighborIdx.push_back(indexStruct(i, j - 1, k - 1));
                }

                for (indexStruct idx : neighborIdx) {
                    point neighbor = jello->p[idx.x][idx.y][idx.z];
                    point neighborVelocity = jello->v[idx.x][idx.y][idx.z];

                    // calculate hook force
                    pSUM(fTotal, HookLaw(jello->kElastic, jello->unrestLengthShear, currentPoint, neighbor), fTotal);
                    // calculate damp force
                    pSUM(fTotal, Damping(jello->dElastic, currentPoint, neighbor, currentPointVelocity, neighborVelocity), fTotal);
                }
                neighborIdx.clear();
                
                // DEBUG: shear diagonal
                if (i + 1 <= maxIdx) {
                    if (j + 1 <= maxIdx) {
                        if (k + 1 <= maxIdx) neighborIdx.push_back(indexStruct(i + 1, j + 1, k + 1));
                        if (k - 1 >= minIdx) neighborIdx.push_back(indexStruct(i + 1, j + 1, k - 1));
                    }

                    if (j - 1 >= minIdx) {
                        if (k + 1 <= maxIdx) neighborIdx.push_back(indexStruct(i + 1, j - 1, k + 1));
                        if (k - 1 >= minIdx) neighborIdx.push_back(indexStruct(i + 1, j - 1, k - 1));
                    }
                }
                if (i - 1 >= minIdx) {
                    if (j + 1 <= maxIdx) {
                        if (k + 1 <= maxIdx) neighborIdx.push_back(indexStruct(i - 1, j + 1, k + 1));
                        if (k - 1 >= minIdx) neighborIdx.push_back(indexStruct(i - 1, j + 1, k - 1));
                    }

                    if (j - 1 >= minIdx) {
                        if (k + 1 <= maxIdx) neighborIdx.push_back(indexStruct(i - 1, j - 1, k + 1));
                        if (k - 1 >= minIdx) neighborIdx.push_back(indexStruct(i - 1, j - 1, k - 1));
                    }
                }

                for (indexStruct idx : neighborIdx) {
                    point neighbor = jello->p[idx.x][idx.y][idx.z];
                    point neighborVelocity = jello->v[idx.x][idx.y][idx.z];

                    // calculate hook force
                    pSUM(fTotal, HookLaw(jello->kElastic, jello->unrestLengthShearDiag, currentPoint, neighbor), fTotal);
                    // calculate damp force
                    pSUM(fTotal, Damping(jello->dElastic, currentPoint, neighbor, currentPointVelocity, neighborVelocity), fTotal);
                }
                neighborIdx.clear();

                // find bending neighbors
                if (i > minIdx + 1) neighborIdx.push_back(indexStruct(i - 2, j, k));
                if (i < maxIdx - 1) neighborIdx.push_back(indexStruct(i + 2, j, k));
                if (j > minIdx + 1) neighborIdx.push_back(indexStruct(i, j - 2, k));
                if (j < maxIdx - 1) neighborIdx.push_back(indexStruct(i, j + 2, k));
                if (k > minIdx + 1) neighborIdx.push_back(indexStruct(i, j, k - 2));
                if (k < maxIdx - 1) neighborIdx.push_back(indexStruct(i, j, k + 2));
                for (indexStruct idx : neighborIdx) {
                    point neighbor = jello->p[idx.x][idx.y][idx.z];
                    point neighborVelocity = jello->v[idx.x][idx.y][idx.z];

                    // calculate hook force
                    pSUM(fTotal, HookLaw(jello->kElastic, jello->unrestLengthBend, currentPoint, neighbor), fTotal);
                    // calculate damp force
                    pSUM(fTotal, Damping(jello->dElastic, currentPoint, neighbor, currentPointVelocity, neighborVelocity), fTotal);
                }
                neighborIdx.clear();

                 //calculate collision springs forces
                for (int f = 0; f < 4; f++) {
                    point fCollision = point();
                    plane face = jello->box[f];
                    point product;
                    point diff;
                    point normalizedDiff;
                    point collisionContact;
                    double dot;
                    double length;
                    pDIFFERENCE(currentPoint, face.pOnPlane, diff);
                    normalizedDiff = diff;
                    pNORMALIZE(normalizedDiff);
                    dot = DOTPRODUCTp(face.normal, diff, dot);

                    if (dot < 0.0) { // past the plane
                        double penetration;
                        point penetrationForce;
                        point invNormal = point(-face.normal.x, -face.normal.y, -face.normal.z);
                        penetration = DOTPRODUCTp(invNormal, diff, penetration);
                        pMULTIPLY(face.normal, penetration, penetrationForce);
                        pDIFFERENCE(currentPoint, penetrationForce, collisionContact);

                        point fCollisionHook = HookLaw(jello->kCollision, 0.0, collisionContact, currentPoint);
                        point fCollisionDamp = Damping(jello->dCollision, collisionContact, currentPoint, penetrationForce, currentPointVelocity);

                        pSUM(fCollisionHook, fCollision, fCollision);
                        pSUM(fCollisionDamp, fCollision, fCollision);
                    }
                    pSUM(fCollision, fTotal, fTotal);
                }

                // calculate external force field
                calculateExternalForce(jello, i, j, k, fTotal);

                // a = F / m
                pMULTIPLY(fTotal, (1.0 / jello->mass), a[i][j][k]);
            }
        }
    }
}

// Calculate external force field, add it to point p at (i, j, k)
// src: https://www.scratchapixel.com/lessons/mathematics-physics-for-computer-graphics/interpolation/trilinear-interpolation.html
void calculateExternalForce(world* jello, int x, int y, int z, point& a) {
    // External force index in resolution array
    int i, j, k;

    // Forces at 8 corners in a specific grid surrounding the point p
    point f000, f001;
    point f010, f011;
    point f100, f101;
    point f110, f111;
    // position in cell
    double px, py, pz;
    // External force field value
    point externalForce;
    externalForce.x = 0;
    externalForce.y = 0;
    externalForce.z = 0;


    // 4 / cube_nums = 4 / (resolution - 1)
    double cube_h = jello->boxSize / (jello->resolution - 1);
    double cube_h_inv = 1.0 / cube_h;

    // index of the cell inside force field, origin (-2, -2, -2)
    i = (int)((jello->p[x][y][z].x + 2) / cube_h); // pos.x / h
    j = (int)((jello->p[x][y][z].y + 2) / cube_h); // pos.y / h
    k = (int)((jello->p[x][y][z].z + 2) / cube_h); // pos.z / h
    
    // Check if the index is at the wall of the bounding box
    if (i == (jello->resolution - 1)) {
        i--;
    }
    else if (i < 0) {
        i = 0;
    }
    if (j == (jello->resolution - 1)) {
        j--;
    }
    else if (j < 0) {
        j = 0;
    }
    if (k == (jello->resolution - 1)) {
        k--;
    }
    else if (k < 0) {
        k = 0;
    }

    // Check if the point is inside the bounding box & read the force field value
    if (((i >= 0) && (i <= jello->resolution - 1)) && ((j >= 0) && (j <= jello->resolution - 1)) && ((k >= 0) && (k <= jello->resolution - 1))) {
        f000 = jello->forceField[(i * jello->resolution * jello->resolution + j * jello->resolution + k)];
        f001 = jello->forceField[(i * jello->resolution * jello->resolution + j * jello->resolution + (k + 1))];

        f010 = jello->forceField[(i * jello->resolution * jello->resolution + (j + 1) * jello->resolution + k)];
        f011 = jello->forceField[(i * jello->resolution * jello->resolution + (j + 1) * jello->resolution + (k + 1))];

        f100 = jello->forceField[((i + 1) * jello->resolution * jello->resolution + j * jello->resolution + k)];
        f101 = jello->forceField[((i + 1) * jello->resolution * jello->resolution + j * jello->resolution + (k + 1))];

        f110 = jello->forceField[((i + 1) * jello->resolution * jello->resolution + (j + 1) * jello->resolution + k)];
        f111 = jello->forceField[((i + 1) * jello->resolution * jello->resolution + (j + 1) * jello->resolution + (k + 1))];

        // 3D interpolation, find coefficient, think of these as weights
        // alpha, beta, gamma coefficients
        // where is this point in the cell, given that it's inside cell (i, j, k)
        double x0 = -2 + cube_h * i;
        double y0 = -2 + cube_h * j;
        double z0 = -2 + cube_h * k;
        px = (jello->p[x][y][z].x - x0) / cube_h; // (x - x0) / len
        py = (jello->p[x][y][z].y - y0) / cube_h; // (y - y0) / len
        pz = (jello->p[x][y][z].z - z0) / cube_h; // (z - z0) / len

        pMULTIPLY(f000, (1 - px) * (1 - py) * (1 - pz), f000);
        pSUM(a, f000, a);
        pMULTIPLY(f001, (1 - px) * (1 - py) * pz, f001);
        pSUM(a, f001, a);
        pMULTIPLY(f010, (1 - px) * py * (1 - pz), f010);
        pSUM(a, f010, a);
        pMULTIPLY(f011, (1 - px) * py * pz, f011);
        pSUM(a, f011, a);
        pMULTIPLY(f100, px * (1 - py) * (1 - pz), f100);
        pSUM(a, f100, a);
        pMULTIPLY(f101, px * (1 - py) * pz, f101);
        pSUM(a, f101, a);
        pMULTIPLY(f110, px * py * (1 - pz), f110);
        pSUM(a, f110, a);
        pMULTIPLY(f111, px * py * pz, f111);
        pSUM(a, f111, a);
    }
}

point HookLaw(double kHook, double restLength, point A, point B) {
    // F = (-kH * (|L| - rL)) * (L/|L|)
    point fHook;
    point L;
    pDIFFERENCE(A, B, L);
    point normL;
    pCPY(L, normL);
    double length;
    pNORMALIZE(normL);
    pMULTIPLY(normL, -kHook * (length - restLength), fHook);
    return fHook;
}

point Damping(double kDamp, point A, point B, point velA, point velB) {
    // F = (-kD * ((vA-vB) dot L) / |L|) * (L/|L|)
    point fDamp;
    point L; // a - b = L
    pDIFFERENCE(A, B, L);
    point velDiff; // vel a - vel b
    pDIFFERENCE(velA, velB, velDiff);
    point normL; // normalize vector L
    pCPY(L, normL);
    double length;
    pNORMALIZE(normL);
    double product; // L dot velDiff
    DOTPRODUCTp(L, velDiff, product);
    product *= (-kDamp) / length;
    pMULTIPLY(normL, product, fDamp);
    return fDamp;
}

double distance(point A, point B) {
    return sqrt(pow(A.x - B.x, 2) + pow(A.y - B.y, 2) + pow(A.z - B.z, 2));
}

/* performs one step of Euler Integration */
/* as a result, updates the jello structure */
void Euler(struct world * jello)
{
  int i,j,k;
  point a[8][8][8];

  computeAcceleration(jello, a);
  
  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
        jello->p[i][j][k].x += jello->dt * jello->v[i][j][k].x;
        jello->p[i][j][k].y += jello->dt * jello->v[i][j][k].y;
        jello->p[i][j][k].z += jello->dt * jello->v[i][j][k].z;
        jello->v[i][j][k].x += jello->dt * a[i][j][k].x;
        jello->v[i][j][k].y += jello->dt * a[i][j][k].y;
        jello->v[i][j][k].z += jello->dt * a[i][j][k].z;

      }
}

/* performs one step of RK4 Integration */
/* as a result, updates the jello structure */
void RK4(struct world * jello)
{
  point F1p[8][8][8], F1v[8][8][8], 
        F2p[8][8][8], F2v[8][8][8],
        F3p[8][8][8], F3v[8][8][8],
        F4p[8][8][8], F4v[8][8][8];

  point a[8][8][8];


  struct world buffer;

  int i,j,k;

  buffer = *jello; // make a copy of jello

  computeAcceleration(jello, a);

  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         pMULTIPLY(jello->v[i][j][k],jello->dt,F1p[i][j][k]);
         pMULTIPLY(a[i][j][k],jello->dt,F1v[i][j][k]);
         pMULTIPLY(F1p[i][j][k],0.5,buffer.p[i][j][k]);
         pMULTIPLY(F1v[i][j][k],0.5,buffer.v[i][j][k]);
         pSUM(jello->p[i][j][k],buffer.p[i][j][k],buffer.p[i][j][k]);
         pSUM(jello->v[i][j][k],buffer.v[i][j][k],buffer.v[i][j][k]);
      }

  computeAcceleration(&buffer, a);

  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         // F2p = dt * buffer.v;
         pMULTIPLY(buffer.v[i][j][k],jello->dt,F2p[i][j][k]);
         // F2v = dt * a(buffer.p,buffer.v);     
         pMULTIPLY(a[i][j][k],jello->dt,F2v[i][j][k]);
         pMULTIPLY(F2p[i][j][k],0.5,buffer.p[i][j][k]);
         pMULTIPLY(F2v[i][j][k],0.5,buffer.v[i][j][k]);
         pSUM(jello->p[i][j][k],buffer.p[i][j][k],buffer.p[i][j][k]);
         pSUM(jello->v[i][j][k],buffer.v[i][j][k],buffer.v[i][j][k]);
      }

  computeAcceleration(&buffer, a);

  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         // F3p = dt * buffer.v;
         pMULTIPLY(buffer.v[i][j][k],jello->dt,F3p[i][j][k]);
         // F3v = dt * a(buffer.p,buffer.v);     
         pMULTIPLY(a[i][j][k],jello->dt,F3v[i][j][k]);
         pMULTIPLY(F3p[i][j][k],1.0,buffer.p[i][j][k]);
         pMULTIPLY(F3v[i][j][k],1.0,buffer.v[i][j][k]);
         pSUM(jello->p[i][j][k],buffer.p[i][j][k],buffer.p[i][j][k]);
         pSUM(jello->v[i][j][k],buffer.v[i][j][k],buffer.v[i][j][k]);
      }
         
  computeAcceleration(&buffer, a);


  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         // F3p = dt * buffer.v;
         pMULTIPLY(buffer.v[i][j][k],jello->dt,F4p[i][j][k]);
         // F3v = dt * a(buffer.p,buffer.v);     
         pMULTIPLY(a[i][j][k],jello->dt,F4v[i][j][k]);

         pMULTIPLY(F2p[i][j][k],2,buffer.p[i][j][k]);
         pMULTIPLY(F3p[i][j][k],2,buffer.v[i][j][k]);
         pSUM(buffer.p[i][j][k],buffer.v[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F1p[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F4p[i][j][k],buffer.p[i][j][k]);
         pMULTIPLY(buffer.p[i][j][k],1.0 / 6,buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],jello->p[i][j][k],jello->p[i][j][k]);

         pMULTIPLY(F2v[i][j][k],2,buffer.p[i][j][k]);
         pMULTIPLY(F3v[i][j][k],2,buffer.v[i][j][k]);
         pSUM(buffer.p[i][j][k],buffer.v[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F1v[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F4v[i][j][k],buffer.p[i][j][k]);
         pMULTIPLY(buffer.p[i][j][k],1.0 / 6,buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],jello->v[i][j][k],jello->v[i][j][k]);
      }

  return;  
}
