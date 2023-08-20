/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code

*/

#include "jello.h"
#include "physics.h"
#include <vector>
#include <iostream>
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
                point currentPoint = jello->p[i][j][k];
                point currentPointVelocity = jello->v[i][j][k];

                std::vector<indexStruct> neighborIdx;
                // find structural neightbors: 6 immediate neighbors
                if (i > minIdx) neighborIdx.push_back(indexStruct(i - 1, j, k));
                if (i < maxIdx) neighborIdx.push_back(indexStruct(i + 1, j, k));
                if (j > minIdx) neighborIdx.push_back(indexStruct(i, j - 1, k));
                if (j < maxIdx) neighborIdx.push_back(indexStruct(i, j + 1, k));
                if (k > minIdx) neighborIdx.push_back(indexStruct(i, j, k - 1));
                if (k < maxIdx) neighborIdx.push_back(indexStruct(i, j, k + 1));
                // find shear neighbors: 
                // xy plane
                if (i < maxIdx && j < maxIdx) neighborIdx.push_back(indexStruct(i + 1, j + 1, k));
                if (i < maxIdx && j > minIdx) neighborIdx.push_back(indexStruct(i + 1, j - 1, k));
                if (i > minIdx && j < maxIdx) neighborIdx.push_back(indexStruct(i - 1, j + 1, k));
                if (i > minIdx && j > minIdx) neighborIdx.push_back(indexStruct(i - 1, j - 1, k));
                // yz plane
                if (j < maxIdx && k < maxIdx) neighborIdx.push_back(indexStruct(i, j + 1, k + 1));
                if (j > minIdx && k < maxIdx) neighborIdx.push_back(indexStruct(i, j - 1, k + 1));
                if (j < maxIdx && k > minIdx) neighborIdx.push_back(indexStruct(i, j + 1, k - 1));
                if (j > minIdx && k > minIdx) neighborIdx.push_back(indexStruct(i, j - 1, k - 1));
                // xz plane
                if (i < maxIdx && k > minIdx) neighborIdx.push_back(indexStruct(i + 1, j, k - 1));
                if (i < maxIdx && k < maxIdx) neighborIdx.push_back(indexStruct(i + 1, j, k + 1));
                if (i > minIdx && k > minIdx) neighborIdx.push_back(indexStruct(i - 1, j, k - 1));
                if (i > minIdx && k < maxIdx) neighborIdx.push_back(indexStruct(i - 1, j, k + 1));
                // diagonal
                if (i < maxIdx && j < maxIdx && k < maxIdx) neighborIdx.push_back(indexStruct(i + 1, j + 1, k + 1));
                if (i < maxIdx && j < maxIdx && k > minIdx) neighborIdx.push_back(indexStruct(i + 1, j + 1, k - 1));
                if (i < maxIdx && j > minIdx && k < maxIdx) neighborIdx.push_back(indexStruct(i + 1, j - 1, k + 1));
                if (i < maxIdx && j > minIdx && k > minIdx) neighborIdx.push_back(indexStruct(i + 1, j - 1, k - 1));
                if (i > minIdx && j < maxIdx && k < maxIdx) neighborIdx.push_back(indexStruct(i - 1, j + 1, k + 1));
                if (i > minIdx && j < maxIdx && k > minIdx) neighborIdx.push_back(indexStruct(i - 1, j + 1, k - 1));
                if (i > minIdx && j > minIdx && k > minIdx) neighborIdx.push_back(indexStruct(i - 1, j - 1, k - 1));
                if (i > minIdx && j > minIdx && k < maxIdx) neighborIdx.push_back(indexStruct(i - 1, j - 1, k + 1));
                // find bending neighbors
                if (i > minIdx + 1) neighborIdx.push_back(indexStruct(i - 2, j, k));
                if (i < maxIdx - 1) neighborIdx.push_back(indexStruct(i + 2, j, k));
                if (j > minIdx + 1) neighborIdx.push_back(indexStruct(i, j - 2, k));
                if (j < maxIdx - 1) neighborIdx.push_back(indexStruct(i, j + 2, k));
                if (k > minIdx + 1) neighborIdx.push_back(indexStruct(i, j, k - 2));
                if (k < maxIdx - 1) neighborIdx.push_back(indexStruct(i, j, k + 2));
                
                // find sum
                point fTotal = point();
                point fHook = point();
                point fDamp = point();
                
                for (indexStruct idx : neighborIdx) {
                    point neighbor = jello->p[idx.x][idx.y][idx.z];
                    point neighborVelocity = jello->v[idx.x][idx.y][idx.z];
                    double restLength = distance(jello->up[idx.x][idx.y][idx.z], jello->up[i][j][k]); // rest length in undeformed state

                    // calculate hook force
                    pSUM(fHook, HookLaw(jello->kElastic, restLength, currentPoint, neighbor), fHook);

                    // calculate damp force
                    pSUM(fDamp, Damping(jello->dElastic, currentPoint, neighbor, currentPointVelocity, neighborVelocity), fDamp);
                }
                // calculate total of spring forces
                pSUM(fTotal, fHook, fTotal);
                pSUM(fTotal, fDamp, fTotal);

                // calculate collision springs forces
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
                point fExtern = point();
                calculateExternalForce(jello, i, j, k, fExtern);
       
                pSUM(fExtern, fTotal, fTotal);

                // a = F / m
                pMULTIPLY(fTotal, (1.0 / jello->mass), a[i][j][k]);
            }
        }
    }
}

// Calculate external force field, add it to point p at (i, j, k) 
void calculateExternalForce(world* jello, int x, int y, int z, point& a) {
    // External force index in resolution array
    int i, j, k;
    // Forces at 8 corners in a specific grid surrounding the point p
    point f000, f001;
    point f010, f011;
    point f100, f101;
    point f110, f111;
    // External force position in grid
    double px, py, pz;
    // Force field grid
    double grid;
    // External force field value
    point externalForce;
    externalForce.x = 0;
    externalForce.y = 0;
    externalForce.z = 0;

    i = int((jello->p[x][y][z].x + 2) * (jello->resolution - 1) / 4);
    j = int((jello->p[x][y][z].y + 2) * (jello->resolution - 1) / 4);
    k = int((jello->p[x][y][z].z + 2) * (jello->resolution - 1) / 4);

    // Check if the index is at the wall of the bounding box
    if (i == (jello->resolution - 1)) {
        i--;
    }
    if (j == (jello->resolution - 1)) {
        j--;
    }
    if (k == (jello->resolution - 1)) {
        k--;
    }
    // Check if the point is inside the bounding box, read the force field value
    if (((i >= 0) && (i <= jello->resolution - 1)) && ((j >= 0) && (j <= jello->resolution - 1)) && ((j >= 0) && (j <= jello->resolution - 1))) {
        f000 = jello->forceField[(i * jello->resolution * jello->resolution + j * jello->resolution + k)];
        f001 = jello->forceField[(i * jello->resolution * jello->resolution + j * jello->resolution + (k + 1))];

        f010 = jello->forceField[(i * jello->resolution * jello->resolution + (j + 1) * jello->resolution + k)];
        f011 = jello->forceField[(i * jello->resolution * jello->resolution + (j + 1) * jello->resolution + (k + 1))];

        f100 = jello->forceField[((i + 1) * jello->resolution * jello->resolution + j * jello->resolution + k)];
        f101 = jello->forceField[((i + 1) * jello->resolution * jello->resolution + j * jello->resolution + (k + 1))];

        f110 = jello->forceField[((i + 1) * jello->resolution * jello->resolution + (j + 1) * jello->resolution + k)];
        f111 = jello->forceField[((i + 1) * jello->resolution * jello->resolution + (j + 1) * jello->resolution + (k + 1))];

        // 3D interpolation
        grid = 1.0 * 4 / (jello->resolution - 1);
        px = (jello->p[x][y][z].x - (-2 + 1.0 * 4 * i / (jello->resolution - 1))) / grid;
        py = (jello->p[x][y][z].y - (-2 + 1.0 * 4 * j / (jello->resolution - 1))) / grid;
        pz = (jello->p[x][y][z].z - (-2 + 1.0 * 4 * k / (jello->resolution - 1))) / grid;

        pMULTIPLY(f000, (1 - px) * (1 - py) * (1 - pz), f000);
        pMULTIPLY(f001, (1 - px) * (1 - py) * pz, f001);
        pMULTIPLY(f010, (1 - px) * py * (1 - pz), f010);
        pMULTIPLY(f011, (1 - px) * py * pz, f011);
        pMULTIPLY(f100, px * (1 - py) * (1 - pz), f100);
        pMULTIPLY(f101, px * (1 - py) * pz, f101);
        pMULTIPLY(f110, px * py * (1 - pz), f110);
        pMULTIPLY(f111, px * py * pz, f111);

        pSUM(externalForce, f000, externalForce);
        pSUM(externalForce, f001, externalForce);
        pSUM(externalForce, f010, externalForce);
        pSUM(externalForce, f011, externalForce);
        pSUM(externalForce, f100, externalForce);
        pSUM(externalForce, f101, externalForce);
        pSUM(externalForce, f110, externalForce);
        pSUM(externalForce, f111, externalForce);
        a.x = externalForce.x;
        a.y = externalForce.y;
        a.z = externalForce.z;
    }
}

point HookLaw(double kHook, double restLength, point A, point B) {
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
    point fDamp;
    point L; // a - b = L
    pDIFFERENCE(A, B, L);
    point velDiff; // vel a - vel b
    pDIFFERENCE(velA, velB, velDiff);
    point normL;
    pCPY(L, normL);
    double length;
    pNORMALIZE(normL);
    double product; // L . velDiff
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
