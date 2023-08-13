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
                point currentPoint = jello->p[i][j][k];
                for (std::vector<point> face : jello->box) {
                    for (point faceP : face) {

                    }
                }
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
                for (indexStruct idx : neighborIdx) {
                    point neighbor = jello->p[idx.x][idx.y][idx.z];
                    point neighborVelocity = jello->v[idx.x][idx.y][idx.z];
                    double restLength = distance(jello->up[idx.x][idx.y][idx.z], jello->up[i][j][k]); // how to store rest length in undeformed state

                    // calculate hook force
                    point fHook;
                    point hook = HookLaw(jello->kElastic, restLength, neighbor, currentPoint);
                    pSUM(fHook, hook, fHook);

                    // calculate damp force
                    point fDamp;
                    point damp = Damping(jello->dElastic, neighbor, currentPoint, neighborVelocity, currentPointVelocity);
                    pSUM(fDamp, damp, fDamp);

                    // calculate external force
                    point fExtern;

                    // calculate total
                    point fTotal;
                    pMAKE(0.0, 0.0, 0.0, fTotal);
                    pSUM(fTotal, fHook, fTotal);
                    pSUM(fTotal, fDamp, fTotal);
                    pSUM(fTotal, fExtern, fTotal);

                    // a = F / m
                    pDIVIDE(fTotal, jello->mass, a[i][j][k]);
                }         
            }
        }
    }
}

point HookLaw(double kHook, double restLength, point A, point B) {
    point vec;
    pDIFFERENCE(A, B, vec);
    double length;
    pNORMALIZE(vec);
    pMULTIPLY(vec, -kHook * (length - restLength), vec);
    return vec;
}

point Damping(double kDamp, point A, point B, point velA, point velB) {
    point L; // a - b = L
    pDIFFERENCE(A, B, L);

    point velDiff; // vel a - vel b
    pDIFFERENCE(velA, velB, velDiff);

    point product;
    DOTPRODUCTp(L, velDiff, product);

    double length;
    pNORMALIZE(L);

    point med1;
    pDIVIDE(product, length, med1);

    point med2;
    DOTPRODUCTp(med1, L, med2);
    
    point fDamp;
    pMULTIPLY(med2, kDamp, fDamp);

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
