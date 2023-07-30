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
  /* for you to implement ... */
    int maxIdx = 7;
    int minIdx = 0;
    for (int i = 0; i <= maxIdx; i++) {
        for (int j = 0; j <= maxIdx; j++) {
            for (int k = 0; k <= maxIdx; k++) {
                // find structural neightbors: 6 immediate neighbors
                std::vector<point> structNeighbors;
                if (i > minIdx) structNeighbors.push_back(jello->p[i-1][j][k]);
                if (i < maxIdx) structNeighbors.push_back(jello->p[i+1][j][k]);
                if (j > minIdx) structNeighbors.push_back(jello->p[i][j-1][k]);
                if (j < maxIdx) structNeighbors.push_back(jello->p[i][j+1][k]);
                if (k > minIdx) structNeighbors.push_back(jello->p[i][j][k-1]);
                if (k < maxIdx) structNeighbors.push_back(jello->p[i][j][k+1]);
                // find shear neighbors: 
                std::vector<point> shearNeighbors;
                shearNeighbors.push_back(jello->p[i+1][j+1][k+1]);
                shearNeighbors.push_back(jello->p[i+1][j+1][k-1]);
                shearNeighbors.push_back(jello->p[i+1][j-1][k+1]);
                shearNeighbors.push_back(jello->p[i+1][j-1][k-1]);
                shearNeighbors.push_back(jello->p[i-1][j+1][k+1]);
                shearNeighbors.push_back(jello->p[i-1][j+1][k-1]);
                shearNeighbors.push_back(jello->p[i-1][j-1][k-1]);
                shearNeighbors.push_back(jello->p[i-1][j-1][k+1]);
                // find bending neighbors
                std::vector<point> bendNeighbors;
                if (i > minIdx + 1) bendNeighbors.push_back(jello->p[i - 2][j][k]);
                if (i < maxIdx - 1) bendNeighbors.push_back(jello->p[i + 2][j][k]);
                if (j > minIdx + 1) bendNeighbors.push_back(jello->p[i][j - 2][k]);
                if (j < maxIdx - 1) bendNeighbors.push_back(jello->p[i][j + 2][k]);
                if (k > minIdx + 1) bendNeighbors.push_back(jello->p[i][j][k - 2]);
                if (k < maxIdx - 1) bendNeighbors.push_back(jello->p[i][j][k + 2]);
                // calculate hook force of shear, structural, and bend
                point fHook;
                pMAKE(0.0, 0.0, 0.0, fHook);
                // calculate damp force of shear, structural, and bend
                point fDamp;
                pMAKE(0.0, 0.0, 0.0, fDamp);
                // calculate external force
                point fExtern;
                pMAKE(0.0, 0.0, 0.0, fExtern);

                point fTotal;
                pMAKE(0.0, 0.0, 0.0, fTotal);
                pSUM(fTotal, fHook, fTotal);
                pSUM(fTotal, fDamp, fTotal);
                pSUM(fTotal, fExtern, fTotal);

                // a = F / m
                //pDIVIDE(fTotal, jello->mass, a);
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
