/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code

*/

#include "jello.h"
#include "physics.h"
#include <iostream>
#include <vector>

// source: https://www.cs.uaf.edu/2009/fall/cs301/lecture/12_09_float_to_int.html
union float2int
{
    int i;
    float f;
};

int convert(float val) {
    float2int unholy;
    unholy.f = val + (1 << 23); /* scrape off the fraction bits with the weird constant */
    return unholy.i & 0x7FffFF; /* mask off the float's sign and exponent bits */
}

point calculateHookLaw(double kHook, double restLength, point A, point B) {
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

point calculateDamping(double kDamp, point A, point B, point velA, point velB) {
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

point calculateExternalForce(world* jello, int x, int y, int z) {
    // External force index in resolution array
    int i, j, k;
    double cubeSize = jello->boxSize / (jello->resolution - 1.0);
    double cubeSizeInv = 1.0 / cubeSize;
    point fExternal = point();

    // Forces at 8 corners in a specific grid surrounding the point p
    point f000, f001;
    point f010, f011;
    point f100, f101;
    point f110, f111;

    // index of the cell inside force field, origin (-2, -2, -2)
    i = convert((jello->p[x][y][z].x + 2.0) * cubeSizeInv); // pos.x / h
    j = convert((jello->p[x][y][z].y + 2.0) * cubeSizeInv); // pos.y / h
    k = convert((jello->p[x][y][z].z + 2.0) * cubeSizeInv); // pos.z / h

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

        // 3D interpolation, find coefficient, think of these as weights: alpha, beta, gamma coefficients
        double x0 = -2.0 + cubeSize * i;
        double y0 = -2.0 + cubeSize * j;
        double z0 = -2.0 + cubeSize * k;
        double px = (jello->p[x][y][z].x - x0) * cubeSizeInv; // (x - x0) / len
        double py = (jello->p[x][y][z].y - y0) * cubeSizeInv; // (y - y0) / len
        double pz = (jello->p[x][y][z].z - z0) * cubeSizeInv; // (z - z0) / len

        pMULTIPLY(f000, (1.0 - px) * (1.0 - py) * (1.0 - pz), f000);
        pSUM(fExternal, f000, fExternal);

        pMULTIPLY(f001, (1.0 - px) * (1.0 - py) * pz, f001);
        pSUM(fExternal, f001, fExternal);

        pMULTIPLY(f010, (1.0 - px) * py * (1.0 - pz), f010);
        pSUM(fExternal, f010, fExternal);

        pMULTIPLY(f011, (1.0 - px) * py * pz, f011);
        pSUM(fExternal, f011, fExternal);

        pMULTIPLY(f100, px * (1.0 - py) * (1.0 - pz), f100);
        pSUM(fExternal, f100, fExternal);

        pMULTIPLY(f101, px * (1.0 - py) * pz, f101);
        pSUM(fExternal, f101, fExternal);

        pMULTIPLY(f110, px * py * (1.0 - pz), f110);
        pSUM(fExternal, f110, fExternal);

        pMULTIPLY(f111, px * py * pz, f111);
        pSUM(fExternal, f111, fExternal);
    }
    return fExternal;
}

/* Computes acceleration to every control point of the jello cube,
   which is in state given by 'jello'.
   Returns result in array 'a'. */
void computeAcceleration(struct world* jello, struct point a[8][8][8])
{
    // a_ith at point ith = (F hook at ith + F damp at ith + F ext at ith) / mass 
    int maxIdx = 7;
    int minIdx = 0;
    point currentPoint;
    point currentPointVelocity;
    point neighbor;
    point neighborVelocity;
    for (int i = minIdx; i <= maxIdx; i++) {
        for (int j = minIdx; j <= maxIdx; j++) {
            for (int k = minIdx; k <= maxIdx; k++) { // at point [i][j][k]                
                // find sum
                point fTotal = point(0.0, 0.0, 0.0);
                currentPoint = jello->p[i][j][k];
                currentPointVelocity = jello->v[i][j][k];

                // structural neightbors: 6 immediate neighbors
                if (i > minIdx) // neighborIdx.push_back(indexStruct(i - 1, j, k));
                {
                    neighbor = jello->p[i - 1][j][k];
                    neighborVelocity = jello->v[i - 1][j][k];
                    // calculate hook force
                    pSUM(fTotal, calculateHookLaw(jello->kElastic, jello->unrestLength, currentPoint, neighbor), fTotal);
                    // calculate damp force
                    pSUM(fTotal, calculateDamping(jello->dElastic, currentPoint, neighbor, currentPointVelocity, neighborVelocity), fTotal);
                }
                if (i < maxIdx) // neighborIdx.push_back(indexStruct(i + 1, j, k));
                {
                    neighbor = jello->p[i + 1][j][k];
                    neighborVelocity = jello->v[i + 1][j][k];
                    // calculate hook force
                    pSUM(fTotal, calculateHookLaw(jello->kElastic, jello->unrestLength, currentPoint, neighbor), fTotal);
                    // calculate damp force
                    pSUM(fTotal, calculateDamping(jello->dElastic, currentPoint, neighbor, currentPointVelocity, neighborVelocity), fTotal);
                }
                if (j > minIdx) // neighborIdx.push_back(indexStruct(i, j - 1, k));
                {
                    neighbor = jello->p[i][j - 1][k];
                    neighborVelocity = jello->v[i][j - 1][k];
                    // calculate hook force
                    pSUM(fTotal, calculateHookLaw(jello->kElastic, jello->unrestLength, currentPoint, neighbor), fTotal);
                    // calculate damp force
                    pSUM(fTotal, calculateDamping(jello->dElastic, currentPoint, neighbor, currentPointVelocity, neighborVelocity), fTotal);
                }
                if (j < maxIdx) // neighborIdx.push_back(indexStruct(i, j + 1, k));
                {
                    neighbor = jello->p[i][j + 1][k];
                    neighborVelocity = jello->v[i][j + 1][k];
                    // calculate hook force
                    pSUM(fTotal, calculateHookLaw(jello->kElastic, jello->unrestLength, currentPoint, neighbor), fTotal);
                    // calculate damp force
                    pSUM(fTotal, calculateDamping(jello->dElastic, currentPoint, neighbor, currentPointVelocity, neighborVelocity), fTotal);
                }
                if (k > minIdx) // neighborIdx.push_back(indexStruct(i, j, k - 1));
                {
                    neighbor = jello->p[i][j][k - 1];
                    neighborVelocity = jello->v[i][j][k - 1];
                    // calculate hook force
                    pSUM(fTotal, calculateHookLaw(jello->kElastic, jello->unrestLength, currentPoint, neighbor), fTotal);
                    // calculate damp force
                    pSUM(fTotal, calculateDamping(jello->dElastic, currentPoint, neighbor, currentPointVelocity, neighborVelocity), fTotal);
                }
                if (k < maxIdx) // neighborIdx.push_back(indexStruct(i, j, k + 1));
                {
                    neighbor = jello->p[i][j][k + 1];
                    neighborVelocity = jello->v[i][j][k + 1];
                    // calculate hook force
                    pSUM(fTotal, calculateHookLaw(jello->kElastic, jello->unrestLength, currentPoint, neighbor), fTotal);
                    // calculate damp force
                    pSUM(fTotal, calculateDamping(jello->dElastic, currentPoint, neighbor, currentPointVelocity, neighborVelocity), fTotal);
                }

                // shear neighbors
                if (i < maxIdx) {
                    if (j < maxIdx) // neighborIdx.push_back(indexStruct(i + 1, j + 1, k));
                    {
                        neighbor = jello->p[i + 1][j + 1][k];
                        neighborVelocity = jello->v[i + 1][j + 1][k];

                        // calculate hook force
                        pSUM(fTotal, calculateHookLaw(jello->kElastic, jello->unrestLengthShear, currentPoint, neighbor), fTotal);
                        // calculate damp force
                        pSUM(fTotal, calculateDamping(jello->dElastic, currentPoint, neighbor, currentPointVelocity, neighborVelocity), fTotal);
                    }
                    if (j > minIdx) // neighborIdx.push_back(indexStruct(i + 1, j - 1, k));
                    {
                        neighbor = jello->p[i + 1][j - 1][k];
                        neighborVelocity = jello->v[i + 1][j - 1][k];

                        // calculate hook force
                        pSUM(fTotal, calculateHookLaw(jello->kElastic, jello->unrestLengthShear, currentPoint, neighbor), fTotal);
                        // calculate damp force
                        pSUM(fTotal, calculateDamping(jello->dElastic, currentPoint, neighbor, currentPointVelocity, neighborVelocity), fTotal);
                    }
                    if (k > minIdx) // neighborIdx.push_back(indexStruct(i + 1, j, k - 1));
                    {
                        neighbor = jello->p[i + 1][j][k - 1];
                        neighborVelocity = jello->v[i + 1][j][k - 1];

                        // calculate hook force
                        pSUM(fTotal, calculateHookLaw(jello->kElastic, jello->unrestLengthShear, currentPoint, neighbor), fTotal);
                        // calculate damp force
                        pSUM(fTotal, calculateDamping(jello->dElastic, currentPoint, neighbor, currentPointVelocity, neighborVelocity), fTotal);
                    }
                    if (k < maxIdx) // neighborIdx.push_back(indexStruct(i + 1, j, k + 1));
                    {
                        neighbor = jello->p[i + 1][j][k + 1];
                        neighborVelocity = jello->v[i + 1][j][k + 1];

                        // calculate hook force
                        pSUM(fTotal, calculateHookLaw(jello->kElastic, jello->unrestLengthShear, currentPoint, neighbor), fTotal);
                        // calculate damp force
                        pSUM(fTotal, calculateDamping(jello->dElastic, currentPoint, neighbor, currentPointVelocity, neighborVelocity), fTotal);
                    }
                }
                if (i > minIdx) {
                    if (j < maxIdx) // neighborIdx.push_back(indexStruct(i + 1, j + 1, k));
                    {
                        neighbor = jello->p[i - 1][j + 1][k];
                        neighborVelocity = jello->v[i - 1][j + 1][k];

                        // calculate hook force
                        pSUM(fTotal, calculateHookLaw(jello->kElastic, jello->unrestLengthShear, currentPoint, neighbor), fTotal);
                        // calculate damp force
                        pSUM(fTotal, calculateDamping(jello->dElastic, currentPoint, neighbor, currentPointVelocity, neighborVelocity), fTotal);
                    }
                    if (j > minIdx) // neighborIdx.push_back(indexStruct(i + 1, j - 1, k));
                    {
                        neighbor = jello->p[i - 1][j - 1][k];
                        neighborVelocity = jello->v[i - 1][j - 1][k];

                        // calculate hook force
                        pSUM(fTotal, calculateHookLaw(jello->kElastic, jello->unrestLengthShear, currentPoint, neighbor), fTotal);
                        // calculate damp force
                        pSUM(fTotal, calculateDamping(jello->dElastic, currentPoint, neighbor, currentPointVelocity, neighborVelocity), fTotal);
                    }
                    if (k > minIdx) // neighborIdx.push_back(indexStruct(i + 1, j, k - 1));
                    {
                        neighbor = jello->p[i - 1][j][k - 1];
                        neighborVelocity = jello->v[i - 1][j][k - 1];

                        // calculate hook force
                        pSUM(fTotal, calculateHookLaw(jello->kElastic, jello->unrestLengthShear, currentPoint, neighbor), fTotal);
                        // calculate damp force
                        pSUM(fTotal, calculateDamping(jello->dElastic, currentPoint, neighbor, currentPointVelocity, neighborVelocity), fTotal);
                    }
                    if (k < maxIdx) // neighborIdx.push_back(indexStruct(i + 1, j, k + 1));
                    {
                        neighbor = jello->p[i - 1][j][k + 1];
                        neighborVelocity = jello->v[i - 1][j][k + 1];

                        // calculate hook force
                        pSUM(fTotal, calculateHookLaw(jello->kElastic, jello->unrestLengthShear, currentPoint, neighbor), fTotal);
                        // calculate damp force
                        pSUM(fTotal, calculateDamping(jello->dElastic, currentPoint, neighbor, currentPointVelocity, neighborVelocity), fTotal);
                    }
                }
                if (j < maxIdx) {
                    if (k > minIdx) // neighborIdx.push_back(indexStruct(i + 1, j, k - 1));
                    {
                        neighbor = jello->p[i][j + 1][k - 1];
                        neighborVelocity = jello->v[i][j + 1][k - 1];

                        // calculate hook force
                        pSUM(fTotal, calculateHookLaw(jello->kElastic, jello->unrestLengthShear, currentPoint, neighbor), fTotal);
                        // calculate damp force
                        pSUM(fTotal, calculateDamping(jello->dElastic, currentPoint, neighbor, currentPointVelocity, neighborVelocity), fTotal);
                    }
                    if (k < maxIdx) // neighborIdx.push_back(indexStruct(i + 1, j, k + 1));
                    {
                        neighbor = jello->p[i][j + 1][k + 1];
                        neighborVelocity = jello->v[i][j + 1][k + 1];

                        // calculate hook force
                        pSUM(fTotal, calculateHookLaw(jello->kElastic, jello->unrestLengthShear, currentPoint, neighbor), fTotal);
                        // calculate damp force
                        pSUM(fTotal, calculateDamping(jello->dElastic, currentPoint, neighbor, currentPointVelocity, neighborVelocity), fTotal);
                    }
                }
                if (j > minIdx) {
                    if (k > minIdx) // neighborIdx.push_back(indexStruct(i + 1, j, k - 1));
                    {
                        neighbor = jello->p[i][j - 1][k - 1];
                        neighborVelocity = jello->v[i][j - 1][k - 1];

                        // calculate hook force
                        pSUM(fTotal, calculateHookLaw(jello->kElastic, jello->unrestLengthShear, currentPoint, neighbor), fTotal);
                        // calculate damp force
                        pSUM(fTotal, calculateDamping(jello->dElastic, currentPoint, neighbor, currentPointVelocity, neighborVelocity), fTotal);
                    }
                    if (k < maxIdx) // neighborIdx.push_back(indexStruct(i + 1, j, k + 1));
                    {
                        neighbor = jello->p[i][j - 1][k + 1];
                        neighborVelocity = jello->v[i][j - 1][k + 1];

                        // calculate hook force
                        pSUM(fTotal, calculateHookLaw(jello->kElastic, jello->unrestLengthShear, currentPoint, neighbor), fTotal);
                        // calculate damp force
                        pSUM(fTotal, calculateDamping(jello->dElastic, currentPoint, neighbor, currentPointVelocity, neighborVelocity), fTotal);
                    }
                }

                // shear diagonal
                if (i + 1 <= maxIdx) {
                    if (j + 1 <= maxIdx) {
                        if (k + 1 <= maxIdx) // neighborIdx.push_back(indexStruct(i + 1, j + 1, k + 1));
                        {
                            neighbor = jello->p[i + 1][j + 1][k + 1];
                            neighborVelocity = jello->v[i + 1][j + 1][k + 1];

                            // calculate hook force
                            pSUM(fTotal, calculateHookLaw(jello->kElastic, jello->unrestLengthShearDiag, currentPoint, neighbor), fTotal);
                            // calculate damp force
                            pSUM(fTotal, calculateDamping(jello->dElastic, currentPoint, neighbor, currentPointVelocity, neighborVelocity), fTotal);
                        }
                        if (k - 1 >= minIdx) // neighborIdx.push_back(indexStruct(i + 1, j + 1, k - 1));
                        {
                            neighbor = jello->p[i + 1][j + 1][k - 1];
                            neighborVelocity = jello->v[i + 1][j + 1][k - 1];

                            // calculate hook force
                            pSUM(fTotal, calculateHookLaw(jello->kElastic, jello->unrestLengthShearDiag, currentPoint, neighbor), fTotal);
                            // calculate damp force
                            pSUM(fTotal, calculateDamping(jello->dElastic, currentPoint, neighbor, currentPointVelocity, neighborVelocity), fTotal);
                        }
                    }

                    if (j - 1 >= minIdx) {
                        if (k + 1 <= maxIdx) // neighborIdx.push_back(indexStruct(i + 1, j - 1, k + 1));
                        {
                            neighbor = jello->p[i + 1][j - 1][k + 1];
                            neighborVelocity = jello->v[i + 1][j - 1][k + 1];

                            // calculate hook force
                            pSUM(fTotal, calculateHookLaw(jello->kElastic, jello->unrestLengthShearDiag, currentPoint, neighbor), fTotal);
                            // calculate damp force
                            pSUM(fTotal, calculateDamping(jello->dElastic, currentPoint, neighbor, currentPointVelocity, neighborVelocity), fTotal);
                        }
                        if (k - 1 >= minIdx) // neighborIdx.push_back(indexStruct(i + 1, j - 1, k - 1));
                        {
                            neighbor = jello->p[i + 1][j - 1][k - 1];
                            neighborVelocity = jello->v[i + 1][j - 1][k - 1];

                            // calculate hook force
                            pSUM(fTotal, calculateHookLaw(jello->kElastic, jello->unrestLengthShearDiag, currentPoint, neighbor), fTotal);
                            // calculate damp force
                            pSUM(fTotal, calculateDamping(jello->dElastic, currentPoint, neighbor, currentPointVelocity, neighborVelocity), fTotal);
                        }
                    }
                }
                if (i - 1 >= minIdx) {
                    if (j + 1 <= maxIdx) {
                        if (k + 1 <= maxIdx) // neighborIdx.push_back(indexStruct(i - 1, j + 1, k + 1));
                        {
                            neighbor = jello->p[i - 1][j + 1][k + 1];
                            neighborVelocity = jello->v[i - 1][j + 1][k + 1];

                            // calculate hook force
                            pSUM(fTotal, calculateHookLaw(jello->kElastic, jello->unrestLengthShearDiag, currentPoint, neighbor), fTotal);
                            // calculate damp force
                            pSUM(fTotal, calculateDamping(jello->dElastic, currentPoint, neighbor, currentPointVelocity, neighborVelocity), fTotal);
                        }
                        if (k - 1 >= minIdx) // neighborIdx.push_back(indexStruct(i - 1, j + 1, k - 1));
                        {
                            neighbor = jello->p[i - 1][j + 1][k - 1];
                            neighborVelocity = jello->v[i - 1][j + 1][k - 1];

                            // calculate hook force
                            pSUM(fTotal, calculateHookLaw(jello->kElastic, jello->unrestLengthShearDiag, currentPoint, neighbor), fTotal);
                            // calculate damp force
                            pSUM(fTotal, calculateDamping(jello->dElastic, currentPoint, neighbor, currentPointVelocity, neighborVelocity), fTotal);
                        }
                    }

                    if (j - 1 >= minIdx) {
                        if (k + 1 <= maxIdx) // neighborIdx.push_back(indexStruct(i - 1, j - 1, k + 1));
                        {
                            neighbor = jello->p[i - 1][j - 1][k + 1];
                            neighborVelocity = jello->v[i - 1][j - 1][k + 1];

                            // calculate hook force
                            pSUM(fTotal, calculateHookLaw(jello->kElastic, jello->unrestLengthShearDiag, currentPoint, neighbor), fTotal);
                            // calculate damp force
                            pSUM(fTotal, calculateDamping(jello->dElastic, currentPoint, neighbor, currentPointVelocity, neighborVelocity), fTotal);
                        }
                        if (k - 1 >= minIdx) // neighborIdx.push_back(indexStruct(i - 1, j - 1, k - 1));
                        {
                            neighbor = jello->p[i - 1][j - 1][k - 1];
                            neighborVelocity = jello->v[i - 1][j - 1][k - 1];

                            // calculate hook force
                            pSUM(fTotal, calculateHookLaw(jello->kElastic, jello->unrestLengthShearDiag, currentPoint, neighbor), fTotal);
                            // calculate damp force
                            pSUM(fTotal, calculateDamping(jello->dElastic, currentPoint, neighbor, currentPointVelocity, neighborVelocity), fTotal);
                        }
                    }
                }

                // find bending neighbors
                if (i > minIdx + 1) // neighborIdx.push_back(indexStruct(i - 2, j, k));
                {                    
                    neighbor = jello->p[i - 2][j][k];
                    neighborVelocity = jello->v[i - 2][j][k];

                    // calculate hook force
                    pSUM(fTotal, calculateHookLaw(jello->kElastic, jello->unrestLengthBend, currentPoint, neighbor), fTotal);
                    // calculate damp force
                    pSUM(fTotal, calculateDamping(jello->dElastic, currentPoint, neighbor, currentPointVelocity, neighborVelocity), fTotal);
                }                    
                if (i < maxIdx - 1) // neighborIdx.push_back(indexStruct(i + 2, j, k));
                {                    
                    neighbor = jello->p[i + 2][j][k];
                    neighborVelocity = jello->v[i + 2][j][k];

                    // calculate hook force
                    pSUM(fTotal, calculateHookLaw(jello->kElastic, jello->unrestLengthBend, currentPoint, neighbor), fTotal);
                    // calculate damp force
                    pSUM(fTotal, calculateDamping(jello->dElastic, currentPoint, neighbor, currentPointVelocity, neighborVelocity), fTotal);
                }                    
                if (j > minIdx + 1) // neighborIdx.push_back(indexStruct(i, j - 2, k));
                {                    
                    neighbor = jello->p[i][j - 2][k];
                    neighborVelocity = jello->v[i][j - 2][k];

                    // calculate hook force
                    pSUM(fTotal, calculateHookLaw(jello->kElastic, jello->unrestLengthBend, currentPoint, neighbor), fTotal);
                    // calculate damp force
                    pSUM(fTotal, calculateDamping(jello->dElastic, currentPoint, neighbor, currentPointVelocity, neighborVelocity), fTotal);
                }                    
                if (j < maxIdx - 1) // neighborIdx.push_back(indexStruct(i, j + 2, k));
                {                    
                    neighbor = jello->p[i][j + 2][k];
                    neighborVelocity = jello->v[i][j + 2][k];

                    // calculate hook force
                    pSUM(fTotal, calculateHookLaw(jello->kElastic, jello->unrestLengthBend, currentPoint, neighbor), fTotal);
                    // calculate damp force
                    pSUM(fTotal, calculateDamping(jello->dElastic, currentPoint, neighbor, currentPointVelocity, neighborVelocity), fTotal);
                }                    
                if (k > minIdx + 1) // neighborIdx.push_back(indexStruct(i, j, k - 2));
                {                    
                    neighbor = jello->p[i][j][k - 2];
                    neighborVelocity = jello->v[i][j][k - 2];

                    // calculate hook force
                    pSUM(fTotal, calculateHookLaw(jello->kElastic, jello->unrestLengthBend, currentPoint, neighbor), fTotal);
                    // calculate damp force
                    pSUM(fTotal, calculateDamping(jello->dElastic, currentPoint, neighbor, currentPointVelocity, neighborVelocity), fTotal);
                }                    
                if (k < maxIdx - 1) // neighborIdx.push_back(indexStruct(i, j, k + 2));
                {
                    neighbor = jello->p[i][j][k + 2];
                    neighborVelocity = jello->v[i][j][k + 2];

                    // calculate hook force
                    pSUM(fTotal, calculateHookLaw(jello->kElastic, jello->unrestLengthBend, currentPoint, neighbor), fTotal);
                    // calculate damp force
                    pSUM(fTotal, calculateDamping(jello->dElastic, currentPoint, neighbor, currentPointVelocity, neighborVelocity), fTotal);
                }

                //calculate collision springs forces
                for (int f = 0; f < jello->box.size(); f++) {
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
                        DOTPRODUCTp(invNormal, diff, penetration);
                        pMULTIPLY(face.normal, penetration, penetrationForce);
                        pDIFFERENCE(currentPoint, penetrationForce, collisionContact);

                        pSUM(calculateHookLaw(jello->kCollision, 0.0, collisionContact, currentPoint), fCollision, fCollision);
                        pSUM(calculateDamping(jello->dCollision, collisionContact, currentPoint, penetrationForce, currentPointVelocity), fCollision, fCollision);
                    }
                    pSUM(fCollision, fTotal, fTotal);
                }

                // calculate external force field
                pSUM(calculateExternalForce(jello, i, j, k), fTotal, fTotal);

                // mouse drag
                pSUM(jello->mouseForce, fTotal, fTotal);

                // a = F / m
                pMULTIPLY(fTotal, (1.0 / jello->mass), a[i][j][k]);
            }
        }
    }
}

/* performs one step of Euler Integration */
/* as a result, updates the jello structure */
void Euler(struct world* jello)
{
    int i, j, k;
    point a[8][8][8];

    computeAcceleration(jello, a);

    for (i = 0; i <= 7; i++)
        for (j = 0; j <= 7; j++)
            for (k = 0; k <= 7; k++)
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
void RK4(struct world* jello)
{
    point F1p[8][8][8], F1v[8][8][8],
        F2p[8][8][8], F2v[8][8][8],
        F3p[8][8][8], F3v[8][8][8],
        F4p[8][8][8], F4v[8][8][8];

    point a[8][8][8];


    struct world buffer;

    int i, j, k;

    buffer = *jello; // make a copy of jello

    computeAcceleration(jello, a);

    for (i = 0; i <= 7; i++)
        for (j = 0; j <= 7; j++)
            for (k = 0; k <= 7; k++)
            {
                pMULTIPLY(jello->v[i][j][k], jello->dt, F1p[i][j][k]);
                pMULTIPLY(a[i][j][k], jello->dt, F1v[i][j][k]);
                pMULTIPLY(F1p[i][j][k], 0.5, buffer.p[i][j][k]);
                pMULTIPLY(F1v[i][j][k], 0.5, buffer.v[i][j][k]);
                pSUM(jello->p[i][j][k], buffer.p[i][j][k], buffer.p[i][j][k]);
                pSUM(jello->v[i][j][k], buffer.v[i][j][k], buffer.v[i][j][k]);
            }

    computeAcceleration(&buffer, a);

    for (i = 0; i <= 7; i++)
        for (j = 0; j <= 7; j++)
            for (k = 0; k <= 7; k++)
            {
                // F2p = dt * buffer.v;
                pMULTIPLY(buffer.v[i][j][k], jello->dt, F2p[i][j][k]);
                // F2v = dt * a(buffer.p,buffer.v);     
                pMULTIPLY(a[i][j][k], jello->dt, F2v[i][j][k]);
                pMULTIPLY(F2p[i][j][k], 0.5, buffer.p[i][j][k]);
                pMULTIPLY(F2v[i][j][k], 0.5, buffer.v[i][j][k]);
                pSUM(jello->p[i][j][k], buffer.p[i][j][k], buffer.p[i][j][k]);
                pSUM(jello->v[i][j][k], buffer.v[i][j][k], buffer.v[i][j][k]);
            }

    computeAcceleration(&buffer, a);

    for (i = 0; i <= 7; i++)
        for (j = 0; j <= 7; j++)
            for (k = 0; k <= 7; k++)
            {
                // F3p = dt * buffer.v;
                pMULTIPLY(buffer.v[i][j][k], jello->dt, F3p[i][j][k]);
                // F3v = dt * a(buffer.p,buffer.v);     
                pMULTIPLY(a[i][j][k], jello->dt, F3v[i][j][k]);
                pMULTIPLY(F3p[i][j][k], 1.0, buffer.p[i][j][k]);
                pMULTIPLY(F3v[i][j][k], 1.0, buffer.v[i][j][k]);
                pSUM(jello->p[i][j][k], buffer.p[i][j][k], buffer.p[i][j][k]);
                pSUM(jello->v[i][j][k], buffer.v[i][j][k], buffer.v[i][j][k]);
            }

    computeAcceleration(&buffer, a);


    for (i = 0; i <= 7; i++)
        for (j = 0; j <= 7; j++)
            for (k = 0; k <= 7; k++)
            {
                // F3p = dt * buffer.v;
                pMULTIPLY(buffer.v[i][j][k], jello->dt, F4p[i][j][k]);
                // F3v = dt * a(buffer.p,buffer.v);     
                pMULTIPLY(a[i][j][k], jello->dt, F4v[i][j][k]);

                pMULTIPLY(F2p[i][j][k], 2, buffer.p[i][j][k]);
                pMULTIPLY(F3p[i][j][k], 2, buffer.v[i][j][k]);
                pSUM(buffer.p[i][j][k], buffer.v[i][j][k], buffer.p[i][j][k]);
                pSUM(buffer.p[i][j][k], F1p[i][j][k], buffer.p[i][j][k]);
                pSUM(buffer.p[i][j][k], F4p[i][j][k], buffer.p[i][j][k]);
                pMULTIPLY(buffer.p[i][j][k], 1.0 / 6, buffer.p[i][j][k]);
                pSUM(buffer.p[i][j][k], jello->p[i][j][k], jello->p[i][j][k]);

                pMULTIPLY(F2v[i][j][k], 2, buffer.p[i][j][k]);
                pMULTIPLY(F3v[i][j][k], 2, buffer.v[i][j][k]);
                pSUM(buffer.p[i][j][k], buffer.v[i][j][k], buffer.p[i][j][k]);
                pSUM(buffer.p[i][j][k], F1v[i][j][k], buffer.p[i][j][k]);
                pSUM(buffer.p[i][j][k], F4v[i][j][k], buffer.p[i][j][k]);
                pMULTIPLY(buffer.p[i][j][k], 1.0 / 6, buffer.p[i][j][k]);
                pSUM(buffer.p[i][j][k], jello->v[i][j][k], jello->v[i][j][k]);
            }

    return;
}
