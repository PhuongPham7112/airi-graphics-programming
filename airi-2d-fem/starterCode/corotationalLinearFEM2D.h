#pragma once

#include "triangleMesh2D.h"
#include "sparseMatrix.h"
#include "linearFEM2D.h"

/* 
  CSCI 520 Computer Animation and Simulation
  Programming Assignment:
  2D Simulator for Mass spring, Linear FEM and Corotational FEM

  Mianlun Zheng and Jernej Barbic
  University of Southern California
*/

class CorotationalLinearFEM2D : public LinearFEM2D
{
public:
    CorotationalLinearFEM2D(TriangleMesh2D * mesh_);
    virtual ~CorotationalLinearFEM2D();

    virtual void ComputeElementEnergyAndForceAndStiffnessMatrix(int elementID, const double * vertexDisplacements, double * elementEnergy = nullptr, 
        double * elementInternalForces = nullptr, double * elementStiffnessMatrix = nullptr);
    
private:
  
    void WarpMatrix(double * K, double * R, double * RK, double * RKRT);
    void polarDecomposition2D(const double * F, double * R, double * S);
    void inverse2x2(const double * A, double * Ainv);
};

