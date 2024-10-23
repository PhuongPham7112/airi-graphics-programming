#pragma once

#include "triangleMesh2D.h"
#include "sparseMatrix.h"
#include "baseModel2D.h"

/* 
  CSCI 520 Computer Animation and Simulation
  Programming Assignment:
  2D Simulator for Mass spring, Linear FEM and Corotational FEM

  Mianlun Zheng and Jernej Barbic
  University of Southern California
*/

class LinearFEM2D : public BaseModel2D
{
public:
    LinearFEM2D(TriangleMesh2D * mesh_);
    virtual ~LinearFEM2D();


    virtual void ComputeElementEnergyAndForceAndStiffnessMatrix(int elementID, const double * vertexDisplacements, double * elementEnergy = nullptr, 
        double * elementInternalForces = nullptr, double * elementStiffnessMatrix = nullptr);
    virtual const int * getVertexIndices(int stencilID) override;
    virtual int getNumElements() { return mesh->getNumTriangles(); }
    virtual int getNumElementVertices() { return mesh->getNumElementVertices(); }

protected:

    vector<vector<double>> PInverse;
    vector<vector<double>> KElementUndeformed;
    
    bool computeElasticityStiffnessTensor(double E[9], int el);
    // build inverse of M = [ v0   v1   v2   v3 ]
    //                      [  1    1    1    1 ]
    // volumetricMesh must be TET of CUBIC. If it's CUBIC, M consists of vertices forming a tet in the center of the cube.
    // static void buildMInverse(double * MInverse, VolumetricMesh * volumetricMesh);

    void clear();
};

