#pragma once

#include <vector>
#include "minivector.h"
#include "sparseMatrix.h"
#include "triangleMesh2D.h"

/* 
  CSCI 520 Computer Animation and Simulation
  Programming Assignment:
  2D Simulator for Mass spring, Linear FEM and Corotational FEM

  Mianlun Zheng and Jernej Barbic
  University of Southern California
*/

class BaseModel2D
{
    public:
     
    BaseModel2D(TriangleMesh2D * mesh_);
    virtual ~BaseModel2D(); 
 
    inline int getNumVertices() { return numVertices; }
    inline int getNumEdges() { return numEdges; }
    inline int * getEdges() { return edges.data(); }

    void GenerateMassMatrix(SparseMatrix **M);


    inline TriangleMesh2D * GetTriangleMesh2D() { return mesh;}

    virtual void ComputeElementEnergyAndForceAndStiffnessMatrix(int elementID, const double * vertexDisplacements, double * elementEnergy = nullptr, double * elementInternalForces = nullptr, double * elementStiffnessMatrix = nullptr) = 0;
    virtual const int* getVertexIndices(int stencilID) = 0;
    virtual int getNumElements() = 0;
    virtual int getNumElementVertices() = 0;
 
  protected:

    TriangleMesh2D *mesh;

    int numVertices;
    int numEdges;
    vector<double> undeformedPositions;
    vector<int> edges;
    vector<double> masses;
    
    double GetTriangleSurfaceArea(Vec2d p0, Vec2d p1, Vec2d p2);
};
