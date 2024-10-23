#pragma once
 
#include <vector>
#include "minivector.h"
#include "sparseMatrix.h"
#include "baseModel2D.h"
 
class MassSpring2D : public BaseModel2D
{
  public:
 
    MassSpring2D(TriangleMesh2D * mesh);
    ~MassSpring2D();
 
    virtual void ComputeElementEnergyAndForceAndStiffnessMatrix(int elementID, const double * vertexDisplacements, double * elementEnergy = nullptr, double * elementInternalForces = nullptr, double * elementStiffnessMatrix = nullptr) override;
    const int * getVertexIndices(int stencilID) override;
    virtual int getNumElements() {return numEdges;}
    virtual int getNumElementVertices() { return 2; }
  private:
 
    std::vector<double> restLengths;

    void GenerateMassSpringSystem(int numVertices, const double * masses, const std::vector<double> &restPositions, int numEdges, const std::vector<int> &edges); // constructor helper function
    void postProcess(double * elementInternalForces = nullptr, double * elementStiffnessMatrix = nullptr);
};