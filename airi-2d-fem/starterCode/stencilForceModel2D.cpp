/* 
  CSCI 520 Computer Animation and Simulation
  Programming Assignment:
  2D Simulator for Mass spring, Linear FEM and Corotational FEM

  Mianlun Zheng and Jernej Barbic
  University of Southern California
*/

#include "stencilForceModel2D.h"
#include <cassert>

 StencilForceModel2D::StencilForceModel2D(BaseModel2D * baseModel2D_) : baseModel2D(baseModel2D_)
 {
    n = baseModel2D->GetTriangleMesh2D()->getNumVertices();
    r = n * 2;
    numStencilsInDifferentTypes.push_back(baseModel2D->getNumElements());
    numStencilVerticesInDifferentTypes.push_back(baseModel2D->getNumElementVertices());
 }

StencilForceModel2D::~StencilForceModel2D()
{
}

const int * StencilForceModel2D::GetStencilVertexIndices(int stencilType, int stencilId) const
{
    assert(stencilType == 0);
    return baseModel2D->getVertexIndices(stencilId);
}

void StencilForceModel2D::GetStencilLocalEnergyAndForceAndMatrix(int stencilType, int stencilId, const double * u, double * energy, double * internalForces, double * tangentStiffnessMatrix)
{
   baseModel2D->ComputeElementEnergyAndForceAndStiffnessMatrix(stencilId, u, energy, internalForces, tangentStiffnessMatrix);
}
