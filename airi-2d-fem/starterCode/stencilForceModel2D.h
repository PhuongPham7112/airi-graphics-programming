/* 
  CSCI 520 Computer Animation and Simulation
  Programming Assignment:
  2D Simulator for Mass spring, Linear FEM and Corotational FEM

  Mianlun Zheng and Jernej Barbic
  University of Southern California
*/

#pragma once
#include "stencilForceModel.h"
#include "baseModel2D.h"

class StencilForceModel2D : public StencilForceModel
{
public:
    StencilForceModel2D(BaseModel2D * baseModel2D_);
    ~StencilForceModel2D();

    virtual const int *GetStencilVertexIndices(int stencilType, int stencilId) const override;
    virtual void GetStencilLocalEnergyAndForceAndMatrix(int stencilType, int stencilId, const double * u, double * energy, double * internalForces, double * tangentStiffnessMatrix) override;

    BaseModel2D * GetForceModelHandle() { return baseModel2D; }

    // Vertex gravity.
    virtual void GetVertexGravityForce(int vertexId, double gravity[2]) { gravity[0] = gravity[1] = 0.0; }

private:
    BaseModel2D * baseModel2D;
};
