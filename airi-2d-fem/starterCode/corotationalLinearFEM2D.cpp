#include "corotationalLinearFEM2D.h"
#include "mat3d.h"

/* 
  CSCI 520 Computer Animation and Simulation
  Programming Assignment:
  2D Simulator for Mass spring, Linear FEM and Corotational FEM

  Mianlun Zheng and Jernej Barbic
  University of Southern California
*/

CorotationalLinearFEM2D::CorotationalLinearFEM2D(TriangleMesh2D * mesh_): LinearFEM2D(mesh_)
{
}

CorotationalLinearFEM2D::~CorotationalLinearFEM2D()
{
}

void CorotationalLinearFEM2D::polarDecomposition2D(const double * F, double * R, double * S)
{
  Vec2d temp(F[0]+F[3], F[2]-F[1]);
  Vec2d temp2 = norm(temp);

  double R_[4] = {temp2[0], -temp2[1], temp2[1], temp2[0]};
  memcpy(R, R_, sizeof(double) * 4);

  double S_[4] = {R_[0]*F[0]+R_[2]*F[2], R_[0]*F[1]+R_[2]*F[3], R_[1]*F[0]+R_[3]*F[2], R_[1]*F[1]+R_[3]*F[3]};
  memcpy(S, S_, sizeof(double) * 4);
}

void CorotationalLinearFEM2D::inverse2x2(const double * A, double * AInv)
{ 
}

void CorotationalLinearFEM2D::ComputeElementEnergyAndForceAndStiffnessMatrix(int el, const double * u, double * elementEnergy, 
    double * elementInternalForces, double * elementStiffnessMatrix)
{
  const int numElementVertices = 3; // each triangle has 3 vertices
  const int numElementDOFs = numElementVertices * 2; // in total 3 *2 dimensions for one triangle
  const int * vtxIndex = mesh->getVertexIndices(el);// vertex index for the triangle el
  // undeformedPositions, size 2*vertexNumber, stores the rest positions of all verteices
  // u, size 2*vertexNumber, stores the displacements of all verteices

  // ********** Students should implement this **********

  // please calculate the deformation gradient matrix F here
  // F = Ds * DmInv
  double Fe[4] = {1.0, 0.0, 0.0, 1.0};
  double Ds[4] = {};
  double Dm[4] = {};
  double DmInv[4] = {};





  // polar decomposition of F is provided here. no need to implement it
  double Re[4]; // rotation (row-major)
  double Se[4]; // symmetric (row-major)
  polarDecomposition2D(Fe, Re, Se);
  // now the rotation Re is computed and stored in Re[4] array

  if(elementStiffnessMatrix)
  {
    // please calculate the warped stiffness matrix K' here
    // the unwarped stiffness matrix for element el is stored in KElementUndeformed[el]

  }

  if(elementEnergy)
  {
    // please calculate the elastic energy here

  }

  if(elementInternalForces)
  {
    // please calculate the internal force here

  }

  // ******************************************
}

// compute RK = R * K and RKRT = R * K * R^T (block-wise)
// input: K, R
// output: RK, RKRT
void CorotationalLinearFEM2D::WarpMatrix(double * K, double * R, double * RK, double * RKRT)
{
}

