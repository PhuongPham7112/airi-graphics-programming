#include "linearFEM2D.h"
#include "mat3d.h"

/* 
  CSCI 520 Computer Animation and Simulation
  Programming Assignment:
  2D Simulator for Mass spring, Linear FEM and Corotational FEM

  Mianlun Zheng and Jernej Barbic
  University of Southern California
*/

LinearFEM2D::LinearFEM2D(TriangleMesh2D * mesh_): BaseModel2D(mesh_)
{
  numVertices = mesh->getNumVertices();
  int numElements = mesh_->getNumElements();

  PInverse.resize(numElements);
  for(int el = 0; el < numElements; el++)
  {
    // get the integer indices of the triangle vertices
    int vtxIndex[3];
    for(int vtx=0; vtx<3; vtx++)
      vtxIndex[vtx] = mesh->getVertexIndex(el, vtx);
    /*
       Form matrix:
       P = [ v0   v1   v2  ]
           [  1    1    1  ]
    */
    double P[9]; // row-major
    for(int vtx=0; vtx<3; vtx++)
      for(int dim=0; dim<2; dim++)
        P[3 * dim + vtx] = undeformedPositions[2 * vtxIndex[vtx] + dim];
    P[6] = P[7] = P[8] = 1.0;

    // invert P and cache inverse (see [Mueller 2004])
    PInverse[el].resize(9);
    inverse3x3(P, PInverse[el].data());
  }

  // compute stiffness matrices for all the elements in the undeformed configuration
  KElementUndeformed.resize(numElements);

  for (int el = 0; el < numElements; el++)
  {
      double * PInv = &PInverse[el][0];

      // Form stiffness matrix of the element in the undeformed configuration.
      // The procedure below is standard in FEM solid mechanics.
      // This code implements the equations given in Ahmed A. Shabana: Theory of Vibration, Volume II: Discrete and Continuous Systems, Springer--Verlag, New York, NY, 1990.
      // compute elasticity stiffness tensor
      double E[9]; // 3 x 3 matrix, stored row-major (symmetric, so row vs column-major storage makes no difference anyway)
      // theta = E * epsilon
      // epsilon = {e11, e22, 2*e12}
      // theta = {t11, t22, t12}
      if(computeElasticityStiffnessTensor(E, el) == false) // failure in constructing E
      {
        clear();  //clean data allocated so far
        throw 2;
      }

      double B[18] =
        { PInv[0], 0,         PInv[3], 0,         PInv[6], 0,  
          0, PInv[1],         0, PInv[4],         0, PInv[7],
          PInv[1], PInv[0],   PInv[4], PInv[3],   PInv[7], PInv[6]};

      // EB = E * B
      double EB[18];
      memset(EB, 0, sizeof(double) * 18);
      for (int i=0; i<3; i++)
        for (int j=0; j<6; j++)
          for (int k=0; k<3; k++)
            EB[6 * i + j] += E[3 * i + k] * B[6 * k + j];

      // // KElementUndeformed[el] = B^T * EB
      KElementUndeformed[el].resize(36); // element stiffness matrix
      for (int i=0; i<6; i++)
        for (int j=0; j<6; j++)
          for (int k=0; k<3; k++)
            KElementUndeformed[el][6 * i + j] += B[6 * k + i] * EB[6 * k + j];

      // KElementUndeformed[el] *= volume
      double area = mesh->getElementArea(el);

      for(int i=0; i<36; i++)
        KElementUndeformed[el][i] *= area;
  }

  cout << "Done." << endl; 
}


LinearFEM2D::~LinearFEM2D()
{
  clear();
}

void LinearFEM2D::ComputeElementEnergyAndForceAndStiffnessMatrix(int el, const double * u, double * elementEnergy, 
    double * elementInternalForces, double * elementStiffnessMatrix)
{
  int numElementVertices = 3; // each element (triangle) has three vertices
  int dof = 6; // in total 3*2 dimentions for one triangle
  int dof2 = dof * dof;
  const int *vtxIndex = mesh->getVertexIndices(el); // vertex index for the triangle el
  // undeformedPositions, size 2*vertexNumber, stores the rest positions of all verteices
  // input u, size 2*vertexNumber, stores the displacements of all verteices

  // ********** Students should implement this **********

  // the constant stiffnessMatrix for element el is stored in KElementUndeformed[el]
  if (elementStiffnessMatrix)
  {
    // please copy the stiffness matrix from  KElementUndeformed[el] to elementStiffnessMatrix
  }
  
  if(elementInternalForces)
  {
    // please calculate the internal force for the element (triangle)
  }

  if (elementEnergy)
  {
    // please calculate the energy for the element (triangle)
  }

  // ***********************************************************
}


// compute elasticity stiffness tensor
// E: 6 x 6 matrix, stored row-major (symmetric, so row vs column-major storage makes no difference anyway)
bool LinearFEM2D::computeElasticityStiffnessTensor(double E[9], int el)
{
  ///////////  only isotropic homogeneous material  implemented here  ///////////////////
  // material is isotropic, specified by E, nu

   // compute Lame coefficients
  double lambda = mesh->getLambda();
  double mu = mesh->getMu();

    double Et[9] = { lambda + 2 * mu, lambda, 0,
                      lambda, lambda + 2 * mu, 0,
                      0,      0,               mu };

    memcpy(E, Et, sizeof(double) * 9);

  return true;
}

void LinearFEM2D::clear()
{
}


const int * LinearFEM2D::getVertexIndices(int stencilID)
{
  return mesh->getVertexIndices(stencilID);
}
