#include "massSpring2D.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <vector>
#include <set>
#include "macros.h"
using namespace std;
 
MassSpring2D::MassSpring2D(TriangleMesh2D * mesh) : BaseModel2D(mesh)
{  
  double stiffness = mesh->getMassSpringStiffness();
  GenerateMassSpringSystem(numVertices, masses.data(), undeformedPositions, numEdges, edges);
}
 
MassSpring2D::~MassSpring2D()
{
}
 
void MassSpring2D::GenerateMassSpringSystem(int numVertices, const double * masses, const std::vector<double> &restPositions, int numEdges, const std::vector<int> & edges) 
{
  // compute rest lengths of springs
  for(int i=0; i<numEdges; i++)
  {
    int particleA = edges[2*i+0];
    int particleB = edges[2*i+1];
 
    double restDisp[2];
    restDisp[0] = restPositions[particleB*2+0] - restPositions[particleA*2+0]; 
    restDisp[1] = restPositions[particleB*2+1] - restPositions[particleA*2+1];
 
    restLengths.push_back(sqrt(restDisp[0]*restDisp[0] + restDisp[1]*restDisp[1]));
  }
}

void MassSpring2D::ComputeElementEnergyAndForceAndStiffnessMatrix(int eid, const double * u, double * elementEnergy, 
    double * elementInternalForces, double * elementStiffnessMatrix)
{
  double stiffness = mesh->getMassSpringStiffness(); // stiffness of the spring
  int particleA = edges[2*eid+0]; // index of vertex A on the spring
  int particleB = edges[2*eid+1]; // index of vertex B on the spring
  // undeformedPositions, size 2*vertexNumber, stores the rest positions of all verteices
  // u, size 2*vertexNumber, stores the displacements of all verteices

  // ********** Students should implement this **********
  
  if (elementEnergy)
  {
    // please calculate the elastic energy on the spring here
    
  }

  if (elementInternalForces)
  {
    // please calculate the the spring forces here
   
  }

  if (elementStiffnessMatrix) 
  {
    // please calculate the stiffness matrix for the spring here
    
  }

  // ***********************************************

  postProcess(elementInternalForces, elementStiffnessMatrix);
}

const int * MassSpring2D::getVertexIndices(int stencilID)
{
  return edges.data() + stencilID*2;
}

void MassSpring2D::postProcess(double * elementInternalForces, double * elementStiffnessMatrix)
{
  if(elementInternalForces)
  {
    for(int i=0; i<4; i++)
      elementInternalForces[i] = -elementInternalForces[i];
  }
  if(elementStiffnessMatrix)
  {
    for(int i=0; i<16; i++)
      elementStiffnessMatrix[i] = -elementStiffnessMatrix[i];
  }
}
