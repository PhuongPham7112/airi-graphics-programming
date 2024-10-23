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
  // undeformedPositions, size 2*vertexNumber, stores the rest positions of all vertices
  // u, size 2*vertexNumber, stores the displacements of all vertices

  // ********** Students should implement this **********

  // point a: rest position, displacement, deformed position
  Vec2d x_a = Vec2d(undeformedPositions[particleA * 2], undeformedPositions[particleA * 2 + 1]);
  Vec2d u_a = Vec2d(u[particleA * 2], u[particleA * 2 + 1]);
  Vec2d p_a = x_a + u_a;

  // point b: rest position, displacement, deformed position
  Vec2d x_b = Vec2d(undeformedPositions[particleB * 2], undeformedPositions[particleB * 2 + 1]);
  Vec2d u_b = Vec2d(u[particleB * 2], u[particleB * 2 + 1]);
  Vec2d p_b = x_b + u_b;
  
  // vector L
  Vec2d L = p_a - p_b;
  double l_length = length(L);
  double inv_l_length = 1 / l_length;
  double inv_l_length_3 = pow(inv_l_length, 3.0);
  double r_length = restLengths[eid];
  Vec2d f_a = -stiffness * (l_length - r_length) * norm(L);
  Vec2d f_b = -f_a;
  
  // k_hook * (1 - R * (1 / L - 1 / L^3 * (p_a.x - p_b.x) * (p_a.x - p_b.x)))
  double x = -stiffness * (1.0 - r_length * (inv_l_length - inv_l_length_3 * (p_a[0] - p_b[0]) * (p_a[0] - p_b[0])));
  // k_hook * (- R * (- 1 / L^3 * (p_a.x - p_b.x) * (p_a.y - p_b.y)))
  double y = -stiffness * (-r_length * (-inv_l_length_3 * (p_a[0] - p_b[0]) * (p_a[1] - p_b[1])));
  // k_hook * (1 - R * (1 / L - 1 / L^3 * (p_a.y - p_b.y) * (p_a.y - p_b.y)))
  double z = -stiffness * (1.0 - r_length * (inv_l_length - inv_l_length_3 * (p_a[1] - p_b[1]) * (p_a[1] - p_b[1])));

  if (elementEnergy)
  {
    // please calculate the elastic energy on the spring here
      *elementEnergy = 0.5 * stiffness * (l_length - r_length) * (l_length - r_length);
  }
  if (elementInternalForces) // 2x2 matrix
  {
    // please calculate the the spring forces here
      elementInternalForces[0] = f_a[0];
      elementInternalForces[1] = f_a[1];
      elementInternalForces[2] = f_b[0];
      elementInternalForces[3] = f_b[1];
  }

  if (elementStiffnessMatrix)  // 4x4 matrix
  {
    // please calculate the stiffness matrix for the spring here
      elementStiffnessMatrix[0] = x;
      elementStiffnessMatrix[1] = y;
      elementStiffnessMatrix[4] = y;
      elementStiffnessMatrix[5] = z;
      
      elementStiffnessMatrix[2] = -x;
      elementStiffnessMatrix[3] = -y;
      elementStiffnessMatrix[6] = -y;
      elementStiffnessMatrix[7] = -z;

      elementStiffnessMatrix[8] = -x;
      elementStiffnessMatrix[9] = -y;
      elementStiffnessMatrix[12] = -y;
      elementStiffnessMatrix[13] = -z;

      elementStiffnessMatrix[10] = x;
      elementStiffnessMatrix[11] = y;
      elementStiffnessMatrix[14] = y;
      elementStiffnessMatrix[15] = z;
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
