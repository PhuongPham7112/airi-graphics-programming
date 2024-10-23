#include "baseModel2D.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <vector>
#include <set>
using namespace std;

/* 
  CSCI 520 Computer Animation and Simulation
  Programming Assignment:
  2D Simulator for Mass spring, Linear FEM and Corotational FEM

  Mianlun Zheng and Jernej Barbic
  University of Southern California
*/

BaseModel2D::BaseModel2D(TriangleMesh2D * mesh_) : mesh(mesh_)
{
  numVertices = mesh_->getNumVertices();
  // store the undeformed positions
  undeformedPositions.resize(2 * numVertices, 0.0);
  for(int i=0; i < numVertices; i++)
  {
    const Vec2d & v = mesh->getRestPosition(i);
    for(int j=0; j<2; j++)
      undeformedPositions[2*i+j] = v[j];
  }

  // ********** Students should implement this **********

  // compute mass for each vertex
  masses.resize(numVertices, 0.0); 
  double a_third = 1.0 / 3.0;
  double density_ = mesh_->getDensity();
  std::vector<Vec3i> triangles = mesh_->getTriangles(); // triangles store the vertex indices for each triangle

  // please calculate the mass for each vertex and store it in the vector masses above
  for (int i = 0; i < triangles.size(); i++)
  {
      // triangle i_th
      Vec3i t = triangles[i];
      Vec2d p_a = mesh_->getPosition(t[0]); // vertex point a
      Vec2d p_b = mesh_->getPosition(t[1]); // vertex point b
      Vec2d p_c = mesh_->getPosition(t[2]); // vertex point c
      
      Vec2d s_ab = p_a - p_b; // side ab
      Vec2d s_bc = p_c - p_b; // side bc
      double dot_product = dot(s_ab, s_bc);

      Vec2d vec_base = (dot_product / length(s_bc)) * norm(s_bc);
      Vec2d vec_height = vec_base - s_ab;
      
      double t_height = length(vec_height);
      double t_base = length(p_b, p_c);
      double v_e = 0.5 * t_base * t_height; // surface area

      double m_e = v_e * density_;
      masses[t[0]] += m_e * a_third;
      masses[t[1]] += m_e * a_third;
      masses[t[2]] += m_e * a_third;
  }
  
  // *********************************************

  // create set for edges
  typedef pair<int,int> edge;
  #define SORTED(i,j) ( (i) <= (j) ? make_pair((i),(j)) : make_pair((j),(i)) )
  set<edge> edge_;
  for(int i=0; i<mesh_->getNumTriangles(); i++)
  {
    int v0 = triangles[i][0];
    int v1 = triangles[i][1];
    int v2 = triangles[i][2];
    edge_.insert(SORTED(v0,v1));
    edge_.insert(SORTED(v0,v2));
 
    edge_.insert(SORTED(v1,v0));
    edge_.insert(SORTED(v1,v2));
 
    edge_.insert(SORTED(v2,v0));
    edge_.insert(SORTED(v2,v1));
  }
 
  numEdges = edge_.size(); 
  edges.resize(2 * numEdges, 0);
  
  int count = 0;
  for(set<edge> :: iterator iter = edge_.begin(); iter != edge_.end(); iter++)
  {
    edges[2*count+0] = iter->first;
    edges[2*count+1] = iter->second;
    count++;
  } 
}

BaseModel2D::~BaseModel2D()
{
}

double BaseModel2D::GetTriangleSurfaceArea(Vec2d p0, Vec2d p1, Vec2d p2)
{
  double s = 0.0;
  return s;
}

void BaseModel2D::GenerateMassMatrix(SparseMatrix ** M)
{
    SparseMatrixOutline outline(2 * numVertices); 
    for(int i=0; i<numVertices; i++)
        for(int j=0; j<2; j++)
            outline.AddEntry(2*i+j, 2*i+j, masses[i]); 
    *M = new SparseMatrix(&outline);
}

