/* 
  CSCI 520 Computer Animation and Simulation
  Programming Assignment:
  2D Simulator for Mass spring, Linear FEM and Corotational FEM

  Mianlun Zheng and Jernej Barbic
  University of Southern California
*/

#include "triangleMeshRendering2D.h"
#include "openGLHelper.h"
#include <iostream>
#include <cfloat>

TriangleMeshRendering2D::TriangleMeshRendering2D(TriangleMesh2D * mesh): pointSize(1.0), pointColor(1.0, 0.0, 0.0), wireSize(1.0), wireColor(0.0), meshColor(0.3)
{
    numVertices = mesh->getNumVertices();
    vertexPosition = mesh->getRestPosition();
    restVertexPosition = mesh->getRestPosition();
    numTriangles = mesh->getNumTriangles();
    triangles = mesh->getTriangles();
    groups = mesh->getGroup();
    materials = mesh->getMaterial();
}

TriangleMeshRendering2D::~TriangleMeshRendering2D()
{
}

void TriangleMeshRendering2D::drawMeshWire()
{
    glLineWidth(wireSize);
    glColor3f(wireColor[0], wireColor[1], wireColor[2]);
    //glBegin(GL_LINE_STRIP);
    glBegin(GL_LINES);
    for(int i=0; i<numTriangles; i++)
    {
        Vec3i index = {triangles[i][0], triangles[i][1], triangles[i][2]};
        glVertex2f(vertexPosition[index[0]][0], vertexPosition[index[0]][1]);
        glVertex2f(vertexPosition[index[1]][0], vertexPosition[index[1]][1]);

        glVertex2f(vertexPosition[index[0]][0], vertexPosition[index[0]][1]);
        glVertex2f(vertexPosition[index[2]][0], vertexPosition[index[2]][1]);

        glVertex2f(vertexPosition[index[1]][0], vertexPosition[index[1]][1]);
        glVertex2f(vertexPosition[index[2]][0], vertexPosition[index[2]][1]);
    }
    glEnd();
}

void TriangleMeshRendering2D::drawMeshPoint()
{
    glPointSize(pointSize);
    glColor3f(pointColor[0], pointColor[1], pointColor[2]);
    glBegin(GL_POINTS);
    for(int i=0; i<numVertices; i++)
    {
      glVertex2f(vertexPosition[i][0], vertexPosition[i][1]);
    }
    glEnd();
}

void TriangleMeshRendering2D::drawMeshSurface()
{
    glColor3f(meshColor[0], meshColor[1], meshColor[2]);
    glEnable(GL_COLOR_MATERIAL);
    glBegin(GL_TRIANGLES);
    for(int i=0; i<numTriangles; i++)
    {
        Vec3i index = {triangles[i][0], triangles[i][1], triangles[i][2]};
        glVertex2f(vertexPosition[index[0]][0], vertexPosition[index[0]][1]);
        glVertex2f(vertexPosition[index[1]][0], vertexPosition[index[1]][1]);
        glVertex2f(vertexPosition[index[2]][0], vertexPosition[index[2]][1]);
    }
    glEnd();
}

void TriangleMeshRendering2D::drawMaterials()
{
  glPushAttrib(GL_ENABLE_BIT|GL_POLYGON_BIT|GL_LIGHTING_BIT|GL_TEXTURE_BIT);
  //glDisable(GL_COLOR_MATERIAL);
  glEnable(GL_COLOR_MATERIAL);
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  glDisable(GL_TEXTURE_2D);

  TriangleMesh2D::Material defaultMaterial;

  for(int i=0; i < groups.size(); i++)
  {
    TriangleMesh2D::Group group = groups[i];
    TriangleMesh2D::Material material; 
    // set material
    if(group.getMaterialIndex()>=materials.size())
      material = defaultMaterial;
    else
      material = materials[group.getMaterialIndex()];
    //printf("Material: %d\n",group.materialIndex());
    
    Vec3d Ka = material.getKa();
    Vec3d Kd = material.getKd();
    Vec3d Ks = material.getKs();

    float shininess = material.getShininess();
    float alpha = material.getAlpha();
    float ambient[4] = { (float)Ka[0], (float)Ka[1], (float)Ka[2], alpha };
    float diffuse[4] = { (float)Kd[0], (float)Kd[1], (float)Kd[2], alpha };
    float specular[4] = { (float)Ks[0], (float)Ks[1], (float)Ks[2], alpha };
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ambient);
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diffuse);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shininess);
    
    glColor3fv(diffuse); // should use material, but.... maybe implement it later... sad.....
    for(int iFace = 0; iFace < group.getNumFaces(); iFace++)
    {
      const Vec3i face = group.getFace(iFace);
      glBegin(GL_TRIANGLES);
        glVertex2f(vertexPosition[face[0]][0], vertexPosition[face[0]][1]);
        glVertex2f(vertexPosition[face[1]][0], vertexPosition[face[1]][1]);
        glVertex2f(vertexPosition[face[2]][0], vertexPosition[face[2]][1]);
      glEnd();
    }
  }
  glPopAttrib();
}

void TriangleMeshRendering2D::setVertexDeformations(const double *u)
{
  for(int i=0; i<numVertices; i++)
    setPosition(i, Vec2d(restVertexPosition[i][0]+u[i*2+0], restVertexPosition[i][1]+u[i*2+1]));
}

int TriangleMeshRendering2D::GetClosestVertex(const Vec2d & pos)
{
  double closestDist2 = DBL_MAX;
  double candidateDist2;
  int indexClosest = 0;
  for(int i=0; i< getNumVertices(); i++)
  {
    Vec2d relPos = getPosition(i) - pos;
    if ((candidateDist2 = dot(relPos,relPos)) < closestDist2)
    {
      closestDist2 = candidateDist2;
      indexClosest = i;
    }
  }
  return indexClosest;
}

