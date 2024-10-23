/* 
  CSCI 520 Computer Animation and Simulation
  Programming Assignment:
  2D Simulator for Mass spring, Linear FEM and Corotational FEM

  Mianlun Zheng and Jernej Barbic
  University of Southern California
*/

#pragma once

#include "vec2d.h"
#include <vector>
#include "triangleMesh2D.h"

class TriangleMeshRendering2D
{
  public:
    TriangleMeshRendering2D(TriangleMesh2D * mesh);
    ~TriangleMeshRendering2D();

    inline int getNumVertices() {return numVertices;}
    inline int getNumTriangles() {return numTriangles;}
    inline Vec2d getPosition(int vertexIndex){return vertexPosition[vertexIndex];}
    inline std::vector<Vec2d> getPosition(){return vertexPosition;}
    inline std::vector<Vec2d> getRestPosition(){return restVertexPosition;}
    inline Vec2d getRestPosition(int vertexIndex){return restVertexPosition[vertexIndex];}
    inline std::vector<Vec3i> getTriangles(){return triangles;}
    inline void setPosition(int vertexIndex, const Vec2d & position) {vertexPosition[vertexIndex] = position;}
    inline void setPosition(const std::vector<Vec2d> & vertexPosition_){vertexPosition = vertexPosition_;}
    inline void resetMesh() {vertexPosition = restVertexPosition;}

    inline void setPointSize(float pointSize_) {pointSize = pointSize_;}
    inline void setPointColor(Vec3d pointColor_) {pointColor = pointColor_;}
    inline void setWireSize(float wireSize_) {wireSize = wireSize_;}
    inline void setWireColor(Vec3d wireColor_) {wireColor = wireColor_;}
    inline void setMeshColor(Vec3d meshColor_) {meshColor = meshColor_;}

    void drawMeshWire();
    void drawMeshPoint();
    void drawMeshSurface();
    void drawMaterials();

    void setVertexDeformations(const double *u);
    int GetClosestVertex(const Vec2d & pos);

  private:
    int numVertices = 0;
    std::vector<Vec2d> vertexPosition;
    std::vector<Vec2d> restVertexPosition;

    int numTriangles = 0;
    std::vector<Vec3i> triangles;

    float pointSize;
    Vec3d pointColor;
    float wireSize;
    Vec3d wireColor;
    Vec3d meshColor;

    std::vector<TriangleMesh2D::Material> materials;
    std::vector<TriangleMesh2D::Group> groups;
};

