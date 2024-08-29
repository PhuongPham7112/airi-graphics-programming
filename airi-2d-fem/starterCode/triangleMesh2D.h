/* 
  CSCI 520 Computer Animation and Simulation
  Programming Assignment:
  2D Simulator for Mass spring, Linear FEM and Corotational FEM

  Mianlun Zheng and Jernej Barbic
  University of Southern California
*/

#pragma once

#include <math.h>
#include <string>
#include <vector>
#include <map>
#include <iostream>
#include <set>
#include <algorithm>
#include <assert.h>
#include <exception>
#include <memory>
#include <functional>
#include <string>
#include "minivector.h"

using namespace std;

#define CROSSPRODUCT(x1,y1,z1,x2,y2,z2,x,y,z)\
\
  x = (y1) * (z2) - (y2) * (z1);\
  y = (x2) * (z1) - (x1) * (z2);\
  z = (x1) * (y2) - (x2) * (y1)
  
class TriangleMesh2DException : public std::exception
{
public:
  TriangleMesh2DException(const std::string & reason);
  TriangleMesh2DException(const std::string & reason, const std::string & filename, unsigned int line);
  virtual ~TriangleMesh2DException() throw() {}
  const std::string & getReason() const throw() { return reason; }
  virtual const char * what() const throw() { return reason.c_str(); }

protected:
  std::string reason;
};

class TriangleMesh2D
{
public:
    class Material;
    class Group;

    TriangleMesh2D(const std::string &fileName, int verbose);
    ~TriangleMesh2D();

    inline int getNumVertices() {return numVertices;}
    inline int getNumTriangles() {return numTriangles;}
    inline int getNumEdges() {return numEdges;}
    inline Vec2d getPosition(int vertexIndex){return vertexPosition[vertexIndex];}
    inline std::vector<Vec2d> getPosition(){return vertexPosition;}
    inline const std::vector<Vec2d> getRestPosition(){return restVertexPosition;}
    inline Vec2d getRestPosition(int vertexIndex){return restVertexPosition[vertexIndex];}
    inline std::vector<Vec3i> getTriangles(){return triangles;}
    inline void setPosition(int vertexIndex, const Vec2d & position) {vertexPosition[vertexIndex] = position;}
    inline void setPosition(const std::vector<Vec2d> & vertexPosition_){vertexPosition = vertexPosition_;}
    inline void resetMesh() {vertexPosition = restVertexPosition;}
    inline int getNumMaterials() const { return materials.size(); }
    inline std::string getMeshName(){ return triangleMeshFilename;}
    inline std::vector<Material> getMaterial() {return materials;}
    inline std::vector<Group> getGroup() {return groups;}
    inline const int * getVertexIndices(int element) const { return &elements[element*numElementVertices]; }
    inline int getVertexIndex(int element, int vertex) { return elements[element*numElementVertices + vertex]; }
    int addDefaultMaterial();
    inline int getNumElementVertices() { return numElementVertices; }
    inline int getNumElements() { return numElements; }
    inline void setMaterialLambda(double lambda) { this->lambda = lambda; }        
    inline void setMaterialMu(double mu) { this->mu = mu; }
    inline void setMaterialDensity(double density) { this->density = density; }
    inline void setMassSpringStiffness(double stiffness) { this->stiffness = stiffness; }
    inline double getLambda() { return lambda; }
    inline double getMu() { return mu; }
    inline double getMassSpringStiffness() { return stiffness; }
    inline double getDensity() { return density; }

    int loadFromAscii(const string & filename, int verbose);
    void parseMaterials(const std::string & objMeshFilename, const std::string & materialFilename, int verbose);
    inline void addMaterial(const Material & material) { materials.push_back(material); }
    inline void addMaterial(const std::string & name, const Vec3d & Ka, const Vec3d & Kd, const Vec3d & Ks, double shininess, const std::string textureFilename=std::string()) { materials.emplace_back(name, Ka, Kd, Ks, shininess, textureFilename);}
    void fgets_(char * s, int n, FILE * stream);
    static void removeWhitespace(char * s);
    static void convertWhitespaceToSingleBlanks(char * s);
    void initTriangleLookup();
    void exportGeometry(std::vector<Vec3d> & vertices, std::vector<Vec3i> & triangles) const;
    void exportTriangles(std::vector<Vec3i> & triangles) const;
    double getElementArea(int element);
    
    class Group
    {
      public:
        explicit Group(const std::string & name_ = "defaultGroup", unsigned int materialIndex_=0)
          : name(name_), materialIndex(materialIndex_) {}
        
        inline const std::string & getName() const { return name; }
        inline void addFace(Vec3i face) { faces.push_back(face); }
        inline void setMaterialIndex(int materialIndex_) { materialIndex = materialIndex_; }
        inline int getMaterialIndex() { return materialIndex; }
        inline int getNumFaces() const { return faces.size(); }
        inline Vec3i getFace(int faceIndex) {return faces[faceIndex]; } 
      protected:
        friend class triangleMesh2D;
        std::string name;
        int materialIndex;
        std::vector< Vec3i > faces;
    };

    class Material
    {
      public:
        explicit Material(const std::string name_="default", const Vec3d & Ka_=Vec3d(0.2), const Vec3d & Kd_=Vec3d(0.6), const Vec3d & Ks_=Vec3d(0.0), double shininess_=65.0, const std::string textureFilename_=std::string()):
          Ka(Ka_), Kd(Kd_), Ks(Ks_), shininess(shininess_), alpha(1.0), name(name_), textureFilename(textureFilename_) {}

        inline const std::string & getName() const { return name; }
        inline const Vec3d & getKa() const { return Ka; }
        inline const Vec3d & getKd() const { return Kd; }
        inline const Vec3d & getKs() const { return Ks; }
        inline double getShininess() const { return shininess; }
        inline double getAlpha() const { return alpha; }
        inline void setKa(const Vec3d & Ka_) { Ka = Ka_; }
        inline void setKd(const Vec3d & Kd_) { Kd = Kd_; }
        inline void setKs(const Vec3d & Ks_) { Ks = Ks_; }
        inline void setShininess(double shininess_) { shininess = shininess_; }
        inline void setAlpha(double alpha_) { alpha = alpha_; }
        
      protected:
        Vec3d Ka, Kd, Ks;
        double shininess;
        double alpha;
        std::string name;
        std::string textureFilename;
    };


private:
    int numVertices = 0;
    std::vector<Vec2d> vertexPosition;
    std::vector<Vec2d> restVertexPosition;

    std::vector<Vec3d> textureCoordinates;

    int numTriangles = 0;
    std::vector<Vec3i> triangles;

    int numEdges = 0;
    std::vector<Vec2i> edges;

    std::vector<int> elements;
    int numElements;
    int numElementVertices = 3;

    std::string triangleMeshFilename;

    std::vector< Material > materials;
    std::vector< Group > groups;

    double lambda = 1.0; // for fem
    double mu = 1.0; // for fem
    double stiffness = 1.0; // for mass spring
    double density = 1.0;
};

