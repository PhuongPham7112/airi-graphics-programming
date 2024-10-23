#include "skinning.h"
#include "vec3d.h"
#include <algorithm>
#include <cassert>
#include <iostream>
#include <fstream>
using namespace std;

// CSCI 520 Computer Animation and Simulation
// Jernej Barbic and Yijing Li

Skinning::Skinning(int numMeshVertices, const double * restMeshVertexPositions,
    const std::string & meshSkinningWeightsFilename)
{
  this->numMeshVertices = numMeshVertices;
  this->restMeshVertexPositions = restMeshVertexPositions;

  cout << "Loading skinning weights..." << endl;
  ifstream fin(meshSkinningWeightsFilename.c_str());
  assert(fin);
  int numWeightMatrixRows = 0, numWeightMatrixCols = 0;
  fin >> numWeightMatrixRows >> numWeightMatrixCols;
  assert(fin.fail() == false);
  assert(numWeightMatrixRows == numMeshVertices);
  int numJoints = numWeightMatrixCols;

  vector<vector<int>> weightMatrixColumnIndices(numWeightMatrixRows);
  vector<vector<double>> weightMatrixEntries(numWeightMatrixRows);
  fin >> ws;
  while(fin.eof() == false)
  {
    int rowID = 0, colID = 0;
    double w = 0.0;
    fin >> rowID >> colID >> w;
    weightMatrixColumnIndices[rowID].push_back(colID);
    weightMatrixEntries[rowID].push_back(w);
    assert(fin.fail() == false);
    fin >> ws;
  }
  fin.close();

  // Build skinning joints and weights.
  numJointsInfluencingEachVertex = 0;
  for (int i = 0; i < numMeshVertices; i++)
    numJointsInfluencingEachVertex = std::max(numJointsInfluencingEachVertex, (int)weightMatrixEntries[i].size());
  assert(numJointsInfluencingEachVertex >= 2);

  // Copy skinning weights from SparseMatrix into meshSkinningJoints and meshSkinningWeights.
  meshSkinningJoints.assign(numJointsInfluencingEachVertex * numMeshVertices, 0);
  meshSkinningWeights.assign(numJointsInfluencingEachVertex * numMeshVertices, 0.0);
  for (int vtxID = 0; vtxID < numMeshVertices; vtxID++)
  {
    vector<pair<double, int>> sortBuffer(numJointsInfluencingEachVertex);
    for (size_t j = 0; j < weightMatrixEntries[vtxID].size(); j++)
    {
      int frameID = weightMatrixColumnIndices[vtxID][j];
      double weight = weightMatrixEntries[vtxID][j];
      sortBuffer[j] = make_pair(weight, frameID);
    }
    sortBuffer.resize(weightMatrixEntries[vtxID].size());
    assert(sortBuffer.size() > 0);
    sort(sortBuffer.rbegin(), sortBuffer.rend()); // sort in descending order using reverse_iterators
    for(size_t i = 0; i < sortBuffer.size(); i++)
    {
      meshSkinningJoints[vtxID * numJointsInfluencingEachVertex + i] = sortBuffer[i].second;
      meshSkinningWeights[vtxID * numJointsInfluencingEachVertex + i] = sortBuffer[i].first;
    }

    // Note: When the number of joints used on this vertex is smaller than numJointsInfluencingEachVertex,
    // the remaining empty entries are initialized to zero due to vector::assign(XX, 0.0) .
  }
}

void Skinning::applySkinning(const RigidTransform4d * jointSkinTransforms, double * newMeshVertexPositions) const
{
    int mode = 0;
  // Students should implement this
  for(int i=0; i<numMeshVertices; i++)
  {
    // i = vertex id
    if (mode == 0)
    {
        Vec4d currSkinningPos = Vec4d(restMeshVertexPositions[3 * i + 0],
            restMeshVertexPositions[3 * i + 1],
            restMeshVertexPositions[3 * i + 2],
            1.0);
        Vec4d newSkinningPos = Vec4d(0.0, 0.0, 0.0, 0.0);
        for (int j = 0; j < numJointsInfluencingEachVertex; j++)
        {
            int jointIdx = meshSkinningJoints[i * numJointsInfluencingEachVertex + j]; // get joint index that affect mesh vertex i
            double weight = meshSkinningWeights[i * numJointsInfluencingEachVertex + j];
            RigidTransform4d transformMatrix = jointSkinTransforms[jointIdx];
            newSkinningPos += weight * transformMatrix * currSkinningPos;
        }
        newMeshVertexPositions[3 * i + 0] = newSkinningPos[0];
        newMeshVertexPositions[3 * i + 1] = newSkinningPos[1];
        newMeshVertexPositions[3 * i + 2] = newSkinningPos[2];
    }
    else
    {
        // dual quat skinning
        glm::ddualquat dlb;
        dlb.real = glm::highp_dquat(0.0, 0.0, 0.0, 0.0);
        dlb.dual = glm::highp_dquat(0.0, 0.0, 0.0, 0.0);
        double rotation[9];
        for (int j = 0; j < numJointsInfluencingEachVertex; j++)
        {
            int jointIdx = meshSkinningJoints[i * numJointsInfluencingEachVertex + j]; // get joint index that affect mesh vertex i
            double weight = meshSkinningWeights[i * numJointsInfluencingEachVertex + j];
            RigidTransform4d transformMatrix = jointSkinTransforms[jointIdx];
        
            // rotation component
            transformMatrix.getRotation().convertToArray(rotation);
            glm::highp_dmat3 rotationMat = glm::highp_dmat3(rotation[0], rotation[3], rotation[6], // col 1
                rotation[1], rotation[4], rotation[7], // col 2
                rotation[2], rotation[5], rotation[8]); // col 3
            glm::dquat rotationQuat = glm::dquat(rotationMat);

            // translation component
            Vec3d translation = transformMatrix.getTranslation();
            glm::highp_dvec3 translationVec = glm::highp_dvec3(translation[0], translation[1], translation[2]);

            // converting rigid transform to dual quaternion
            glm::ddualquat q = glm::ddualquat(rotationQuat, translationVec);
            dlb.real += q.real * weight;
            dlb.dual += q.dual * weight;
        }

        double c0_len = glm::length(dlb.real);
        glm::dquat c0 = dlb.real / c0_len;
        glm::dquat c1 = dlb.dual / c0_len;

        double w0 = c0.w;
        double x0 = c0.x;
        double y0 = c0.y;
        double z0 = c0.z;

        double we = c1.w;
        double xe = c1.x;
        double ye = c1.y;
        double ze = c1.z;

        double t0 = 2.0 * (-we * x0 + xe * w0 - ye * z0 + ze * y0);
        double t1 = 2.0 * (-we * y0 + xe * z0 + ye * w0 - ze * x0);
        double t2 = 2.0 * (-we * z0 - xe * y0 + ye * x0 + ze * w0);

        glm::dmat4x3 M; // 4 columns of 3 components matrix 
        M[0] = glm::dvec3(1 - 2 * y0 * y0 - 2 * z0 * z0, 
            2 * x0 * y0 + 2 * w0 * z0, 
            2 * x0 * z0 - 2 * w0 * y0); // col 0
        M[1] = glm::dvec3(2 * x0 * y0 - 2 * w0 * z0,
            1 - 2 * x0 * x0 - 2 * z0 * z0,
            2 * y0 * z0 + 2 * w0 * x0); // col 1
        M[2] = glm::dvec3(2 * x0 * z0 + 2 * w0 * y0,
            2 * y0 * z0 - 2 * w0 * x0,
            1 - 2 * x0 * x0 - 2 * y0 * y0); // col 2
        M[3] = glm::dvec3(t0, t1, t2); // col 3

        glm::dvec3 newSkinningPos = M * glm::dvec4(restMeshVertexPositions[3 * i + 0],
            restMeshVertexPositions[3 * i + 1],
            restMeshVertexPositions[3 * i + 2],
            1.0);

        newMeshVertexPositions[3 * i + 0] = newSkinningPos[0];
        newMeshVertexPositions[3 * i + 1] = newSkinningPos[1];
        newMeshVertexPositions[3 * i + 2] = newSkinningPos[2];
    }
  }
}

