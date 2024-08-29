/*************************************************************************
 *                                                                       *
 * Vega FEM Simulation Library Version 4.0                               *
 *                                                                       *
 * "Stencil Force Model" library , Copyright (C) 2018 USC                *
 * All rights reserved.                                                  *
 *                                                                       *
 * Code authors: Bohan Wang, Jernej Barbic                               *
 * http://www.jernejbarbic.com/vega                                      *
 *                                                                       *
 * Research: Jernej Barbic, Hongyi Xu, Yijing Li,                        *
 *           Danyong Zhao, Bohan Wang,                                   *
 *           Fun Shing Sin, Daniel Schroeder,                            *
 *           Doug L. James, Jovan Popovic                                *
 *                                                                       *
 * Funding: National Science Foundation, Link Foundation,                *
 *          Singapore-MIT GAMBIT Game Lab,                               *
 *          Zumberge Research and Innovation Fund at USC,                *
 *          Sloan Foundation, Okawa Foundation,                          *
 *          USC Annenberg Foundation                                     *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of the BSD-style license that is            *
 * included with this library in the file LICENSE.txt                    *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the file     *
 * LICENSE.TXT for more details.                                         *
 *                                                                       *
 *************************************************************************/

#include "forceModelAssembler2D.h"
#include <cassert>
#include <iostream>

using namespace std;

ForceModelAssembler2D::ForceModelAssembler2D(StencilForceModel *eleFM) : stencilForceModel(eleFM)
{
  r = stencilForceModel->Getr();
  SparseMatrixOutline *smo = new SparseMatrixOutline(r);
  SparseMatrixOutline *smo1 = new SparseMatrixOutline(r / 2);
  for (int eltype = 0; eltype < stencilForceModel->GetNumStencilTypes(); eltype++) 
  {
    int nelev = stencilForceModel->GetNumStencilVertices(eltype);
    for (int ele = 0; ele < stencilForceModel->GetNumStencils(eltype); ele++) 
    {
      const int *vertexIndices = stencilForceModel->GetStencilVertexIndices(eltype, ele);

      for (int vi = 0; vi < nelev; vi++) 
      {
        for (int vj = 0; vj < nelev; vj++) 
        {
          smo->AddBlock2x2Entry(vertexIndices[vi], vertexIndices[vj]);
          smo1->AddEntry(vertexIndices[vi], vertexIndices[vj]);
        }
      }
    }
  }

  // compute stiffness matrix topology
  Ktemplate = new SparseMatrix(smo);
  delete smo;

  SparseMatrix *vertexK = new SparseMatrix(smo1);
  delete smo1;

  inverseIndices.resize(stencilForceModel->GetNumStencilTypes());
  for (int eltype = 0; eltype < stencilForceModel->GetNumStencilTypes(); eltype++) 
  {
    int nelev = stencilForceModel->GetNumStencilVertices(eltype);
    std::vector<int> &indices = inverseIndices[eltype];
    indices.resize(nelev * nelev * stencilForceModel->GetNumStencils(eltype));

    for (int ele = 0; ele < stencilForceModel->GetNumStencils(eltype); ele++) 
    {
      const int *vertexIndices = stencilForceModel->GetStencilVertexIndices(eltype, ele);
      int *inverseVtxIdx = indices.data() + ele * nelev * nelev;

      for (int vi = 0; vi < nelev; vi++) 
      {
        for (int vj = 0; vj < nelev; vj++) 
        {
          int vtxColIdx = vertexK->GetInverseIndex(vertexIndices[vi], vertexIndices[vj]);
          assert(vtxColIdx >= 0);
          inverseVtxIdx[vj * nelev + vi] = vtxColIdx;
        } // end vi
      } // end vj
    } // end ele
  } // end ele type

  delete vertexK;

  // initialize all necessary buffers
  bufferExamplars.resize(stencilForceModel->GetNumStencilTypes());
  for (int eltype = 0; eltype < stencilForceModel->GetNumStencilTypes(); eltype++) 
  {
    int nelev = stencilForceModel->GetNumStencilVertices(eltype);
    bufferExamplars[eltype].resize(nelev * 2 + nelev * nelev * 4);
  }
}

ForceModelAssembler2D::~ForceModelAssembler2D()
{
}

void ForceModelAssembler2D::GetTangentStiffnessMatrixTopology(SparseMatrix ** tangentStiffnessMatrix)
{
  *tangentStiffnessMatrix = new SparseMatrix(*Ktemplate);
  std::vector<double> zeros(stencilForceModel->Getr(), 0.0);
  GetEnergyAndForceAndMatrix(zeros.data(), nullptr, nullptr, *tangentStiffnessMatrix);
}

void ForceModelAssembler2D::GetEnergyAndForceAndMatrix(const double * u, double * energy, double * internalForces, SparseMatrix * tangentStiffnessMatrix)
{
  // reset to zero
  if (internalForces)
    memset(internalForces, 0, sizeof(double) * r);

  if (tangentStiffnessMatrix)
    tangentStiffnessMatrix->ResetToZero();

  for (int eltype = 0; eltype < stencilForceModel->GetNumStencilTypes(); eltype++) 
  {
    int nelev = stencilForceModel->GetNumStencilVertices(eltype);
    int nele = stencilForceModel->GetNumStencils(eltype);

    for (int ele = 0; ele < nele; ele++) 
    {
      double *fEle = bufferExamplars[eltype].data();
      double *KEle = bufferExamplars[eltype].data() + nelev * 2;

      double energyEle = 0;

      stencilForceModel->GetStencilLocalEnergyAndForceAndMatrix(eltype, ele, u,
        (energy ? &energyEle : nullptr),
        (internalForces ? fEle : nullptr),
        (tangentStiffnessMatrix ? KEle : nullptr)
      );

      const int *vIndices = stencilForceModel->GetStencilVertexIndices(eltype, ele);

      if (internalForces) 
      {
        for (int v = 0; v < nelev; v++) 
        {
          internalForces[vIndices[v] * 2] += fEle[v * 2];
          internalForces[vIndices[v] * 2 + 1] += fEle[v * 2 + 1];
        }
      }

      if (tangentStiffnessMatrix) 
      {
        const int *vtxColIndices = inverseIndices[eltype].data() + ele * nelev * nelev;

        // write matrices in place
        for (int va = 0; va < nelev; va++) 
        {
          int vIdxA = vIndices[va];
          for (int vb = 0; vb < nelev; vb++) 
          {
            int columnIndexCompressed = vtxColIndices[vb * nelev + va];

            for (int i = 0; i < 2; i++) 
            {
              for (int j = 0; j < 2; j++) 
              {
                int row = 2 * vIdxA + i;
                int columnIndex = 2 * columnIndexCompressed + j;

                int local_row = 2 * va + i;
                int local_col = 2 * vb + j;

                tangentStiffnessMatrix->AddEntry(row, columnIndex, KEle[local_col * nelev * 2 + local_row]);
              } // i
            } // j
          } // vb
        } // va
      }

      if (energy) 
      {
        *energy += energyEle;
      }
    }
  }

  if (internalForces)
  {
    for (int vi = 0; vi < stencilForceModel->Getr() / 2; vi++) {
      double g[2];
      stencilForceModel->GetVertexGravityForce(vi, g);
      internalForces[vi * 2 + 1] += g[1];
    }
  }
}

double ForceModelAssembler2D::GetElasticEnergy(const double *u)
{
  double E = 0;
  GetEnergyAndForceAndMatrix(u, &E, nullptr, nullptr);

  return E;
}

void ForceModelAssembler2D::GetInternalForce(const double * u, double * internalForces)
{
  GetEnergyAndForceAndMatrix(u, nullptr, internalForces, nullptr);
}

void ForceModelAssembler2D::GetTangentStiffnessMatrix(const double * u, SparseMatrix *tangentStiffnessMatrix)
{
  GetEnergyAndForceAndMatrix(u, nullptr, nullptr, tangentStiffnessMatrix);
}

void ForceModelAssembler2D::GetForceAndMatrix(const double * u, double * internalForces, SparseMatrix * tangentStiffnessMatrix)
{
  GetEnergyAndForceAndMatrix(u, nullptr, internalForces, tangentStiffnessMatrix);
}
