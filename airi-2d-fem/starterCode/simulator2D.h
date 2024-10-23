/* 
  CSCI 520 Computer Animation and Simulation
  Programming Assignment:
  2D Simulator for Mass spring, Linear FEM and Corotational FEM

  Mianlun Zheng and Jernej Barbic
  University of Southern California
*/

#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <cstdio>
#include <cassert>
#include <fstream>
#include <memory>
#include <cctype> 

#ifdef __APPLE__
  #include "TargetConditionals.h"
#endif

#include "openGLHelper.h"
#include "minivector.h"
#include "ortho2DCamera.h"
#include "performanceCounter.h"
#include "configFile.h"
#include "sparseMatrix.h"
#include "stencilForceModel.h"
#include "forceModelAssembler2D.h"
#include "stencilForceModel2D.h"
#include "triangleMesh2D.h"
#include "triangleMeshRendering2D.h"
#include "baseModel2D.h"
#include "corotationalLinearFEM2D.h"
#include "linearFEM2D.h"
#include "massSpring2D.h"
#include "implicitBackwardEulerSparse.h"
#include "graph.h"
#include "averagingBuffer.h"
#include "listIO.h"
#include "saveScreenShot.h"

//ImGui API
#if defined(_WIN32) || defined(WIN32)  
  #include <imgui/imgui.h>
  #include <imgui/examples/imgui_impl_glut.h>
  #include <imgui/examples/imgui_impl_opengl2.h>
#elif defined(__APPLE__) || defined(linux) || defined (__linux__)
  #include <imgui/imgui.h>
  #include <imgui/imgui_impl_glut.h>
  #include <imgui/imgui_impl_opengl2.h>
#endif

using namespace std;

// openGL state
static char windowTitleBase[4096] = "Simulator2D";
static int windowID = 0;
static int windowWidth = 1600;//512;
static int windowHeight = 900; //512;
static int leftMouseButton=0, middleMouseButton=0, rightMouseButton=0;
static int g_vMousePos[2] = {0,0};
static int dragStartX, dragStartY;
static bool shiftPressed = false, controlPressed = false, altPressed = false;
static int graphicFrame = 0;
static bool enableMaterialRender = true;
static bool enableMeshRender = false;
static bool enableWireRender = true;
static bool enablePointRender = false;

Vec3d backgroundColor(1.0); // 196.0 / 256);
bool pauseSimulation = false;

static int activeNode = -1;

shared_ptr<Ortho2DCamera> camera;
shared_ptr<TriangleMesh2D> triangleMesh2D;
shared_ptr<TriangleMeshRendering2D> triangleMeshRendering2D;

PerformanceCounter simulationCounter;
AveragingBuffer fpsBuffer(100);
double FPS;

SparseMatrix * massMatrix = nullptr;
vector<double> vertexMasses2D;
Graph * meshGraph = nullptr;

IntegratorBase * integratorBase = nullptr;
IntegratorBaseSparse * integratorBaseSparse = nullptr;
ImplicitNewmarkSparse * implicitNewmarkSparse = nullptr;
ForceModel * forceModel = nullptr;
StencilForceModel * stencilForceModel = nullptr;
StencilForceModel2D * stencilForceModel2D = nullptr;
ForceModelAssembler2D *forceModelAssembler2D = nullptr;
BaseModel2D * baseModel2D = nullptr;

int numVertices;
vector<double> u;
vector<double> uvel;
vector<double> uaccel;
vector<double> f_ext;
vector<double> f_extBase;
vector<double> uSecondary;
vector<double> uInitial;
vector<double> velInitial;

int numSolverThreads = 1;
double epsilon = 1e-6;
int maxIterations = 1;
int numFixedVertices = 0;
int * fixedVertices = nullptr;
int numFixedDOFs;
static bool key_h_pressed = false;
int pulledVertex = -1;
float newmarkBeta = 0.25;
float newmarkGamma = 0.5;

// configFile
static ConfigFile simulator2DConfig;
static string simulator2DConfigFilename;
// simulation parameters
float dampingMassCoef = 0.0; // Rayleigh mass damping
float dampingStiffnessCoef = 0.00; // Rayleigh stiffness damping
float timeStep = 1.0/30;
static string triangleMeshFilename;
static string forceModelName;
double focusPositionX = 0.0;
double focusPositionY = 0.0;
double cameraScale = 1.0;
bool addGravity = 0;
double g=0.981;
double deformableObjectCompliance = 1.0;
int forceNeighborhoodSize = 2;
int substepsPerTimeStep = 1;
bool staticSolver = 0;
bool renderFixedVertices = true;
string fixedVerticesFilename = "_none";
string forceMaterialName = "_none";
string materialName = "_none";
string solverName = "_none";


// render parameter
static float pointSize = 3.0;
static float wireSize = 1.0;
static Vec3d pointColor(1.0, 0.0, 0.0);
static Vec3d wireColor(0.0);
static Vec3d meshColor(0.3);

enum forceMaterialType { MASSSPRING, COROTLINFEM, LINFEM, UNSPECIFIED } forceMaterial = UNSPECIFIED;
enum solverType { IMPLICITNEWMARK, IMPLICITBACKWARDEULER, UNKNOWN } solver = UNKNOWN;

double materialLambda = 1.0;
double materialMu = 1.0;
double materialDensity = 1.0;
double massSpringStiffness = 1.0;
float internalForceScalingFactor = 1.0;
