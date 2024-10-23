/* 
  CSCI 520 Computer Animation and Simulation
  Programming Assignment:
  2D Simulator for Mass spring, Linear FEM and Corotational FEM

  Mianlun Zheng and Jernej Barbic
  University of Southern California
*/

#include "simulator2D.h"

static void reshape(int x, int y)
{
  // cout << "reshape" << endl;
  ImGui_ImplGLUT_ReshapeFunc(x, y);
  glViewport(0,0,x,y);

  windowWidth = x;
  windowHeight = y;

  // Setup model transformations.
  glMatrixMode ( GL_PROJECTION );
  glLoadIdentity ();

  //gluPerspective(45.0f, 1.0 * x / y, 0.1, 1000);
  //gluOrtho2D(-1.5,1.5,-1.0,1.0);
  if(x>y)
    gluOrtho2D(-1.0*x/y,1.0*x/y,-1.0,1.0);
  else
    gluOrtho2D(-1.0,1.0,-1.0*y/x,1.0*y/x);
  
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}

/******************************************************************************
 Sets up graphics pipeline
******************************************************************************/
static void initGraphics(int windowWidth, int windowHeight)
{
  glClearColor(backgroundColor[0], backgroundColor[1], backgroundColor[2], 0.0);

  glShadeModel(GL_SMOOTH);
  glEnable(GL_POLYGON_SMOOTH);
  glEnable(GL_LINE_SMOOTH);

  printf ("Graphics initialization complete.\n");

  const char *openGLVersion = (const char*)glGetString(GL_VERSION);
  printf("OpenGL version: %s\n", openGLVersion);
}

void reset_callback()
{
  triangleMeshRendering2D->resetMesh();
  integratorBase->ResetToRest();
  integratorBase->SetState(uInitial.data());
  f_ext.assign(2 * numVertices, 0.0);
  f_extBase.assign(2 * numVertices, 0.0);
  integratorBase->SetExternalForcesToZero();
  u.assign(integratorBase->Getq(), integratorBase->Getq() + 2 * numVertices);
  triangleMeshRendering2D->setVertexDeformations(u.data());
}

void exitCallback()
{
  cout << endl << "======================================" << endl;
  cout << "Exiting..." << endl;
  exit(0);
}

static void displayImGUI()
{
  ImGui_ImplOpenGL2_NewFrame();
  ImGui_ImplGLUT_NewFrame();

  ImGui::Begin("Controls");

  ImGui::PushItemWidth(90);

  ImGui::Text("Overall FPS: %.1f", FPS);

  ImGui::Checkbox("Render Material", &enableMaterialRender); ImGui::SameLine();
  ImGui::Checkbox("Render Point", &enablePointRender);
  ImGui::Checkbox("Render Wireframe", &enableWireRender); ImGui::SameLine();
  ImGui::Checkbox("Render Mesh", &enableMeshRender); 

  if(enablePointRender && ImGui::CollapsingHeader("Point Render"))
  {
    ImGui::Indent();
    float rp = pointColor[0], gp = pointColor[1], bp = pointColor[2];
    bool pointColorChanged = false;
    if(ImGui::SliderFloat("Point Red: ", &rp, 0.0, 1.0, "%.1f"))
    {
      pointColor[0] = rp;
      pointColorChanged = true;
    }
    if(ImGui::SliderFloat("Point Green: ", &gp, 0.0, 1.0, "%.1f"))
    {
      pointColor[1] = gp;
      pointColorChanged = true;
    }
    if(ImGui::SliderFloat("Point Blue: ", &bp, 0.0, 1.0, "%.1f"))
    {
      pointColor[2] = bp;
      pointColorChanged = true;
    }
    if(pointColorChanged)
      triangleMeshRendering2D->setPointColor(pointColor);
    ImGui::Separator();
    bool pointSizeChanged = false;
    if(ImGui::SliderFloat("Point Size: ", &pointSize, 0.1, 15.0, "%.1f"))
    {
      pointSizeChanged = true;
    }
    if(pointSizeChanged)
      triangleMeshRendering2D->setPointSize(pointSize);
    ImGui::Unindent();
  }

  if(enableWireRender && ImGui::CollapsingHeader("Wire Render"))
  {
    ImGui::Indent();
    float rw = wireColor[0], gw = wireColor[1], bw = wireColor[2];
    bool wireColorChanged = false;
    if(ImGui::SliderFloat("Wire Red: ", &rw, 0.0, 1.0, "%.1f"))
    {
      wireColor[0] = rw;
      wireColorChanged = true;
    }
    if(ImGui::SliderFloat("Wire Green: ", &gw, 0.0, 1.0, "%.1f"))
    {
      wireColor[1] = gw;
      wireColorChanged = true;
    }
    if(ImGui::SliderFloat("Wire Blue: ", &bw, 0.0, 1.0, "%.1f"))
    {
      wireColor[2] = bw;
      wireColorChanged = true;
    }
    if(wireColorChanged)
      triangleMeshRendering2D->setWireColor(wireColor);
    ImGui::Separator();
    bool wireSizeChanged = false;
    if(ImGui::SliderFloat("Wire Size: ", &wireSize, 0.1, 15.0, "%.1f"))
    {
      wireSizeChanged = true;
    }
    if(wireSizeChanged)
      triangleMeshRendering2D->setWireSize(wireSize);
    ImGui::Unindent();
  }

  if(enableMeshRender && ImGui::CollapsingHeader("Mesh Render"))
  {
    ImGui::Indent();
    float rm = meshColor[0], gm = meshColor[1], bm = meshColor[2];
    bool meshColorChanged = false;
    if(ImGui::SliderFloat("Mesh Red: ", &rm, 0.0, 1.0, "%.1f"))
    {
      meshColor[0] = rm;
      meshColorChanged = true;
    }
    if(ImGui::SliderFloat("Mesh Green: ", &gm, 0.0, 1.0, "%.1f"))
    {
      meshColor[1] = gm;
      meshColorChanged = true;
    }
    if(ImGui::SliderFloat("Mesh Blue: ", &bm, 0.0, 1.0, "%.1f"))
    {
      meshColor[2] = bm;
      meshColorChanged = true;
    }
    if(meshColorChanged)
      triangleMeshRendering2D->setMeshColor(meshColor);
    ImGui::Unindent();
  }

  ImGui::Checkbox("Enable Gravity", &addGravity);
  if(ImGui::Checkbox("Static Solver", &staticSolver))
  {
    if (implicitNewmarkSparse)
      implicitNewmarkSparse->UseStaticSolver(staticSolver);
  }
  ImGui::Checkbox("Pause simulation", &pauseSimulation);
  ImGui::Checkbox("Render fixed vertices", &renderFixedVertices);
  if(ImGui::Button("Rest Mesh"))
    reset_callback();
  if(ImGui::InputFloat("Internal Force Scaling ", &internalForceScalingFactor, 0.1f, 10.0f))
    integratorBase->SetInternalForceScalingFactor(internalForceScalingFactor);
  if(ImGui::InputFloat("Mass Damping Coefficient", &dampingMassCoef, 0.1f, 10.0f))
  {
    if (dampingMassCoef < 0)
    dampingMassCoef = 0;
    integratorBase->SetDampingMassCoef(dampingMassCoef);
  }
  if(ImGui::InputFloat("Stifness Damping Coefficient", &dampingStiffnessCoef, 0.1f, 10.0f))
  {
    if (dampingStiffnessCoef < 0)
      dampingStiffnessCoef = 0;
    integratorBase->SetDampingStiffnessCoef(dampingStiffnessCoef);
  }
  if(ImGui::InputFloat("Time Step", &timeStep, 0.1f, 10.0f, "%.2f"))
  {
    if(timeStep < 0.0) 
      timeStep = 0.0;
    integratorBase->SetTimestep(timeStep / substepsPerTimeStep);
  }

  if(ImGui::InputInt("Substeps PerTimeStep", &substepsPerTimeStep, 1, 10))
  {
    if(substepsPerTimeStep < 1) 
      substepsPerTimeStep = 1;
    integratorBase->SetTimestep(timeStep / substepsPerTimeStep);
  }
  // ImGui::Checkbox("Static solver only", &enableStaticSolver);

  ImGui::Separator();
  if(ImGui::CollapsingHeader("Mouse buttons"))
  {
    ImGui::Indent();
      ImGui::Text("Left + drag: apply force");
      ImGui::Text("Middle + drag: zoom in/out");
    ImGui::Unindent();
  }

  ImGui::Separator();
  ImGui::Text("USC, CSCI 520, HW 4");

  ImGui::Separator();
  if(ImGui::Button("        Exit program        "))
  {
   	exit(0);
  }

  ImGui::End();
  ImGui::Render();
  ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());  
}

static void displayFunction(void)
{
  glClear(GL_COLOR_BUFFER_BIT);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  camera->look();

  // render the mesh
  if(enableMaterialRender)
    triangleMeshRendering2D->drawMaterials();
  if(enableMeshRender)
   triangleMeshRendering2D->drawMeshSurface();
  if(enableWireRender)
   triangleMeshRendering2D->drawMeshWire();
  if(enablePointRender)
   triangleMeshRendering2D->drawMeshPoint();

  // render the currently pulled vertex
  if(pulledVertex >=0)
  {
    glColor3f(0,1,0);
    Vec2d pulledVertexPos;
    pulledVertexPos = triangleMeshRendering2D->getPosition(pulledVertex);
  
    glPointSize(8.0);
    glBegin(GL_POINTS);
    glVertex2f(pulledVertexPos[0], pulledVertexPos[1]);
    glEnd();
  }

  // render model fixed vertices
  if (renderFixedVertices && numFixedVertices > 0)
  {
    glColor3f(1,0,0);
    glPointSize(12.0);
    glBegin(GL_POINTS);
    Vec2d fixedVertexPos(0.0);
    for(int i=0; i<numFixedVertices; i++)
    {
      fixedVertexPos = triangleMeshRendering2D->getPosition(fixedVertices[i]);
      glVertex2f(fixedVertexPos[0], fixedVertexPos[1]);
    }
    glEnd();
  }

  displayImGUI();
  glutSwapBuffers();
}


static void idleFunction(void)
{
  simulationCounter.StartCounter();
  glutSetWindow(windowID);

  if(!pauseSimulation)
  {
    f_ext.assign(f_extBase.begin(), f_extBase.end());
    if (addGravity)
    {
      for(int i = 0; i < numVertices; i++)
        f_ext[2*i+1] += -vertexMasses2D[2*i+1] * g * 0.01;
    }

    if (leftMouseButton) 
    {
      if (pulledVertex != -1)
      {
        double forceX = (g_vMousePos[0] - dragStartX);
        double forceY = -(g_vMousePos[1] - dragStartY);

        Vec2d externalForce = {forceX, forceY};
        externalForce *= deformableObjectCompliance;
        // print the external force from the mouse drag
        // printf("pulledVertex: %d, fx: %G fy: %G | %G %G\n", pulledVertex, forceX, forceY, externalForce[0], externalForce[1]);
        // register force on the pulled vertex and its neighborhood
        map<int, int> vtxDist = meshGraph->GetNeighborhoodWithDistance(pulledVertex, forceNeighborhoodSize-1);
        for(auto p : vtxDist)
        {
          int vtx = p.first, dist = p.second;
          double forceMagnitude = 1.0 * (forceNeighborhoodSize-dist) / forceNeighborhoodSize;
          Vec2d vtxForce = forceMagnitude * externalForce;
          vtxForce.addToArray(&f_ext[2*vtx]);
        }
      }
    }

    // set forces to the integrator
    integratorBaseSparse->SetExternalForces(f_ext.data());

    // timestep the dynamics 
    for(int i=0; i<substepsPerTimeStep; i++)
    {
      int code = integratorBase->DoTimestep();
      //printf("."); fflush(nullptr);

      if (code != 0)
      {
        printf("The integrator went unstable. Reduce the timestep, or increase the number of substeps per timestep.\n");
        integratorBase->ResetToRest();
        f_ext.assign(2 * numVertices, 0.0);
        f_extBase.assign(2 * numVertices, 0.0);
        
        integratorBase->SetExternalForcesToZero();
        break;
      }
    }

    u.assign(integratorBase->Getq(), integratorBase->Getq() + 2 * numVertices);

    triangleMeshRendering2D->setVertexDeformations(u.data());
  }

  simulationCounter.StopCounter();
  fpsBuffer.addValue(simulationCounter.GetElapsedTime());
  FPS = 1.0 / fpsBuffer.getAverage();

  glutPostRedisplay();
}

static void initSimulation()
{

  camera = make_shared<Ortho2DCamera>(focusPositionX, focusPositionY, 1);
  camera->setScale(cameraScale);

  // init mesh
  triangleMesh2D = make_shared<TriangleMesh2D>(triangleMeshFilename, 1);
  triangleMesh2D->setMaterialLambda(materialLambda);
  triangleMesh2D->setMaterialMu(materialMu);
  triangleMesh2D->setMaterialDensity(materialDensity);
  triangleMesh2D->setMassSpringStiffness(massSpringStiffness);
  triangleMeshRendering2D = make_shared<TriangleMeshRendering2D>(triangleMesh2D.get());
  triangleMeshRendering2D->setPointSize(pointSize);
  triangleMeshRendering2D->setWireSize(wireSize);
  triangleMeshRendering2D->setPointColor(pointColor);
  triangleMeshRendering2D->setWireColor(wireColor);
  triangleMeshRendering2D->setMeshColor(meshColor);

  // init force model
  if(forceMaterial == COROTLINFEM )
  { 
    printf("Force model: corotational linear FEM\n");
    CorotationalLinearFEM2D * corotationalLinearFEM2D = new CorotationalLinearFEM2D(triangleMesh2D.get());
    baseModel2D = corotationalLinearFEM2D;
    stencilForceModel2D = new StencilForceModel2D(baseModel2D);
  }
  else if(forceMaterial == LINFEM)
  {
    printf("Force model: linear FEM\n");
    LinearFEM2D * linearFEM2D = new LinearFEM2D(triangleMesh2D.get());
    baseModel2D = linearFEM2D;
    stencilForceModel2D = new StencilForceModel2D(baseModel2D);
  }
  else if(forceMaterial == MASSSPRING)
  {
    printf("Force model: mass spring\n");
    MassSpring2D * massSpring2D = new MassSpring2D(triangleMesh2D.get());
    baseModel2D = massSpring2D;
    stencilForceModel2D = new StencilForceModel2D(baseModel2D);
  }
  stencilForceModel = stencilForceModel2D;
  numVertices = baseModel2D->getNumVertices();
  meshGraph = new Graph(baseModel2D->getNumVertices(), baseModel2D->getNumEdges(), baseModel2D->getEdges());
  baseModel2D->GenerateMassMatrix(&massMatrix);

  // init variables
  u.resize(2*numVertices, 0.0);
  uvel.resize(2*numVertices, 0.0);
  uaccel.resize(2*numVertices, 0.0);
  f_ext.resize(2*numVertices, 0.0);
  f_extBase.resize(2*numVertices, 0.0);
  uInitial.resize(2*numVertices, 0.0);
  
  assert(stencilForceModel != nullptr);
  forceModelAssembler2D = new ForceModelAssembler2D(stencilForceModel); 
  forceModel = forceModelAssembler2D; 
  vertexMasses2D.resize(2*numVertices);
  massMatrix->SumRowEntries(vertexMasses2D.data());

  // load constraints
  if(fixedVerticesFilename != "_none")
  {
    if (ListIO::load(fixedVerticesFilename.data(), &numFixedVertices, &fixedVertices) != 0)
      {
        printf("Error reading fixed vertices.\n");
        exit(1);
      }
      sort(fixedVertices, fixedVertices+numFixedVertices);
  }
  else
  {
    numFixedVertices = 0;
    fixedVertices = nullptr;
  }
  printf("Loaded %d fixed vertices.\n",numFixedVertices);

  // create 0-indexed fixed DOFs
  int numFixedDOFs = 2 * numFixedVertices;
  int * fixedDOFs = (int*) malloc (sizeof(int) * numFixedDOFs);
  for(int i=0; i<numFixedVertices; i++)
  {
    fixedDOFs[2*i+0] = 2*fixedVertices[i]-2;
    fixedDOFs[2*i+1] = 2*fixedVertices[i]-1;
  }
  for(int i=0; i<numFixedVertices; i++)
    fixedVertices[i]--;
  printf("Boundary vertices processed.\n");


  if (solver == IMPLICITNEWMARK)
  {
    printf("Integrator: implicit newmark\n");
    implicitNewmarkSparse = new ImplicitNewmarkSparse(2*numVertices, timeStep, massMatrix, forceModel, numFixedDOFs, fixedDOFs,
       dampingMassCoef, dampingStiffnessCoef, maxIterations, epsilon, newmarkBeta, newmarkGamma, numSolverThreads);
    integratorBaseSparse = implicitNewmarkSparse;
  }
  else if (solver == IMPLICITBACKWARDEULER)
  {
    printf("Integrator: implicit backward euler\n");
    implicitNewmarkSparse = new ImplicitBackwardEulerSparse(2*numVertices, timeStep, massMatrix, forceModel, numFixedDOFs, fixedDOFs,
        dampingMassCoef, dampingStiffnessCoef, maxIterations, epsilon, numSolverThreads);
    integratorBaseSparse = implicitNewmarkSparse;
  }


  integratorBase = integratorBaseSparse;
  if (integratorBase == nullptr)
  {
    printf("Error: failed to initialize numerical integrator.\n");
    exit(1);
  }

  integratorBase->ResetToRest();
  integratorBase->SetState(uInitial.data(), velInitial.data());
  integratorBase->SetTimestep(timeStep / substepsPerTimeStep);

  if (implicitNewmarkSparse != nullptr)
  {
    implicitNewmarkSparse->UseStaticSolver(staticSolver);
    if (velInitial.size() == 2*numVertices)
      implicitNewmarkSparse->SetState(implicitNewmarkSparse->Getq(), velInitial.data());
  }
}

static void initImGUI()
{
// Setup ImGui binding
  ImGui::CreateContext();
  ImGuiIO& io = ImGui::GetIO(); (void)io;
  io.WantCaptureKeyboard = false;
  io.WantCaptureMouse = true;
  //io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;  // Enable Keyboard Controls

  ImGui_ImplGLUT_Init();
  // set additional glut callback functions
  glutKeyboardUpFunc(ImGui_ImplGLUT_KeyboardUpFunc);
  glutSpecialUpFunc(ImGui_ImplGLUT_SpecialUpFunc);

 // Setup style
  ImGui::StyleColorsDark();
}

static void keyboardFunction (unsigned char key, int x, int y)
{
  if (ImGui::GetIO().WantCaptureMouse)
  {
    ImGui_ImplGLUT_KeyboardFunc(key, x, y);
    if (ImGui::GetIO().WantCaptureKeyboard)
      return;
  }

  switch (key)
  {     
    case 27:
      exit(0);
    break;

    case 9:
    {
      static bool fullscreen = true;
      if (fullscreen)
      {
        cout << "Full screen" << endl;
        glutFullScreen();
      }
      else
      {
        cout << "Normal screen" << endl;
        glutReshapeWindow(800, 600);
        glutPositionWindow(50, 50);
      }
      fullscreen = !fullscreen;
      break;
    }

    case 'w':
      enableWireRender = !enableWireRender;
      break;

    case 'r':
      enableMaterialRender = !enableMaterialRender;
      break;

    case '0':
      reset_callback();
      break;
    
    case 'p':
      pauseSimulation = !pauseSimulation;
      break;
    
    case 'g':
      addGravity = !addGravity;
      break;

    default:
    break;
  }
}

void keyboardUpFunction(unsigned char key, int x, int y)
{
  // cout << "key " << key << " up" << endl;
  switch(key)
  {
    case 'h':
      key_h_pressed = false;
      break;

    default:
      break;
  }
}

Vec2d screenPos2WorldPos(int x, int y)
{
  double sx, sy;
  double ratio = (double)windowWidth/(double)windowHeight;
  if(windowWidth>windowHeight)
  {
    sx = (2.0 * x / windowWidth - 1.)* ratio;
    sy = 1. - 2.0 * y / windowHeight; 
  }
  else
  {
    sx = 2.0 * x/ windowWidth - 1.;
    sy = (1. - 2.0 * y / windowHeight)* 1.0 / ratio;
  }

  double cameraPos[2] = {sx,sy}; // clicked position in camera coord.
  double wp[2] = {0,0};
  camera->cameraVector2WorldVector(cameraPos, wp);

  return Vec2d(wp[0], wp[1]);
}

static void mouseMotionFunction(int x, int y)
{
  ImGui_ImplGLUT_MotionFunc(x,y);

  int mouseDeltaX = x - g_vMousePos[0];
  int mouseDeltaY = y - g_vMousePos[1];

  g_vMousePos[0] = x;
  g_vMousePos[1] = y;

  if (leftMouseButton) 
  {
    if (activeNode >= 0)
    {
      Vec2d pos = screenPos2WorldPos(x,y);
    }
  }

  if (rightMouseButton) 
  {
  }

  if ((middleMouseButton) || (leftMouseButton && altPressed)) {// handle zoom in/out
    double factor = 0.01;
    if(shiftPressed) factor *= 0.1;
    double scale = camera->getScale() + factor * mouseDeltaY;
    if(scale < 0) scale = 0;
    camera->setScale(scale);
  }
}

static void mouseButtonActivityFunction(int button, int state, int x, int y)
{
  ImGui_ImplGLUT_MouseFunc(button, state, x, y);

  if (ImGui::GetIO().WantCaptureMouse) // mouse is clicked on imGui
  {
    pulledVertex = -1;
    return;
  }

  shiftPressed = (glutGetModifiers() == GLUT_ACTIVE_SHIFT);
  controlPressed = (glutGetModifiers() == GLUT_ACTIVE_CTRL);
  altPressed = (glutGetModifiers() == GLUT_ACTIVE_ALT);

  switch (button)
  {
    case GLUT_LEFT_BUTTON:
      leftMouseButton = (state==GLUT_DOWN);
      // find closest control point
      if (leftMouseButton && (controlPressed == false && key_h_pressed == false) && shiftPressed == false)
      {
        dragStartX = x;
        dragStartY = y;
        Vec2d pos = screenPos2WorldPos(x,y);
        pulledVertex = triangleMeshRendering2D->GetClosestVertex(pos);
        printf("Clicked on vertex: %d (0-indexed)\n", pulledVertex);
      }

      if (!leftMouseButton)
      {
        pulledVertex = -1;
      }
    break;

    case GLUT_MIDDLE_BUTTON:
      middleMouseButton = (state==GLUT_DOWN);
    break;

    case GLUT_RIGHT_BUTTON:
      rightMouseButton = (state==GLUT_DOWN);
    break;
  }
  
  g_vMousePos[0] = x;
  g_vMousePos[1] = y;
}


static void specialFunction(int key, int x, int y)
{

  if (ImGui::GetIO().WantCaptureMouse)
  {
    ImGui_ImplGLUT_SpecialFunc(key, x, y);
	  if (ImGui::GetIO().WantCaptureKeyboard)
	    return;
  }
  
  shiftPressed = (glutGetModifiers() == GLUT_ACTIVE_SHIFT);
  controlPressed = (glutGetModifiers() == GLUT_ACTIVE_CTRL);
  altPressed = (glutGetModifiers() == GLUT_ACTIVE_ALT);

  double factor = 0.1;
  if(shiftPressed) factor *= 0.1;
  switch (key) {
  case GLUT_KEY_LEFT:

    camera->moveRight(factor);
    break;

  case GLUT_KEY_RIGHT:
    camera->moveRight(-factor);
    break;

  case GLUT_KEY_DOWN:
    camera->moveUp(factor);
    break;

  case GLUT_KEY_UP:
    camera->moveUp(-factor);
    break;

  default:
    break;
  }
}

/******************************************************************************
 Initializes GLUT.
******************************************************************************/
static void initGLUT(int argc, char* argv[], char * windowTitle, int windowWidth, int windowHeight, int * windowID)
{
    // Initialize GLUT.
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_MULTISAMPLE);
    glutInitWindowSize(windowWidth, windowHeight);
    glutInitWindowPosition(50,50);
    *windowID = glutCreateWindow(windowTitle);

    // Setup GLUT callbacks.
    glutDisplayFunc(displayFunction);
    glutReshapeFunc(reshape);
    glutIdleFunc(idleFunction);
    glutMouseFunc(mouseButtonActivityFunction);
    glutKeyboardFunc(keyboardFunction);
    glutSpecialFunc(specialFunction);
    glutMotionFunc(mouseMotionFunction);
    // callback for mouse movement without any buttons pressed
    //glutPassiveMotionFunc(MouseNoDrag);
}

#define ADD_CONFIG(v) simulator2DConfig.addOptionOptional(#v, &v, v)
void initConfigurations()
{
  ADD_CONFIG(triangleMeshFilename);
  ADD_CONFIG(forceMaterialName);
  ADD_CONFIG(solverName);
  ADD_CONFIG(dampingMassCoef);
  ADD_CONFIG(dampingStiffnessCoef);
  ADD_CONFIG(deformableObjectCompliance);
  ADD_CONFIG(timeStep);
  ADD_CONFIG(addGravity);
  ADD_CONFIG(g);
  ADD_CONFIG(forceNeighborhoodSize);
  ADD_CONFIG(substepsPerTimeStep);
  ADD_CONFIG(fixedVerticesFilename);
  ADD_CONFIG(windowWidth);
  ADD_CONFIG(windowHeight);
  ADD_CONFIG(backgroundColor);
  ADD_CONFIG(focusPositionX);
  ADD_CONFIG(focusPositionY);
  ADD_CONFIG(cameraScale);
  ADD_CONFIG(materialLambda);
  ADD_CONFIG(materialMu);
  ADD_CONFIG(materialDensity);
  ADD_CONFIG(massSpringStiffness);
  ADD_CONFIG(internalForceScalingFactor);
  
  ifstream fin(simulator2DConfigFilename.c_str());
  if (fin) {
    printf("Parsing configuration file %s...\n", simulator2DConfigFilename.c_str());
    fin.close();
    int ret = simulator2DConfig.parseOptions(simulator2DConfigFilename.c_str());
    if (ret != 0) {
      cout << "Error parsing " << simulator2DConfigFilename << endl;
      exit(1);
    }
    simulator2DConfig.printOptions();
  }

  // set the solver based on config file input
  solver = UNKNOWN;
  cout<<solverName<<endl;
  if (solverName == "implicitNewmark")
    solver = IMPLICITNEWMARK;
  else if (solverName == "implicitBackwardEuler")
    solver = IMPLICITBACKWARDEULER;

  if (solver == UNKNOWN)
  {
    printf("Error: unknown implicit solver specified.\n");
    exit(1);
  }

  forceMaterial = UNSPECIFIED;
  if(forceMaterialName == "CLFEM")
    forceMaterial = COROTLINFEM;
  else if(forceMaterialName == "LFEM")
    forceMaterial = LINFEM;
  else if(forceMaterialName == "MS")
    forceMaterial = MASSSPRING;
  if (forceMaterial == UNSPECIFIED)
  {
    printf("Error: unknown material type specified.\n");
    exit(1);
  }
}

int main(int argc, char* argv[])
{
  int numFixedArgs = 2;
  if ( argc < numFixedArgs ) 
  {
    printf("Simulator2D.\n");
    printf("Use: %s <config file>\n", argv[0]);
    return 1;
  }

  printf("Starting application.\n");

  simulator2DConfigFilename = argv[1];
  printf("Loading scene configuration from %s.\n", simulator2DConfigFilename.c_str());
  initConfigurations();

  initGLUT(argc, argv, windowTitleBase , windowWidth, windowHeight, &windowID);

  initGraphics(windowWidth, windowHeight);

  initImGUI();

  initSimulation();
  
  reshape(windowWidth,windowHeight);
   
  atexit(exitCallback);

  glutMainLoop(); 

  return 0;
}


