﻿/*
  CSCI 420 Computer Graphics, USC
  Assignment 1: Height Fields with Shaders.
  C++ starter code

  Student username: phuongp
*/

#include "basicPipelineProgram.h"
#include "openGLMatrix.h"
#include "imageIO.h"
#include "openGLHeader.h"
#include "glutHeader.h"
#include <vector>
#include <iostream>
#include <cstring>

#if defined(WIN32) || defined(_WIN32)
#ifdef _DEBUG
#pragma comment(lib, "glew32d.lib")
#else
#pragma comment(lib, "glew32.lib")
#endif
#endif

#if defined(WIN32) || defined(_WIN32)
char shaderBasePath[1024] = SHADER_BASE_PATH;
#else
char shaderBasePath[1024] = "../openGLHelper-starterCode";
#endif

using namespace std;

int mousePos[2]; // x,y screen coordinates of the current mouse position

int leftMouseButton = 0; // 1 if pressed, 0 if not 
int middleMouseButton = 0; // 1 if pressed, 0 if not
int rightMouseButton = 0; // 1 if pressed, 0 if not

typedef enum { ROTATE, TRANSLATE, SCALE } CONTROL_STATE;
CONTROL_STATE controlState = ROTATE;

typedef enum { POINT_MODE, LINE_MODE, SOLID_MODE } RENDER_STATE;
RENDER_STATE renderState = POINT_MODE;

// Transformations of the terrain.
float terrainRotate[3] = { 0.0f, 0.0f, 0.0f };
// terrainRotate[0] gives the rotation around x-axis (in degrees)
// terrainRotate[1] gives the rotation around y-axis (in degrees)
// terrainRotate[2] gives the rotation around z-axis (in degrees)
float terrainTranslate[3] = { 0.0f, 0.0f, 0.0f };
float terrainScale[3] = { 1.0f, 1.0f, 1.0f };

// Width and height of the OpenGL window, in pixels.
int windowWidth = 1280;
int windowHeight = 720;
char windowTitle[512] = "CSCI 420 homework I";
int numBytesInPositions, numBytesInPositionsLine, numBytesInPositionsSolid;
int numBytesInColors, numBytesInColorsLine, numBytesInColorsSolid;

// Width and height of the image
int imageWidth;
int imageHeight;
float scale = 0.002f;

// Stores the image loaded from disk.
ImageIO* heightmapImage;

// VBO and VAO for point mode
std::vector<float> pointPos;
std::vector<float> pointCol;
GLuint pointVBO;
GLuint pointVAO;
int pointNumVertices;

// VBO and VAO for line mode
std::vector<float> linePos;
std::vector<float> lineCol;
GLuint lineVBO;
GLuint lineVAO;
int lineNumVertices;

// VBO and VAO for solid mode
std::vector<float> solidPos;
std::vector<float> solidCol;
GLuint solidVBO;
GLuint solidVAO;
int solidNumVertices;

// CSCI 420 helper classes.
OpenGLMatrix matrix;
BasicPipelineProgram* pipelineProgram;

// Write a screenshot to the specified filename.
void saveScreenshot(const char* filename)
{
	unsigned char* screenshotData = new unsigned char[windowWidth * windowHeight * 3];
	glReadPixels(0, 0, windowWidth, windowHeight, GL_RGB, GL_UNSIGNED_BYTE, screenshotData);

	ImageIO screenshotImg(windowWidth, windowHeight, 3, screenshotData);

	if (screenshotImg.save(filename, ImageIO::FORMAT_JPEG) == ImageIO::OK)
		cout << "File " << filename << " saved successfully." << endl;
	else cout << "Failed to save file " << filename << '.' << endl;

	delete[] screenshotData;
}

void renderPointMode()
{
	// Execute rendering
	int numVertex = pointPos.size() / 3;
	glBindVertexArray(pointVAO);
	glDrawArrays(GL_POINTS, 0, numVertex);
	glBindVertexArray(0); // unbind the VAO
}

void renderLineMode()
{
	// Execute rendering
	int numVertex = linePos.size() / 3;
	glBindVertexArray(lineVAO);
	glDrawArrays(GL_LINES, 0, numVertex);
	glBindVertexArray(0); // unbind the VAO
}

void renderSolidMode()
{
	// Execute rendering
	int numVertex = solidPos.size() / 3;
	glBindVertexArray(solidVAO);
	glDrawArrays(GL_TRIANGLES, 0, numVertex);
	glBindVertexArray(0); // unbind the VAO
}

void initVertices()
{}

void initVBOs()
{}

// initialize the scene
void initScene(int argc, char* argv[])
{
	// Load the image from a jpeg disk file into main memory.
	heightmapImage = new ImageIO();
	if (heightmapImage->loadJPEG(argv[1]) != ImageIO::OK)
	{
		cout << "Error reading image " << argv[1] << "." << endl;
		exit(EXIT_FAILURE);
	}

	// Set the background color.
	glClearColor(0.5f, 0.5f, 0.5f, 1.0f); // Black color.

	// Enable z-buffering (i.e., hidden surface removal using the z-buffer algorithm).
	glEnable(GL_DEPTH_TEST);

	// Create and bind the pipeline program. This operation must be performed BEFORE we initialize any VAOs.
	pipelineProgram = new BasicPipelineProgram;
	int ret = pipelineProgram->Init(shaderBasePath);
	if (ret != 0)
	{
		abort();
	}
	pipelineProgram->Bind();

	// Prepare the triangle position and color data for the VBO. 
	// The code below sets up a single triangle (3 vertices).
	// The triangle will be rendered using GL_TRIANGLES (in displayFunc()).
	imageWidth = heightmapImage->getWidth();
	imageHeight = heightmapImage->getHeight();
	pointNumVertices = imageWidth * imageHeight; // This must be a global variable, so that we know how many vertices to render in glDrawArrays.

	for (int x = 0; x < imageWidth; x++)
	{
		for (int y = 0; y < imageHeight; y++)
		{
			int numVertices = 3;
			int numColors = 4;
			//int triIdx = (x + y * imageWidth) * 3;
			//int colorIdx = (x + y * imageWidth) * 4;

			// center point
			float centerPos[3];
			float centerCol[4];

			centerPos[0] = static_cast<float>(y) / imageHeight;
			centerPos[1] = scale * heightmapImage->getPixel(x, y, 0);
			centerPos[2] = static_cast<float>(x) / imageWidth;

			centerCol[0] = static_cast<float>(y) / imageHeight - 0.5f;
			centerCol[1] = scale * heightmapImage->getPixel(x, y, 0);
			centerCol[2] = static_cast<float>(x) / imageWidth - 0.5f;
			centerCol[3] = 1.0f;

			// above point: (i, j + 1)
			float abovePos[3];
			float aboveCol[4];

			abovePos[0] = static_cast<float>(y + 1) / imageHeight;
			abovePos[1] = scale * heightmapImage->getPixel(x, y + 1, 0);
			abovePos[2] = centerPos[2];

			aboveCol[0] = static_cast<float>(y + 1) / imageHeight - 0.5f;
			aboveCol[1] = scale * heightmapImage->getPixel(x, y + 1, 0);
			aboveCol[2] = static_cast<float>(x) / imageWidth - 0.5f;
			aboveCol[3] = 1.0f;

			// right: (i + 1, j)
			float rightPos[3];
			float rightCol[4];

			rightPos[0] = centerPos[0];
			rightPos[1] = scale * heightmapImage->getPixel(x + 1, y, 0);
			rightPos[2] = static_cast<float>(x + 1) / imageWidth;

			rightCol[0] = static_cast<float>(y) / imageHeight - 0.5f;
			rightCol[1] = scale * heightmapImage->getPixel(x + 1, y, 0);
			rightCol[2] = static_cast<float>(x + 1) / imageWidth - 0.5f;
			rightCol[3] = 1.0f;

			// right and above: (i + 1, j + 1)
			float aboveRightPos[3];
			float aboveRightCol[4];

			aboveRightPos[0] = abovePos[0];
			aboveRightPos[1] = scale * heightmapImage->getPixel(x + 1, y + 1, 0);
			aboveRightPos[2] = rightPos[2];

			aboveRightCol[0] = static_cast<float>(y + 1) / imageHeight - 0.5f;
			aboveRightCol[1] = scale * heightmapImage->getPixel(x + 1, y + 1, 0);
			aboveRightCol[2] = static_cast<float>(x + 1) / imageWidth - 0.5f;
			aboveRightCol[3] = 1.0f;

			// POINT MODE:
			pointPos.insert(pointPos.end(), centerPos, centerPos + numVertices);
			pointCol.insert(pointCol.end(), centerCol, centerCol + numColors);

			// LINE MODE:
			linePos.insert(linePos.end(), centerPos, centerPos + numVertices);
			if (y < imageHeight)
				linePos.insert(linePos.end(), abovePos, abovePos + numVertices);
			linePos.insert(linePos.end(), centerPos, centerPos + numVertices);
			if (x < imageWidth)
				linePos.insert(linePos.end(), rightPos, rightPos + numVertices);

			lineCol.insert(lineCol.end(), centerCol, centerCol + numColors);
			if (y < imageHeight)
				lineCol.insert(lineCol.end(), aboveCol, aboveCol + numColors);
			lineCol.insert(lineCol.end(), centerCol, centerCol + numColors);
			if (x < imageWidth)
				lineCol.insert(lineCol.end(), rightCol, rightCol + numColors);

			// SOLID MODE:
			// first triangle
			solidPos.insert(solidPos.end(), centerPos, centerPos + numVertices);
			solidPos.insert(solidPos.end(), abovePos, abovePos + numVertices);
			solidPos.insert(solidPos.end(), rightPos, rightPos + numVertices);
			// second triangle
			solidPos.insert(solidPos.end(), aboveRightPos, aboveRightPos + numVertices);
			solidPos.insert(solidPos.end(), abovePos, abovePos + numVertices);
			solidPos.insert(solidPos.end(), rightPos, rightPos + numVertices);
			// color
			solidCol.insert(solidCol.end(), centerCol, centerCol + 4);
			solidCol.insert(solidCol.end(), aboveCol, aboveCol + 4);
			solidCol.insert(solidCol.end(), rightCol, rightCol + 4);
			solidCol.insert(solidCol.end(), aboveRightCol, aboveRightCol + 4);
			solidCol.insert(solidCol.end(), aboveCol, aboveCol + 4);
			solidCol.insert(solidCol.end(), rightCol, rightCol + 4);
		}
	}

	// VBOs for each mode
	const int stride = 0; // Stride is 0, i.e., data is tightly packed in the VBO.
	const GLboolean normalized = GL_FALSE; // Normalization is off.
	// POINT_MODE
	// Create the VBOs. This operation must be performed BEFORE we initialize any VAOs.
	glGenBuffers(1, &pointVBO);
	glBindBuffer(GL_ARRAY_BUFFER, pointVBO);
	// First, allocate an empty VBO of the correct size to hold positions and colors.
	numBytesInPositions = pointPos.size() * sizeof(float);
	numBytesInColors = pointCol.size() * sizeof(float);
	glBufferData(GL_ARRAY_BUFFER, numBytesInPositions + numBytesInColors, nullptr, GL_STATIC_DRAW);
	// Next, write the position and color data into the VBO.
	glBufferSubData(GL_ARRAY_BUFFER, 0, numBytesInPositions, pointPos.data()); // The VBO starts with positions.
	glBufferSubData(GL_ARRAY_BUFFER, numBytesInPositions, numBytesInColors, pointCol.data()); // The colors are written after the positions.
	// Create the VAOs. There is a single VAO in this example.
	glGenVertexArrays(1, &pointVAO);
	glBindVertexArray(pointVAO);
	// Set up the relationship between the "position" shader variable and the VAO.
	const GLuint locationOfPosition = glGetAttribLocation(pipelineProgram->GetProgramHandle(), "position"); // Obtain a handle to the shader variable "position".
	glEnableVertexAttribArray(locationOfPosition); // Must always enable the vertex attribute. By default, it is disabled.
	glVertexAttribPointer(locationOfPosition, 3, GL_FLOAT, normalized, stride, (const void*)0); // The shader variable "position" receives its data from the currently bound VBO (i.e., vertexPositionAndColorVBO), starting from offset 0 in the VBO. There are 3 float entries per vertex in the VBO (i.e., x,y,z coordinates). 
	// Set up the relationship between the "color" shader variable and the VAO.
	const GLuint locationOfColor = glGetAttribLocation(pipelineProgram->GetProgramHandle(), "color"); // Obtain a handle to the shader variable "color".
	glEnableVertexAttribArray(locationOfColor); // Must always enable the vertex attribute. By default, it is disabled.
	glVertexAttribPointer(locationOfColor, 4, GL_FLOAT, normalized, stride, (const void*)(unsigned long)numBytesInPositions); // The shader variable "color" receives its data from the currently bound VBO (i.e., vertexPositionAndColorVBO), starting from offset "numBytesInPositions" in the VBO. There are 4 float entries per vertex in the VBO (i.e., r,g,b,a channels). 
	glBindVertexArray(0); // unbind the VAO

	// LINE MODE
	glGenBuffers(1, &lineVBO);
	glBindBuffer(GL_ARRAY_BUFFER, lineVBO);
	// First, allocate an empty VBO of the correct size to hold positions and colors.
	numBytesInPositionsLine = linePos.size() * sizeof(float);
	numBytesInColorsLine = lineCol.size() * sizeof(float);
	glBufferData(GL_ARRAY_BUFFER, numBytesInPositionsLine + numBytesInColorsLine, nullptr, GL_STATIC_DRAW);
	// Next, write the position and color data into the VBO.
	glBufferSubData(GL_ARRAY_BUFFER, 0, numBytesInPositionsLine, linePos.data()); // The VBO starts with positions.
	glBufferSubData(GL_ARRAY_BUFFER, numBytesInPositionsLine, numBytesInColorsLine, lineCol.data()); // The colors are written after the positions.
	// Create the VAOs. There is a single VAO in this example.
	glGenVertexArrays(1, &lineVAO);
	glBindVertexArray(lineVAO);
	// Set up the relationship between the "position" shader variable and the VAO.
	const GLuint locationOfPositionLine = glGetAttribLocation(pipelineProgram->GetProgramHandle(), "position"); // Obtain a handle to the shader variable "position".
	glEnableVertexAttribArray(locationOfPositionLine); // Must always enable the vertex attribute. By default, it is disabled.
	glVertexAttribPointer(locationOfPositionLine, 3, GL_FLOAT, normalized, stride, (const void*)0); // The shader variable "position" receives its data from the currently bound VBO (i.e., vertexPositionAndColorVBO), starting from offset 0 in the VBO. There are 3 float entries per vertex in the VBO (i.e., x,y,z coordinates). 
	// Set up the relationship between the "color" shader variable and the VAO.
	const GLuint locationOfColorLine = glGetAttribLocation(pipelineProgram->GetProgramHandle(), "color"); // Obtain a handle to the shader variable "color".
	glEnableVertexAttribArray(locationOfColorLine); // Must always enable the vertex attribute. By default, it is disabled.
	glVertexAttribPointer(locationOfColorLine, 4, GL_FLOAT, normalized, stride, (const void*)(unsigned long)numBytesInPositionsLine); // The shader variable "color" receives its data from the currently bound VBO (i.e., vertexPositionAndColorVBO), starting from offset "numBytesInPositions" in the VBO. There are 4 float entries per vertex in the VBO (i.e., r,g,b,a channels). 
	glBindVertexArray(0); // unbind the VAO

	// SOLID MODE
	glGenBuffers(1, &solidVBO);
	glBindBuffer(GL_ARRAY_BUFFER, solidVBO);
	numBytesInPositionsSolid = solidPos.size() * sizeof(float);
	numBytesInColorsSolid = solidCol.size() * sizeof(float);
	glBufferData(GL_ARRAY_BUFFER, numBytesInPositionsSolid + numBytesInColorsSolid, nullptr, GL_STATIC_DRAW);
	// subdata
	glBufferSubData(GL_ARRAY_BUFFER, 0, numBytesInPositionsSolid, solidPos.data()); // The VBO starts with positions.
	glBufferSubData(GL_ARRAY_BUFFER, numBytesInPositionsSolid, numBytesInColorsSolid, solidCol.data()); // The colors are written after the positions.
	// Create the VAOs. There is a single VAO in this example.
	glGenVertexArrays(1, &solidVAO);
	glBindVertexArray(solidVAO);
	// Set up the relationship between the "position" shader variable and the VAO.
	const GLuint locationOfPositionSolid = glGetAttribLocation(pipelineProgram->GetProgramHandle(), "position"); // Obtain a handle to the shader variable "position".
	glEnableVertexAttribArray(locationOfPositionSolid); // Must always enable the vertex attribute. By default, it is disabled.
	glVertexAttribPointer(locationOfPositionSolid, 3, GL_FLOAT, normalized, stride, (const void*)0); // The shader variable "position" receives its data from the currently bound VBO (i.e., vertexPositionAndColorVBO), starting from offset 0 in the VBO. There are 3 float entries per vertex in the VBO (i.e., x,y,z coordinates). 
	// Set up the relationship between the "color" shader variable and the VAO.
	const GLuint locationOfColorSolid = glGetAttribLocation(pipelineProgram->GetProgramHandle(), "color"); // Obtain a handle to the shader variable "color".
	glEnableVertexAttribArray(locationOfColorSolid); // Must always enable the vertex attribute. By default, it is disabled.
	glVertexAttribPointer(locationOfColorSolid, 4, GL_FLOAT, normalized, stride, (const void*)(unsigned long)numBytesInPositionsSolid); // The shader variable "color" receives its data from the currently bound VBO (i.e., vertexPositionAndColorVBO), starting from offset "numBytesInPositions" in the VBO. There are 4 float entries per vertex in the VBO (i.e., r,g,b,a channels). 
	glBindVertexArray(0); // unbind the VAO

	// Check for any OpenGL errors.
	std::cout << "GL error: " << glGetError() << std::endl;
}

void displayFunc()
{
	// This function performs the actual rendering.
	// First, clear the screen.
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	// Set up the camera position, focus point, and the up vector.
	matrix.SetMatrixMode(OpenGLMatrix::ModelView);
	matrix.LoadIdentity();
	matrix.LookAt(-1.0, 1.5, 1.5,
		0.0, 0.0, 0.0,
		0.0, 1.0, 0.0);

	// In here, you can do additional modeling on the object, such as performing translations, rotations and scales.
	// ...

	// Read the current modelview and projection matrices.
	float modelViewMatrix[16];
	matrix.SetMatrixMode(OpenGLMatrix::ModelView);
	matrix.GetMatrix(modelViewMatrix);

	float projectionMatrix[16];
	matrix.SetMatrixMode(OpenGLMatrix::Projection);
	matrix.GetMatrix(projectionMatrix);

	pipelineProgram->Bind(); // This call is redundant in hw1, but it is good to keep for consistency.

	// Upload the modelview and projection matrices to the GPU.
	pipelineProgram->SetModelViewMatrix(modelViewMatrix);
	pipelineProgram->SetProjectionMatrix(projectionMatrix);

	switch (renderState)
	{
	case POINT_MODE:
		renderPointMode();
		break;
	case LINE_MODE:
		renderLineMode();
		break;
	case SOLID_MODE:
		renderSolidMode();
		break;
	}
	// Swap the double-buffers.
	glutSwapBuffers();
}

void idleFunc()
{
	// Do some stuff... 

	// For example, here, you can save the screenshots to disk (to make the animation).

	// Send the signal that we should call displayFunc.
	glutPostRedisplay();
}

void reshapeFunc(int w, int h)
{
	glViewport(0, 0, w, h);

	// When the window has been resized, we need to re-set our projection matrix.
	matrix.SetMatrixMode(OpenGLMatrix::Projection);
	matrix.LoadIdentity();
	// You need to be careful about setting the zNear and zFar. 
	// Anything closer than zNear, or further than zFar, will be culled.
	const float zNear = 0.1f;
	const float zFar = 10000.0f;
	const float humanFieldOfView = 60.0f;
	matrix.Perspective(humanFieldOfView, 1.0f * w / h, zNear, zFar);
}

void mouseMotionDragFunc(int x, int y)
{
	// Mouse has moved, and one of the mouse buttons is pressed (dragging).

	// the change in mouse position since the last invocation of this function
	int mousePosDelta[2] = { x - mousePos[0], y - mousePos[1] };

	switch (controlState)
	{
		// translate the terrain
	case TRANSLATE:
		if (leftMouseButton)
		{
			// control x,y translation via the left mouse button
			terrainTranslate[0] += mousePosDelta[0] * 0.01f;
			terrainTranslate[1] -= mousePosDelta[1] * 0.01f;
		}
		if (middleMouseButton)
		{
			// control z translation via the middle mouse button
			terrainTranslate[2] += mousePosDelta[1] * 0.01f;
		}
		break;

		// rotate the terrain
	case ROTATE:
		if (leftMouseButton)
		{
			// control x,y rotation via the left mouse button
			terrainRotate[0] += mousePosDelta[1];
			terrainRotate[1] += mousePosDelta[0];
		}
		if (middleMouseButton)
		{
			// control z rotation via the middle mouse button
			terrainRotate[2] += mousePosDelta[1];
		}
		break;

		// scale the terrain
	case SCALE:
		if (leftMouseButton)
		{
			// control x,y scaling via the left mouse button
			terrainScale[0] *= 1.0f + mousePosDelta[0] * 0.01f;
			terrainScale[1] *= 1.0f - mousePosDelta[1] * 0.01f;
		}
		if (middleMouseButton)
		{
			// control z scaling via the middle mouse button
			terrainScale[2] *= 1.0f - mousePosDelta[1] * 0.01f;
		}
		break;
	}

	// store the new mouse position
	mousePos[0] = x;
	mousePos[1] = y;
}

void mouseMotionFunc(int x, int y)
{
	// Mouse has moved.
	// Store the new mouse position.
	mousePos[0] = x;
	mousePos[1] = y;
}

void mouseButtonFunc(int button, int state, int x, int y)
{
	// A mouse button has has been pressed or depressed.

	// Keep track of the mouse button state, in leftMouseButton, middleMouseButton, rightMouseButton variables.
	switch (button)
	{
	case GLUT_LEFT_BUTTON:
		leftMouseButton = (state == GLUT_DOWN);
		break;

	case GLUT_MIDDLE_BUTTON:
		middleMouseButton = (state == GLUT_DOWN);
		break;

	case GLUT_RIGHT_BUTTON:
		rightMouseButton = (state == GLUT_DOWN);
		break;
	}

	// Keep track of whether CTRL and SHIFT keys are pressed.
	switch (glutGetModifiers())
	{
	case GLUT_ACTIVE_CTRL:
		controlState = TRANSLATE;
		break;

	case GLUT_ACTIVE_SHIFT:
		controlState = SCALE;
		break;

		// If CTRL and SHIFT are not pressed, we are in rotate mode.
	default:
		controlState = ROTATE;
		break;
	}

	// Store the new mouse position.
	mousePos[0] = x;
	mousePos[1] = y;
}

void keyboardFunc(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 27: // ESC key
		exit(0); // exit the program
		break;

	case ' ':
		cout << "You pressed the spacebar." << endl;
		break;

	case 'x':
		// Take a screenshot.
		saveScreenshot("screenshot.jpg");
		break;
	case '1':
		renderState = POINT_MODE;
		break;
	case '2':
		renderState = LINE_MODE;
		break;
	case '3':
		renderState = SOLID_MODE;
		break;
	}
}


int main(int argc, char* argv[])
{
	if (argc != 2)
	{
		cout << "The arguments are incorrect." << endl;
		cout << "usage: ./hw1 <heightmap file>" << endl;
		exit(EXIT_FAILURE);
	}

	cout << "Initializing GLUT..." << endl;
	glutInit(&argc, argv);

	cout << "Initializing OpenGL..." << endl;

#ifdef __APPLE__
	glutInitDisplayMode(GLUT_3_2_CORE_PROFILE | GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH | GLUT_STENCIL);
#else
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH | GLUT_STENCIL);
#endif

	glutInitWindowSize(windowWidth, windowHeight);
	glutInitWindowPosition(0, 0);
	glutCreateWindow(windowTitle);

	cout << "OpenGL Version: " << glGetString(GL_VERSION) << endl;
	cout << "OpenGL Renderer: " << glGetString(GL_RENDERER) << endl;
	cout << "Shading Language Version: " << glGetString(GL_SHADING_LANGUAGE_VERSION) << endl;

#ifdef __APPLE__
	// This is needed on recent Mac OS X versions to correctly display the window.
	glutReshapeWindow(windowWidth - 1, windowHeight - 1);
#endif

	// tells glut to use a particular display function to redraw 
	glutDisplayFunc(displayFunc);
	// perform animation inside idleFunc
	glutIdleFunc(idleFunc);
	// callback for mouse drags
	glutMotionFunc(mouseMotionDragFunc);
	// callback for idle mouse movement
	glutPassiveMotionFunc(mouseMotionFunc);
	// callback for mouse button changes
	glutMouseFunc(mouseButtonFunc);
	// callback for resizing the window
	glutReshapeFunc(reshapeFunc);
	// callback for pressing the keys on the keyboard
	glutKeyboardFunc(keyboardFunc);

	// init glew
#ifdef __APPLE__
  // nothing is needed on Apple
#else
  // Windows, Linux
	GLint result = glewInit();
	if (result != GLEW_OK)
	{
		cout << "error: " << glewGetErrorString(result) << endl;
		exit(EXIT_FAILURE);
	}
#endif

	// Perform the initialization.
	initScene(argc, argv);

	// Sink forever into the GLUT loop.
	glutMainLoop();
}