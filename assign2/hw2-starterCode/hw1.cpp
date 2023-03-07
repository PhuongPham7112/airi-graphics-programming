/*
  CSCI 420 Computer Graphics, USC
  Assignment 2: Roller Coaster
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <glm/glm.hpp>

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

typedef enum { POINT_MODE, LINE_MODE, SOLID_MODE, SMOOTH_MODE } RENDER_STATE;
RENDER_STATE renderState = POINT_MODE;

// Transformations of the terrain.
float terrainRotate[3] = { 0.0f, 0.0f, 0.0f };
// terrainRotate[0] gives the rotation around x-axis (in degrees)
// terrainRotate[1] gives the rotation around y-axis (in degrees)
// terrainRotate[2] gives the rotation around z-axis (in degrees)
float terrainTranslate[3] = { 0.0f, 0.0f, 0.0f };
float terrainScale[3] = { 1.0f, 1.0f, 1.0f };

// Width and height of the OpenGL window, in pixels.
GLint shaderMode = 0;
GLint maxHeightShader = 0;
int windowWidth = 1280;
int windowHeight = 720;
char windowTitle[512] = "CSCI 420 homework II";

// Width and height of the image
int imageWidth;
int imageHeight;
float scale = 0.002f;
float maxHeight = 0.0f;
// Stores the image loaded from disk.
ImageIO* heightmapImage;

// VBO and VAO for 
std::vector<float> curvePos;
std::vector<float> curveCol;
GLuint curveVBO;
GLuint curveVAO;

// CSCI 420 helper classes.
int frameNum = 0;
OpenGLMatrix matrix;
BasicPipelineProgram* pipelineProgram;

// represents one control point along the spline 
struct Point
{
	float x;
	float y;
	float z;
};

// spline struct 
// contains how many control points the spline has, and an array of control points 
struct Spline
{
	int numControlPoints;
	Point* points;
};

// the spline array 
Spline* splines;
// total number of splines 
int numSplines;

void initSplines(Spline spl)
{
	float uStep = 0.001f;
	float s = 0.5f;

	glm::mat4 basisMat = glm::mat4(
		-s, 2.0f * s, -s, 0.0f,
		2.0f - s, s - 3.0f, 0.0f, 1.0f,
		s - 2.0f, 3.0f - 2.0f * s, s, 0.0f,
		s, -s, 0.0f, 0.0f);
	for (int i = 1; i < spl.numControlPoints - 2; i++)
	{
		float u = 0.0f;
		glm::mat3x4 controlMat = glm::mat3x4(
			spl.points[i - 1].x, spl.points[i].x, spl.points[i + 1].x, spl.points[i + 2].x,
			spl.points[i - 1].y, spl.points[i].y, spl.points[i + 1].y, spl.points[i + 2].y,
			spl.points[i - 1].z, spl.points[i].z, spl.points[i + 1].z, spl.points[i + 2].z);
		while (u < 1.0f)
		{
			glm::vec4 params1 = glm::vec4(pow(u, 3), pow(u, 2), u, 1);
			glm::vec3 p1 = params1 * basisMat * controlMat;
			curvePos.push_back(p1.x);
			curvePos.push_back(p1.y);
			curvePos.push_back(p1.z);

			curveCol.push_back(1.0f);
			curveCol.push_back(1.0f);
			curveCol.push_back(1.0f);
			curveCol.push_back(1.0f);


			glm::vec4 params2 = glm::vec4(pow(u + uStep, 3), pow(u + uStep, 2), u + uStep, 1);
			glm::vec3 p2 = params2 * basisMat * controlMat;
			curvePos.push_back(p2.x);
			curvePos.push_back(p2.y);
			curvePos.push_back(p2.z);

			curveCol.push_back(1.0f);
			curveCol.push_back(1.0f);
			curveCol.push_back(1.0f);
			curveCol.push_back(1.0f);

			u += uStep;
		}
	}
}

int loadSplines(char* argv)
{
	cout << argv << endl;
	char* cName = (char*)malloc(128 * sizeof(char));
	FILE* fileList;
	FILE* fileSpline;
	int iType, i = 0, j, iLength;

	// load the track file 
	fileList = fopen(argv, "r");
	if (fileList == NULL)
	{
		printf("can't open file\n");
		exit(1);
	}

	// stores the number of splines in a global variable 
	fscanf(fileList, "%d", &numSplines);

	splines = (Spline*)malloc(numSplines * sizeof(Spline));

	// reads through the spline files 
	for (j = 0; j < numSplines; j++)
	{
		i = 0;
		fscanf(fileList, "%s", cName);
		fileSpline = fopen(cName, "r");

		if (fileSpline == NULL)
		{
			printf("can't open file\n");
			exit(1);
		}

		// gets length for spline file
		fscanf(fileSpline, "%d %d", &iLength, &iType);

		// allocate memory for all the points
		splines[j].points = (Point*)malloc(iLength * sizeof(Point));
		splines[j].numControlPoints = iLength;

		// saves the data to the struct
		while (fscanf(fileSpline, "%f %f %f",
			&splines[j].points[i].x,
			&splines[j].points[i].y,
			&splines[j].points[i].z) != EOF)
		{
			i++;
		}
	}

	free(cName);

	return 0;
}

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

void initVBOsMode(GLuint& vbo, GLuint& vao, std::vector<float>& pos, std::vector<float>& col)
{
	const int stride = 0; // Stride is 0, i.e., data is tightly packed in the VBO.
	const GLboolean normalized = GL_FALSE; // Normalization is off.

	glGenBuffers(1, &vbo);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	// First, allocate an empty VBO of the correct size to hold positions and colors.
	const int numBytesInPositions = pos.size() * sizeof(float);
	const int numBytesInColors = col.size() * sizeof(float);
	glBufferData(GL_ARRAY_BUFFER, numBytesInPositions + numBytesInColors, nullptr, GL_STATIC_DRAW);
	// Next, write the position and color data into the VBO.
	glBufferSubData(GL_ARRAY_BUFFER, 0, numBytesInPositions, pos.data()); // The VBO starts with positions.
	glBufferSubData(GL_ARRAY_BUFFER, numBytesInPositions, numBytesInColors, col.data()); // The colors are written after the positions.
	// Create the VAOs. There is a single VAO in this example.
	glGenVertexArrays(1, &vao);
	glBindVertexArray(vao);
	// Set up the relationship between the "position" shader variable and the VAO.
	shaderMode = glGetUniformLocation(pipelineProgram->GetProgramHandle(), "shaderMode");
	maxHeightShader = glGetUniformLocation(pipelineProgram->GetProgramHandle(), "maxHeight");
	const GLuint locationOfPosition = glGetAttribLocation(pipelineProgram->GetProgramHandle(), "position"); // Obtain a handle to the shader variable "position".
	glEnableVertexAttribArray(locationOfPosition); // Must always enable the vertex attribute. By default, it is disabled.
	glVertexAttribPointer(locationOfPosition, 3, GL_FLOAT, normalized, stride, (const void*)0); // The shader variable "position" receives its data from the currently bound VBO (i.e., vertexPositionAndColorVBO), starting from offset 0 in the VBO. There are 3 float entries per vertex in the VBO (i.e., x,y,z coordinates). 
	// Set up the relationship between the "color" shader variable and the VAO.
	const GLuint locationOfColor = glGetAttribLocation(pipelineProgram->GetProgramHandle(), "color"); // Obtain a handle to the shader variable "color".
	glEnableVertexAttribArray(locationOfColor); // Must always enable the vertex attribute. By default, it is disabled.
	glVertexAttribPointer(locationOfColor, 4, GL_FLOAT, normalized, stride, (const void*)(unsigned long)numBytesInPositions); // The shader variable "color" receives its data from the currently bound VBO (i.e., vertexPositionAndColorVBO), starting from offset "numBytesInPositions" in the VBO. There are 4 float entries per vertex in the VBO (i.e., r,g,b,a channels). 
	glBindVertexArray(0); // unbind the VAO
}

// initialize the scene
void initScene(int argc, char* argv[])
{

	// Set the background color.
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f); // Black color.

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

	// init the splines
	for (int i = 0; i < numSplines; i++)
	{
		initSplines(splines[i]);
	}

	initVBOsMode(curveVBO, curveVAO, curvePos, curveCol);

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
	matrix.LookAt(-0.5, 1.0, 1.5,
		0.0, 0.0, 0.0,
		0.0, 1.0, 0.0);

	// In here, you can do additional modeling on the object, such as performing translations, rotations and scales.
	// ...
	matrix.Rotate(terrainRotate[0], 1.0f, 0.0f, 0.0f);
	matrix.Rotate(terrainRotate[1], 0.0f, 1.0f, 0.0f);
	matrix.Rotate(terrainRotate[2], 0.0f, 0.0f, 1.0f);

	matrix.Translate(terrainTranslate[0], terrainTranslate[1], terrainTranslate[2]);
	matrix.Scale(terrainScale[0], terrainScale[1], terrainScale[2]);

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

	glBindVertexArray(curveVAO);
	glDrawArrays(GL_LINES, 0, curvePos.size() / 3);
	glBindVertexArray(0); // unbind the VAO

	// Swap the double-buffers.
	glutSwapBuffers();
}

void animate()
{
	std::string folder = "../screenshots/";
	if (frameNum < 50)
	{
		renderState = POINT_MODE;
		glUniform1i(shaderMode, (GLint)0);
		terrainRotate[1] += 0.3f;
		saveScreenshot((folder + "frame_" + std::to_string(frameNum) + ".jpeg").c_str());
		frameNum++;
	}
	else if (frameNum < 100)
	{
		renderState = LINE_MODE;
		glUniform1i(shaderMode, (GLint)0);
		terrainRotate[1] += 0.3f;

		saveScreenshot((folder + "frame_" + std::to_string(frameNum) + ".jpeg").c_str());
		frameNum++;
	}
	else if (frameNum < 150)
	{
		renderState = SOLID_MODE;
		glUniform1i(shaderMode, (GLint)0);
		terrainRotate[1] += 0.3f;
		saveScreenshot((folder + "frame_" + std::to_string(frameNum) + ".jpeg").c_str());
		frameNum++;
	}
	else if (frameNum < 200)
	{
		renderState = SMOOTH_MODE;
		glUniform1i(shaderMode, (GLint)1);
		terrainRotate[1] += 0.3f;
		saveScreenshot((folder + "frame_" + std::to_string(frameNum) + ".jpeg").c_str());
		frameNum++;
	}
}

void idleFunc()
{
	// Do some stuff... 
	// For example, here, you can save the screenshots to disk (to make the animation).
	// animate();
	saveScreenshot("star.jpeg");

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
		glUniform1i(shaderMode, (GLint)0);
		renderState = POINT_MODE;
		break;
	case '2':
		glUniform1i(shaderMode, (GLint)0);
		renderState = LINE_MODE;
		break;
	case '3':
		glUniform1i(shaderMode, (GLint)0);
		renderState = SOLID_MODE;
		break;
	case '4':
		glUniform1i(shaderMode, (GLint)1);
		renderState = SMOOTH_MODE;
		break;
	}
}

int initTexture(const char* imageFilename, GLuint textureHandle)
{
	// read the texture image
	ImageIO img;
	ImageIO::fileFormatType imgFormat;
	ImageIO::errorType err = img.load(imageFilename, &imgFormat);

	if (err != ImageIO::OK)
	{
		printf("Loading texture from %s failed.\n", imageFilename);
		return -1;
	}

	// check that the number of bytes is a multiple of 4
	if (img.getWidth() * img.getBytesPerPixel() % 4)
	{
		printf("Error (%s): The width*numChannels in the loaded image must be a multiple of 4.\n", imageFilename);
		return -1;
	}

	// allocate space for an array of pixels
	int width = img.getWidth();
	int height = img.getHeight();
	unsigned char* pixelsRGBA = new unsigned char[4 * width * height]; // we will use 4 bytes per pixel, i.e., RGBA

	// fill the pixelsRGBA array with the image pixels
	memset(pixelsRGBA, 0, 4 * width * height); // set all bytes to 0
	for (int h = 0; h < height; h++)
		for (int w = 0; w < width; w++)
		{
			// assign some default byte values (for the case where img.getBytesPerPixel() < 4)
			pixelsRGBA[4 * (h * width + w) + 0] = 0; // red
			pixelsRGBA[4 * (h * width + w) + 1] = 0; // green
			pixelsRGBA[4 * (h * width + w) + 2] = 0; // blue
			pixelsRGBA[4 * (h * width + w) + 3] = 255; // alpha channel; fully opaque

			// set the RGBA channels, based on the loaded image
			int numChannels = img.getBytesPerPixel();
			for (int c = 0; c < numChannels; c++) // only set as many channels as are available in the loaded image; the rest get the default value
				pixelsRGBA[4 * (h * width + w) + c] = img.getPixel(w, h, c);
		}

	// bind the texture
	glBindTexture(GL_TEXTURE_2D, textureHandle);

	// initialize the texture
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, pixelsRGBA);

	// generate the mipmaps for this texture
	glGenerateMipmap(GL_TEXTURE_2D);

	// set the texture parameters
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	// query support for anisotropic texture filtering
	GLfloat fLargest;
	glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &fLargest);
	printf("Max available anisotropic samples: %f\n", fLargest);
	// set anisotropic texture filtering
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, 0.5f * fLargest);

	// query for any errors
	GLenum errCode = glGetError();
	if (errCode != 0)
	{
		printf("Texture initialization error. Error code: %d.\n", errCode);
		return -1;
	}

	// de-allocate the pixel array -- it is no longer needed
	delete[] pixelsRGBA;

	return 0;
}

// Note: You should combine this file
// with the solution of homework 1.C:\Users\alass\Graphics-Programming\assign2\hw2-starterCode\splines

// Note for Windows/MS Visual Studio:C:\Users\alass\Graphics-Programming\assign2\hw2-starterCode\splines\star.sp
// You should set argv[1] to track.txt.
// To do this, on the "Solution Explorer",
// right click your project, choose "Properties",
// go to "Configuration Properties", click "Debug",
// then type your track file name for the "Command Arguments".
// You can also repeat this process for the "Release" configuration.

int main(int argc, char* argv[])
{
	if (argc < 2)
	{
		printf("usage: %s <trackfile>\n", argv[0]);
		exit(0);
	}

	// load the splines from the provided filename
	loadSplines(argv[1]);

	printf("Loaded %d spline(s).\n", numSplines);
	for (int i = 0; i < numSplines; i++)
		printf("Num control points in spline %d: %d.\n", i, splines[i].numControlPoints);

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