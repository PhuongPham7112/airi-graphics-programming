﻿/*
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
#include <cmath>
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
float maxHeight = -INFINITY;

// Stores the image loaded from disk.
ImageIO* heightmapImage;


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

// camera data structure
struct SloanPoint
{
	glm::vec3 p; // position
	glm::vec3 t; // tangent
	glm::vec3 n; // normal
	glm::vec3 b; // binormal
	SloanPoint(glm::vec3 tang, glm::vec3 norm, glm::vec3 binorm, glm::vec3 pos) : t(tang), n(norm), b(binorm), p(pos) {}
};
unsigned int sloanIndex = 0;
std::vector<SloanPoint*> camTransition; // track the coordinate system of each point on spline

// VBO and VAO for the rail 
std::vector<float> curvePos;
std::vector<float> curveCol;
std::vector<SloanPoint*> crossRail;
GLuint curveVBO;
GLuint curveVAO;

void Subdivide(Point& u0, Point& u1, float maxLength)
{
	Point umid;
	umid.x = (u0.x + u1.x) / 2.0f;
	umid.y = (u0.y + u1.y) / 2.0f;
	umid.z = (u0.z + u1.z) / 2.0f;
}

void insertPointPos(std::vector<float>& pointList, glm::vec3& point)
{
	pointList.push_back(point.x);
	pointList.push_back(point.y);
	pointList.push_back(point.z);
}

void insertPointCol(std::vector<float>& pointList, glm::vec3& col)
{
	pointList.push_back(col.x);
	pointList.push_back(col.y);
	pointList.push_back(col.z);
}

SloanPoint* initSloanPoint(glm::mat4& basisMat, glm::mat3x4& controlMat, float u, std::vector<SloanPoint*>& pointList)
{
	// initial arbitrary vector
	glm::vec3 v = glm::vec3(0.0f, 0.0f, 5.0f);
	glm::vec3 normal;
	glm::vec3 binormal;
	glm::vec3 pos = glm::vec4(pow(u, 3), pow(u, 2), u, 1) * basisMat * controlMat;
	glm::vec3 tangent = glm::normalize(glm::vec4(3.0f * pow(u, 2), 2.0f * u, 1.0f, 0.0f) * basisMat * controlMat); // tangent
	int lastIndex = pointList.size() - 1;
	//cout << "Last index = " << lastIndex << ", last value = " <<  << endl;
	if (pointList.size() == 0)
	{
		normal = glm::normalize(glm::cross(tangent, v));
		binormal = glm::normalize(glm::cross(tangent, normal));
	}
	else
	{
		normal = glm::normalize(glm::cross(pointList[lastIndex]->b, tangent));
		binormal = glm::normalize(glm::cross(tangent, normal));
	}
	SloanPoint* point = new SloanPoint(tangent, normal, binormal, pos);
	return point;
}

void initCameraMove(Spline spl)
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

		// initial arbitrary vector
		glm::vec3 v = glm::vec3(0.0f, 0.0f, 5.0f);

		while (u < 1.0f)
		{
			// move camera along the main point
			SloanPoint* sl = initSloanPoint(basisMat, controlMat, u, camTransition);
			sl->p += sl->n * 0.1f;
			camTransition.push_back(sl);
			// update max height
			// modify for realistic gravity
			glm::vec3 unnormTang = glm::vec4(3.0f * pow(u, 2), 2.0f * u, 1.0f, 0.0f) * basisMat * controlMat;
			uStep = 0.0016f * sqrt(2.0f * 9.8f * (maxHeight - sl->p.z)) / (glm::length(unnormTang));
			u += uStep;
		}
	}
}

void initSplines(Spline spl)
{
	float uStep = 0.001f;
	float s = 0.5f;
	float alpha = 0.05f;

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

		// initial arbitrary vector
		glm::vec3 v = glm::vec3(0.0f, 0.0f, 5.0f);

		while (u < 1.0f)
		{
			glm::vec3 white = glm::vec3(1.0f, 1.0f, 1.0f);
			SloanPoint* sl1 = initSloanPoint(basisMat, controlMat, u, crossRail);
			glm::vec3 v0 = sl1->p + alpha * (-sl1->n + sl1->b);
			glm::vec3 v1 = sl1->p + alpha * (sl1->n + sl1->b);
			glm::vec3 v2 = sl1->p + alpha * (sl1->n - sl1->b);
			glm::vec3 v3 = sl1->p + alpha * (-sl1->n - sl1->b);

			crossRail.push_back(sl1);

			SloanPoint* sl2 = initSloanPoint(basisMat, controlMat, u + uStep, crossRail);
			glm::vec3 v4 = sl2->p + alpha * (-sl2->n + sl2->b);
			glm::vec3 v5 = sl2->p + alpha * (sl2->n + sl2->b);
			glm::vec3 v6 = sl2->p + alpha * (sl2->n - sl2->b);
			glm::vec3 v7 = sl2->p + alpha * (-sl2->n - sl2->b);

			// top
			insertPointPos(curvePos, v1);
			insertPointPos(curvePos, v5);
			insertPointPos(curvePos, v2);
			insertPointPos(curvePos, v6);
			insertPointPos(curvePos, v5);
			insertPointPos(curvePos, v2);
			insertPointCol(curveCol, -sl1->n);
			insertPointCol(curveCol, -sl2->n);
			insertPointCol(curveCol, -sl1->n);
			insertPointCol(curveCol, -sl2->n);
			insertPointCol(curveCol, -sl2->n);
			insertPointCol(curveCol, -sl1->n);

			// right
			insertPointPos(curvePos, v1);
			insertPointPos(curvePos, v5);
			insertPointPos(curvePos, v0);
			insertPointPos(curvePos, v4);
			insertPointPos(curvePos, v5);
			insertPointPos(curvePos, v0);
			insertPointCol(curveCol, -sl1->b);
			insertPointCol(curveCol, -sl2->b);
			insertPointCol(curveCol, -sl1->b);
			insertPointCol(curveCol, -sl2->b);
			insertPointCol(curveCol, -sl2->b);
			insertPointCol(curveCol, -sl1->b);

			// left
			insertPointPos(curvePos, v2);
			insertPointPos(curvePos, v6);
			insertPointPos(curvePos, v3);
			insertPointPos(curvePos, v7);
			insertPointPos(curvePos, v6);
			insertPointPos(curvePos, v3);
			insertPointCol(curveCol, sl1->b);
			insertPointCol(curveCol, sl2->b);
			insertPointCol(curveCol, sl1->b);
			insertPointCol(curveCol, sl2->b);
			insertPointCol(curveCol, sl2->b);
			insertPointCol(curveCol, sl1->b);

			// bottom
			insertPointPos(curvePos, v0);
			insertPointPos(curvePos, v4);
			insertPointPos(curvePos, v3);
			insertPointPos(curvePos, v7);
			insertPointPos(curvePos, v4);
			insertPointPos(curvePos, v3);
			insertPointCol(curveCol, sl1->n);
			insertPointCol(curveCol, sl2->n);
			insertPointCol(curveCol, sl1->n);
			insertPointCol(curveCol, sl2->n);
			insertPointCol(curveCol, sl2->n);
			insertPointCol(curveCol, sl1->n);

			// find max height
			if (sl1->p.z > maxHeight)
			{
				maxHeight = sl1->p.z;
			}

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
	glVertexAttribPointer(locationOfColor, 3, GL_FLOAT, normalized, stride, (const void*)(unsigned long)numBytesInPositions); // The shader variable "color" receives its data from the currently bound VBO (i.e., vertexPositionAndColorVBO), starting from offset "numBytesInPositions" in the VBO. There are 4 float entries per vertex in the VBO (i.e., r,g,b,a channels). 
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
	for (int i = 0; i < numSplines; i++)
	{
		initCameraMove(splines[i]);
	}
	sloanIndex = 0;
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
	// update camera in here?
	if (sloanIndex < camTransition.size())
	{
		matrix.LookAt(
			camTransition[sloanIndex]->p.x,
			camTransition[sloanIndex]->p.y,
			camTransition[sloanIndex]->p.z,
			camTransition[sloanIndex]->t.x + camTransition[sloanIndex]->p.x,
			camTransition[sloanIndex]->t.y + camTransition[sloanIndex]->p.y,
			camTransition[sloanIndex]->t.z + camTransition[sloanIndex]->p.z,
			camTransition[sloanIndex]->n.x,
			camTransition[sloanIndex]->n.y,
			camTransition[sloanIndex]->n.z);
		sloanIndex++;
	}
	else
	{
		sloanIndex = 0;
	}

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
	GLuint program = pipelineProgram->GetProgramHandle();

	// upload light direction
	GLint h_viewLightDirection = glGetUniformLocation(program, "viewLightDirection");
	glm::mat4 viewMat = glm::mat4(
		modelViewMatrix[0],
		modelViewMatrix[1],
		modelViewMatrix[2],
		modelViewMatrix[3],
		modelViewMatrix[4],
		modelViewMatrix[5],
		modelViewMatrix[6],
		modelViewMatrix[7],
		modelViewMatrix[8],
		modelViewMatrix[9],
		modelViewMatrix[10],
		modelViewMatrix[11],
		modelViewMatrix[12],
		modelViewMatrix[13],
		modelViewMatrix[14],
		modelViewMatrix[15]
	);
	float viewLightDirection[3];
	glm::vec4 dir = viewMat * glm::vec4(0.0f, 1.0f, 0.0f, 0.0f);
	viewLightDirection[0] = dir.x;
	viewLightDirection[1] = dir.y;
	viewLightDirection[2] = dir.z;
	glUniform3fv(h_viewLightDirection, 1, viewLightDirection);

	// upload normal matrix
	GLint h_normalMatrix = glGetUniformLocation(program, "normalMatrix");
	float n[16];
	matrix.SetMatrixMode(OpenGLMatrix::ModelView);
	matrix.GetNormalMatrix(n);
	GLboolean isRowMajor = GL_FALSE;
	glUniformMatrix4fv(h_normalMatrix, 1, isRowMajor, n);

	// setting coefficients
	float light[4] = { 1.0f, 1.0f, 1.0f, 1.0f };
	GLint h_La = glGetUniformLocation(program, "La");
	glUniform4fv(h_La, 1, light);
	GLint h_Ld = glGetUniformLocation(program, "Ld");
	glUniform4fv(h_Ld, 1, light);
	GLint h_Ls = glGetUniformLocation(program, "Ls");
	glUniform4fv(h_Ls, 1, light);

	float ka[4] = { 0.2f, 0.2f, 0.2f, 0.2f };
	GLint h_Ka = glGetUniformLocation(program, "ka");
	glUniform4fv(h_Ka, 1, ka);

	float kd[4] = { 0.3f, 0.3f, 0.3f, 0.3f };
	GLint h_Kd = glGetUniformLocation(program, "kd");
	glUniform4fv(h_Kd, 1, kd);

	float ks[4] = { 0.3f, 0.3f, 0.3f, 0.3f };
	GLint h_Ks = glGetUniformLocation(program, "ks");
	glUniform4fv(h_Ks, 1, ks);

	GLint h_alpha = glGetUniformLocation(program, "alpha");
	glUniform1f(h_alpha, 0.5f);

	// render
	glBindVertexArray(curveVAO);
	glDrawArrays(GL_TRIANGLES, 0, curvePos.size() / 3);
	glBindVertexArray(0); // unbind the VAO

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