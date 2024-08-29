#include "ortho2DCamera.h"
#include "openGL-headers.h"
#include <cstring>
#include <iostream>

Ortho2DCamera::Ortho2DCamera(double fx, double fy, double ws)
{
  focus[0] = fx;
  focus[1] = fy;
  scale = 2. / ws;
}

void Ortho2DCamera::look()
{
  //  double cameraPosition[3] = {0.5,0.5,1};
  //  double focusPosition[3] = {0.5,0.5,-1};
  double up[3] = {0,1,0};
  glScalef(scale, scale, 1);
  gluLookAt(focus[0], focus[1], 1, focus[0], focus[1], -1, up[0], up[1], up[2]);
}

void Ortho2DCamera::moveRight(double dx)
{
  focus[0] += dx;
}

void Ortho2DCamera::moveUp(double dy)
{
  focus[1] += dy;
}

void Ortho2DCamera::zoomIn(double zoomInFactor)
{
  scale *= zoomInFactor;
}

void Ortho2DCamera::getFocusPosition(double f[2]) const
{
  memcpy(f, focus, sizeof(double) * 2);
}

void Ortho2DCamera::cameraVector2WorldVector(const double camera[2], double world[2]) const
{
  for(int i = 0; i < 2; i++)
    world[i] = camera[i] / scale + focus[i];
}

void Ortho2DCamera::cameraVector2WorldVector_Differnce(const double camera[2], double world[2]) const
{
  for(int i = 0; i < 2; i++)
    world[i] = camera[i] / scale;
}
