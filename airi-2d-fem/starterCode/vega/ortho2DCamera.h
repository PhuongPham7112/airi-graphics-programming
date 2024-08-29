#ifndef ORTHO2DCAMERA_H
#define ORTHO2DCAMERA_H

// A orthogonal 2D projection OpenGL camera class, with the ability to set the OpenGL modelview transformation matrix.
// It has two parameters: focusPosition={focusX, focusY} and scale. It is useful for rendering 2D graphics.

class Ortho2DCamera
{
public:
  //            ^ y
  //            |
  //            |
  // -------------------------x
  //            | focus
  //            |
  //            |
  // default OpenGL ortho2d focus is (0,0) and window size is 2: gluOrtho2D(left=-1,right=1,bottom=-1,top=1)
  // z coordinate is always 0 and camera is shooting towards -z
  // the default parameters for Ortho2DCamera conform to the default OpenGL ortho2d
  Ortho2DCamera(double focusX=0, double focusY=0, double windowSize = 2);
  virtual ~Ortho2DCamera() {}

  // calling gluLookAt to perform coordinates transform
  // in order for Ortho2DCamera to work properly, OpenGL projection matrix must be set to identity,
  //   OpenGL model-view matrix is reset to identity and set active before look() is called
  void look();

  // move camera by amount in world coordinate
  void moveRight(double dx);
  void moveUp(double dy);

  // set a new scale for camera. this scale is used by calling glScalef(scale, scale, 1) to scale OpenGL view
  void setScale(double newScale) { scale = newScale; }

  // zoomIn(zoomInFactor=2) means zooming in 2X
  // its implemented as scale *= zoomInFactor
  void zoomIn(double zoomInFactor);

  void getFocusPosition(double f[2]) const; 
  double getScale() const { return scale; }

  // transform a location specified in the camera coordinate system into the world coordinate system
  // camera coordinate is the same as default openGL ortho2d: x range: [-1,1], y range: [-1,1]
  void cameraVector2WorldVector(const double camera[2], double world[2]) const;
  // same as above, except that it doesn't take into account the position of the camera
  // useful for transforming velocities
  void cameraVector2WorldVector_Differnce(const double camera[2], double world[2]) const;

protected:
  double focus[2];
  double scale;
};

#endif
