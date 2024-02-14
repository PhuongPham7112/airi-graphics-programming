#if defined(WIN32)  || defined(_WIN32) 
#include <Windows.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <glm.hpp>
#include <gtc/type_ptr.hpp> // For glm::make_mat4
#include <GL/freeglut.h>
#elif defined(linux) || defined(__linux__)
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#elif defined(__APPLE__)
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#endif

