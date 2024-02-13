#ifndef _IMAGELOADER_H_
#define _IMAGELOADER_H_

#include "openGL-headers.h"
#include <string>

// Texture
extern GLuint textureID;

struct BMP
{
  int sizeX;
  int sizeY;
  unsigned char *data;
};

BMP* getBMPData(std::string filename);
void loadTextures(std::string filename);

#endif
