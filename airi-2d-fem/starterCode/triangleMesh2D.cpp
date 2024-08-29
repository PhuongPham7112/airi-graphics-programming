/* 
  CSCI 520 Computer Animation and Simulation
  Programming Assignment:
  2D Simulator for Mass spring, Linear FEM and Corotational FEM

  Mianlun Zheng and Jernej Barbic
  University of Southern California
*/

#include "triangleMesh2D.h"
#include <float.h>
#include <math.h>
#include <string.h>
#include <vector>
#include <string>
#include <map>
#include <iomanip>
#include <set>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <functional>
#include <cctype>
#include <assert.h>
#include <cassert>
#include "fileIO.h"
#include "stringHelper.h"
// #include "macros.h"

using namespace std;

void TriangleMesh2D::convertWhitespaceToSingleBlanks(char * s)
{
  char * p = s;
  while (*p != 0)
  {
    // erase consecutive empty space characters, or end-of-string spaces
    while ((*p == ' ') && ((*(p+1) == 0) || (*(p+1) == ' ')))
    {
      char * q = p;
      while (*q != 0) // move characters to the left, by one character
      {
        *q = *(q+1);
        q++;
      }
    }
    p++;
  }
}

int TriangleMesh2D::loadFromAscii(const string & filename, int verbose)
{
  unsigned int numFaces = 0;

  const int maxline = 4096;
  std::ifstream ifs(filename.c_str());
  char line[maxline];

  unsigned int currentGroup=0;
  unsigned int ignoreCounter=0;

  unsigned int currentMaterialIndex = 0;

  // Note: the default material will be added when encountered in the obj file, or at the end if necessary. One cannot simply add it here at the beginning because a material read from the .mtl file could also be called "default".

  if (verbose)
    std::cout << "Parsing .obj file '" << filename << "'." << std::endl;

  if (!ifs)
  {
    std::string message = "Could not open .obj file '";
    message.append(filename);
    message.append( "'" );
    throw TriangleMesh2DException( message );
  }

  int lineNum = 0;
  int numGroupFaces = 0;
  int groupCloneIndex = 0;
  std::string groupSourceName;

  while(ifs)
  {
    lineNum++;
    ifs.getline(line, maxline);
    if (strlen(line) > 0)
    {
      // if ending in '\\', the next line should be concatenated to the current line
      int lastCharPos = (int)strlen(line)-1;
      while(line[lastCharPos] == '\\')
      {
        line[lastCharPos] = ' ';  // first turn '\' to ' '
        char nextline[maxline];
        ifs.getline(nextline, maxline);
        strcat(line, nextline);
        lastCharPos = (int)strlen(line)-1;
      }
    }

    std::string lineString(line);
    // trim white space ahead
    lineString.erase(lineString.begin(), std::find_if(lineString.begin(), lineString.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
    // trim white space in the end
    lineString.erase(std::find_if(lineString.rbegin(), lineString.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), lineString.end());


    memset(line, 0, maxline);
    strcpy(line, lineString.c_str());

    convertWhitespaceToSingleBlanks(line);

    char command = line[0];

    if (strncmp(line,"v ",2) == 0) // vertex
    {
      //std::cout << "v " ;
      Vec2d pos;
      if (sscanf(line, "v %lf %lf\n", &pos[0], &pos[1]) < 2)
      {
        throw TriangleMesh2DException("Invalid vertex", filename, lineNum);
      }
      vertexPosition.push_back( pos );
      restVertexPosition.push_back( pos );
    }
    //normal
    else if (strncmp(line, "vn ", 3) == 0)
    {
      // std::cout << "vn " 
      // Vec3d normal;
      // if (sscanf(line,"vn %lf %lf %lf\n", &normal[0], &normal[1], &normal[2]) < 3)
      // {
      //   throw TriangleMesh2DException("Invalid normal", filename, lineNum);
      // }
      // normals.push_back(normal);
    }
    //texture
    else if (strncmp(line, "vt ", 3) == 0 )
    {
      //std::cout << "vt " ;
      Vec3d tex(0.0);
      double x,y;
      if (sscanf(line, "vt %lf %lf\n", &x, &y) < 2)
      {
        throw TriangleMesh2DException("Invalid texture coordinate", filename, lineNum);
      }
      tex = Vec3d(x,y,0);
      textureCoordinates.push_back(tex);
    }
    else if (strncmp(line, "g ", 2) == 0)
    {
      // remove last newline
      if (strlen(line) > 0)
      {
        if (line[strlen(line)-1] == '\n')
          line[strlen(line)-1] = 0;
      }

      // remove last carriage return
      if (strlen(line) > 0)
      {
        if (line[strlen(line)-1] == '\r')
          line[strlen(line)-1] = 0;
      }

      std::string name;
      if (strlen(line) < 2)
      {
        if (verbose)
          cout << "Warning:  Empty group name encountered: " << filename << " " << lineNum << endl;
        name = string("");
      }
      else
        name = string(&line[2]);

      //printf("Detected group: %s\n", &line[2]);

      // check if this group already existsf
      bool groupFound = false;
      unsigned int counter = 0;
      for(std::vector< Group >::const_iterator itr = groups.begin(); itr != groups.end(); itr++)
      {
        if (itr->getName() == name)
        {
          currentGroup = counter;
          groupFound = true;
          break;
        }
        counter++;
      }
      if (!groupFound)
      {
        groups.push_back(Group(name, currentMaterialIndex));
        currentGroup = groups.size() - 1;
        numGroupFaces = 0;
        groupCloneIndex = 0;
        groupSourceName = name;
      }
    }
	else if ((strncmp(line, "f ", 2) == 0) || (strncmp(line, "fo ", 3) == 0))
    {
      char * faceLine = &line[2];
      if (strncmp(line, "fo", 2) == 0)
        faceLine = &line[3];

      //std::cout << "f " ;
      if (groups.empty())
      {
        groups.emplace_back();
        currentGroup = 0;
      }

      Vec3i triangle;

      // the faceLine string now looks like the following:
      //   vertex1 vertex2 ... vertexn
      // where vertexi is v/t/n, v//n, v/t, or v

      char * curPos = faceLine;
      int index = 0;
      while( *curPos != '\0' )
      {
        if(index >2){
          throw TriangleMesh2DException( "Not Triangle Mesh", filename, triangles.size());
        }
        // seek for next whitespace or eof
        char * tokenEnd = curPos;
        while ((*tokenEnd != ' ') && (*tokenEnd != '\0'))
          tokenEnd++;

        bool whiteSpace = false;
        if (*tokenEnd == ' ')
        {
          *tokenEnd = '\0';
          whiteSpace = true;
        }

        int pos;
        int nor;
        int tex;
        std::pair< bool, unsigned int > texPos;
        std::pair< bool, unsigned int > normal;

        // now, parse curPos
        if (strstr(curPos,"//") != NULL)
        {
          if (sscanf(curPos, "%d//%d", &pos, &nor) < 2)
          {
            throw TriangleMesh2DException( "Invalid face", filename, lineNum);
          }

          // v//n
          if (pos < 0)
            pos = (int)vertexPosition.size() + pos + 1;
          // if (nor < 0)
          //   nor = (int)normals.size() + nor + 1;

          //texPos = make_pair(false, 0);
          //normal = make_pair(true, (unsigned int)nor);
        }
        else
        {
          if (sscanf(curPos, "%d/%d/%d", &pos, &tex, &nor) != 3)
          {
            if (strstr(curPos, "/") != NULL)
            {
              if (sscanf(curPos, "%d/%d", &pos, &tex) == 2)
              {
                // v/t
                if (pos < 0)
                  pos = (int)vertexPosition.size() + pos + 1;
                // if (tex < 0)
                //   tex = (int)textureCoordinates.size() + tex + 1;

                //texPos = make_pair(true, (unsigned int)tex);
                //normal = make_pair(false, 0);
              }
              else
              {
                throw TriangleMesh2DException("Invalid face", filename, lineNum);
              }
            }
            else
            {
              if (sscanf(curPos, "%d", &pos) == 1)
              {
                // v
                if (pos < 0)
                  pos = (int)vertexPosition.size() + pos + 1;

                //texPos = make_pair(false, 0);
                //normal = make_pair(false, 0);
              }
              else
              {
                throw TriangleMesh2DException("Invalid face", filename, lineNum);
              }
            }
          }
          else
          {
            // v/t/n
            if (pos < 0)
              pos = (int)vertexPosition.size() + pos + 1;
            // if (tex < 0)
            //   tex = (int)textureCoordinates.size() + tex + 1;
            // if (nor < 0)
            //   nor = (int)normals.size() + nor + 1;

            //texPos = make_pair(true, (unsigned int)tex);
           // normal = make_pair(true, (unsigned int)nor);
          }
        }

        // sanity check
        if ((pos < 1) || (pos > (int)vertexPosition.size()))
        {
          printf("Error: vertex %d is out of bounds.\n", pos);
          throw 51;
        }
        
        // decrease indices to make them 0-indexed
        pos--;
        triangle[index++] = pos;
        //face.addVertex(Vertex((unsigned int)pos, texPos, normal));

        if (whiteSpace)
        {
          *tokenEnd = ' ';
          curPos = tokenEnd + 1;
        }
        else
          curPos = tokenEnd;
      }

      triangles.push_back(triangle);
      numFaces++;
      groups[currentGroup].addFace(triangle);
      numGroupFaces++;
    }
    else if ((strncmp(line, "#", 1) == 0 ) || (strncmp(line, "\0", 1) == 0))
    {
      // ignore comment lines and empty lines
    }
    else if (strncmp(line, "usemtl", 6) == 0)
    {
      // switch to a new material
      if (numGroupFaces > 0)
      {
        // usemtl without a "g" statement; must create a new group
        // first, create unique name
        char newNameC[4096];
        sprintf(newNameC, "%s.%d", groupSourceName.c_str(), groupCloneIndex);
        //printf("Splitting group...\n");
        //printf("New name=%s\n", newNameC);
        std::string newName(newNameC);
        groups.push_back(Group(newName, currentMaterialIndex));
        currentGroup = groups.size()-1;
        numGroupFaces = 0;
        groupCloneIndex++;
      }

      materialSearch:
      bool materialFound = false;
      unsigned int counter = 0;
      char * materialName = &line[7];
      for(std::vector< Material >::const_iterator itr = materials.begin(); itr != materials.end(); itr++)
      {
        if (itr->getName() == string(materialName))
        {
          currentMaterialIndex = counter;

          // update current group
          if (groups.empty())
          {
            groups.emplace_back();
            currentGroup = 0;
          }

          groups[currentGroup].setMaterialIndex(currentMaterialIndex);
          materialFound = true;
          break;
        }
        counter++;
      }

      if (!materialFound)
      {
        if (strcmp(materialName, "default") == 0)
        {
          addDefaultMaterial();
          goto materialSearch;
        }

        char msg[4096];
        sprintf(msg, "Obj mesh material %s does not exist.\n", materialName);
        throw TriangleMesh2DException(msg);
      }
    }
    else if (strncmp(line, "mtllib", 6) == 0)
    {
      char mtlFilename[4096];
      strcpy(mtlFilename, filename.c_str());
      try
      {
        parseMaterials(mtlFilename, &line[7], verbose);
      }
      catch(TriangleMesh2DException & o)
      {
        cout << o.what() << endl;
      }
    }
    else if ((strncmp(line, "s ", 2) == 0 ) || (strncmp(line, "o ", 2) == 0))
    {
      // ignore lines beginning with s and o
      //std::cout << command << " ";
      if (ignoreCounter < 5)
      {
        if (verbose)
          std::cout << "Warning: ignoring '" << command << "' line" << std::endl;
        ignoreCounter++;
      }
      if (ignoreCounter == 5)
      {
        if (verbose)
          std::cout << "(suppressing further output of ignored lines)" << std::endl;
        ignoreCounter++;
      }
    }
    else
    {
      //std::cout << "invalid ";
      std::ostringstream msg;
      msg << "Invalid line in .obj file '" << filename << "': " << line;
      throw TriangleMesh2DException(msg.str(), filename, lineNum);
    }
  }

  // add the "default" material if it doesn't already exist
  addDefaultMaterial();
  numTriangles = triangles.size();
  numVertices = vertexPosition.size();

  numElements = numTriangles;
  elements.resize(3 * numElements);
  for(int i=0; i< numElements; i++)
  {
    elements[i*3 + 0] = triangles[i][0];
    elements[i*3 + 1] = triangles[i][1];
    elements[i*3 + 2] = triangles[i][2];
  }
  return 0;
}

void TriangleMesh2D::fgets_(char * s, int n, FILE * stream)
{
  char * result = fgets(s, n, stream);
  if (result == NULL)
    printf("Warning: bad input file syntax. fgets_ returned NULL.\n");
  return;
}

void TriangleMesh2D::parseMaterials(const std::string & objMeshFilename, const std::string & materialFilename, int verbose)
{
  FILE * file;
  //char buf[128];
  //unsigned int numMaterials;

  char objMeshFilenameCopy[4096];
  strcpy(objMeshFilenameCopy, objMeshFilename.c_str());

  string filename = getPathDirectoryName(objMeshFilename) + "/" + materialFilename;

  file = fopen(filename.c_str(), "r");
  if (!file)
  {
    fprintf(stderr, " parseMaterials() failed: can't open material file %s.\n", filename.c_str());
    std::string message = "Failed to open material file '";
    message.append(filename);
    message.append("'");
    throw TriangleMesh2DException(message);
  }

  // default material
  Material defaultMat;
  Vec3d Ka = defaultMat.getKa();
  Vec3d Kd = defaultMat.getKd();
  Vec3d Ks = defaultMat.getKs();
  double shininess = defaultMat.getShininess();
  string matName;
  string textureFile;
  // now, read in the data
  char buf[4096];
  unsigned int numMaterials = 0;
  while(fscanf(file, "%s", buf) != EOF)
  {
    switch(buf[0])
    {
      case '#':
        // comment
        // ignore the rest of line
        fgets_(buf, sizeof(buf), file);
      break;

      case 'n':
        // newmtl
        if (numMaterials >= 1) // flush previous material
          addMaterial(matName, Ka, Kd, Ks, shininess, textureFile);

        // reset to default
        Ka = defaultMat.getKa();
        Kd = defaultMat.getKd();
        Ks = defaultMat.getKs();
        shininess = defaultMat.getShininess();
        textureFile.clear();

        fgets_(buf, sizeof(buf), file);
        sscanf(buf, "%s %s", buf, buf);
        numMaterials++;
        matName = stripLight(buf);
      break;

      case 'N':
        if (buf[1] == 's')
        {
          if (fscanf(file, "%lf", &shininess) < 1)
            printf("Warning: incorect mtl file syntax. Unable to read shininess.\n");
          // wavefront shininess is from [0, 1000], so scale for OpenGL
          shininess *= 128.0 / 1000.0;
        }
        else
          fgets_(buf, sizeof(buf), file); // ignore the rest of the line
      break;

      case 'K':
        switch(buf[1])
        {
          case 'd':
            if (fscanf(file, "%lf %lf %lf", &Kd[0], &Kd[1], &Kd[2]) < 3)
              printf("Warning: incorect mtl file syntax. Unable to read Kd.\n");
          break;

          case 's':
            if (fscanf(file, "%lf %lf %lf", &Ks[0], &Ks[1], &Ks[2]) < 3)
              printf("Warning: incorect mtl file syntax. Unable to read Ks.\n");
           break;

          case 'a':
            if (fscanf(file, "%lf %lf %lf", &Ka[0], &Ka[1], &Ka[2]) < 3)
              printf("Warning: incorect mtl file syntax. Unable to read Ka.\n");
          break;

          default:
            // ignore the rest of the line
            fgets_(buf, sizeof(buf), file);
          break;
        }
      break;

      case 'm':
        if (strcmp(buf, "map_Kd") == 0)
        {
          fgets_(buf, sizeof(buf), file);
          sscanf(buf, "%s %s", buf, buf);
          textureFile = stripLight(buf);
          if (verbose)
            printf("Noticed texture %s.\n", textureFile.c_str());
        }
      break;

      default:
        // ignore the rest of the line
        fgets_(buf, sizeof(buf), file);
      break;
    }
  }

  if (numMaterials >= 1) // flush last material
    addMaterial(matName, Ka, Kd, Ks, shininess, textureFile);

  fclose(file);
}

TriangleMesh2D::TriangleMesh2D(const std::string & filename_, int verbose = 1)
{
  triangleMeshFilename = filename_;
  loadFromAscii(filename_, verbose);

  if (verbose)
  {
    std::cout << "Parsed obj file '" << filename_ << "'; statistics:" << std::endl;
    std::cout << "   " << triangles.size() << " triangles," << std::endl;
    std::cout << "   " << vertexPosition.size() << " vertices," << std::endl;
  }
}

TriangleMesh2D::~TriangleMesh2D()
{

}

TriangleMesh2DException::TriangleMesh2DException(const std::string & reason)
{
  std::ostringstream oss;
  oss << "Error:  " << reason;
  std::cout << std::endl << oss.str() << std::endl;
}

TriangleMesh2DException::TriangleMesh2DException(const std::string & reason, const std::string & filename, unsigned int line)
{
  std::ostringstream oss;
  oss << "Error in file '" << filename << "', line " << line << ": " << reason;
  std::cout << std::endl << oss.str() << std::endl;
}

int TriangleMesh2D::addDefaultMaterial()
{
  // search if there already is the "default" material
  int numObjMaterials = getNumMaterials();
  for (int materialIndex=0; materialIndex<numObjMaterials; materialIndex++)
  {
    if(materials[materialIndex].getName() == "default")
    {
      return materialIndex;
    }
  }

  materials.emplace_back();
  return materials.size()-1;
}


double TriangleMesh2D::getElementArea(int element)
{
    Vec2d p0 = restVertexPosition[triangles[element][0]];
    Vec2d p1 = restVertexPosition[triangles[element][1]];
    Vec2d p2 = restVertexPosition[triangles[element][2]];
    double s0[3] = { p1[0] - p0[0], p1[1] - p0[1], 0};
    double s1[3] = { p2[0] - p0[0], p2[1] - p0[1], 0};
    double crossp[3];
    CROSSPRODUCT(s0[0], s0[1], s0[2], s1[0], s1[1], s1[2], crossp[0], crossp[1], crossp[2]);
    return 0.5 * sqrt(crossp[0]*crossp[0] + crossp[1]*crossp[1] + crossp[2]*crossp[2]);
}

