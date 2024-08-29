# CSCI 520 FEM 2D Simulator 
# Mianlun Zheng, Jernej Barbic, USC

CXX = g++
#CXXFLAGS= -g -std=c++11 -fsanitize=address -fsanitize=undefined
CXXFLAGS= -g -O3 -std=c++11 -DGL_SILENCE_DEPRECATION -Wno-deprecated-declarations -Wno-deprecated

# the object files to be compiled for this utility
SIMULATOR2D_OBJECTS= simulator2D.o baseModel2D.o stencilForceModel2D.o triangleMeshRendering2D.o triangleMesh2D.o linearFEM2D.o massSpring2D.o corotationalLinearFEM2D.o 

# the headers in this library
SIMULATOR2D_HEADERS=triangleMesh2D.h triangleMeshRendering2D.h baseModel2D.h linearFEM2D.h massSpring2D.h corotationalLinearFEM2D.h simulator2D.h

# the libraries this utility depends on
SIMULATOR2D_LIBS= vec2d.o stringHelper.o matrixIO.o listIO.o sparseMatrix.o graph.o constrainedDOFs.o openGLHelper.o configFile.o averagingBuffer.o forceModelAssembler2D.o fileIO.o linearSolver.o CGSolver.o  integratorBase.o integratorBaseSparse.o implicitNewmarkSparse.o implicitBackwardEulerSparse.o centralDifferencesSparse.o eulerSparse.o forceModel.o ortho2DCamera.o imageIO.o saveScreenShot.o 

KERNEL=$(shell uname -s)
ifeq ($(KERNEL),Linux)
	OPENGL_LIBS = -lGL -lGLU -lglut
else 
ifeq ($(KERNEL),Darwin)
	OPENGL_LIBS = -framework OpenGL /usr/local/Cellar/freeglut/3.2.1/lib/libglut.dylib 
endif
endif

IMGUI_INCLUDE=-Iimgui/include
IMGUI_LIB=-limgui -limgui_impl_glut -limgui_impl_opengl2 -Limgui/lib

IMAGE_LIB=-lpng -ljpeg -lz

INCLUDE = -Ivega/ $(IMGUI_INCLUDE)

ALL = simulator2D
all: $(ALL)

simulator2D : $(SIMULATOR2D_OBJECTS) vega/libpartialVega.a
	$(CXX) $(CXXFLAGS) $(INCLUDE) $^ $(IMGUI_LIB) $(OPENGL_LIBS) $(IMAGE_LIB) -lm -o $@

vega/libpartialVega.a:  $(addprefix vega/, $(SIMULATOR2D_LIBS))
	ar r $@ $^

$(SIMULATOR2D_OBJECTS): %.o: %.cpp $(SIMULATOR2D_HEADERS) vega/*.h
	$(CXX) $(CXXFLAGS) $(INCLUDE) -c $< -o $@

$(SIMULATOR2D_LIBS): %.o: %.cpp vega/*.h
	$(CXX) $(CXXFLAGS) $(INCLUDE) -c $^ -o $@

clean:
	-rm -rf core *.o vega/*.o vega/libpartialVega.a $(ALL)

