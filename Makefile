# Makefile. Created by Lucio A. Rocha at 30-09-2023
# Based on: https://www.inf.ufpr.br/roberto/ci067/15_make.html

FREETYPE2 = /usr/include/freetype2
SHADER = ./shader
LIB_FREETYPE2 = /usr/local/lib
CXX = g++
CXXFLAGS = -Wall -O2 -std=c++11 -I$(FREETYPE2) -L$(LIB_FREETYPE2)
OPENGL = -lGL -lglut -lGLU -lfreetype -lGLEW  

all: clean Main.o
	$(CXX) $(CFLAGS) Main.o common/shader_utils.cpp -o bioswarm $(OPENGL)   

clean: 
	rm -f *.o bioswarm *~ *.*~
