
#pragma once

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <string>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <opencv2/opencv.hpp>

GLuint LoadShaders(const char * vertex_file_path,const char * fragment_file_path);
