// #include <glad/glad.h>
// #include <glad/glad.h>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <math.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include "eigen3/Eigen/Dense"
#include <glm/gtc/type_ptr.hpp>
#include <stdlib.h>
#include <stdio.h>

#include <string>
#include <vector>
#include "point_cloud.h"
#include <stdlib.h>

class OpenglPointProcessing{
public:

    std::string w_name;
    OpenglPointProcessing(std::string window_name);
    ~OpenglPointProcessing();

    void init_opengl();
    static void framebuffer_size_callback(GLFWwindow* window, int width, int height);
    void processInput();

    GLFWwindow * window;
    void clear_window();
    void plot_3d_points(PointCloud * pt_cld);
    void plot_global_points(std::vector<PointCloud> * g_pt_cld);
    void terminate();
};