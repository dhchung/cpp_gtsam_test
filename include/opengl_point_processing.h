#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "point_cloud.h"
#include "eigen3/Eigen/Dense"
#include <string>
#include <vector>

#include <iostream>
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