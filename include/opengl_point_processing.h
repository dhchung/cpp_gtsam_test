#pragma once




#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <math.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include "Eigen/Dense"
#include <glm/gtc/type_ptr.hpp>
#include <stdlib.h>
#include <stdio.h>

#include <string>
#include <vector>
#include "point_cloud.h"
#include "shader.h"
#include "calculate_transformations.h"
#include "camera.h"

class OpenglPointProcessing{
public:

    OpenglPointProcessing(std::string window_name);
    ~OpenglPointProcessing();

    Shader shader;
    unsigned int VBO, VAO;

    std::string w_name;

    int screenWidth;
    int screenHeight;
    float lastX;
    float lastY;
    bool firstMouse;
    float deltaTime;
    float lastFrame;


    CalTransform c_trans;

    Camera * camera;


    typedef struct{
        GLfloat x, y, z;
        GLfloat r, g, b, a;
    } Vertex;

    void init_opengl();
    static void framebuffer_size_callback(GLFWwindow* window, int width, int height);
    void processInput();
    void processInput_end();

    GLFWwindow * window;
    void clear_window();



    void draw_point_global(std::vector<PointCloud> & global_cloud, GLfloat size);

    void draw_axis(float line_length, float line_width);

    void plot_3d_points(PointCloud & pt_cld);
    void plot_global_points(std::vector<PointCloud> & g_pt_cld, std::vector<float> & state, int & idx);
    void terminate();

    glm::mat4 eigen_mat4_to_glm_mat4(Eigen::Matrix4f & e_mat4);


    static void mouse_callback(GLFWwindow * window, double xpos, double ypos);
    static void scroll_callback(GLFWwindow * window, double xoffset, double yoffset);

    void mouse_callback_function(double xpos, double ypos);
    void scroll_callback_function(double xoffset, double yoffset);
};