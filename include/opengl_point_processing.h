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
// #include "shader_2.h"
#include "shader.h"
#include "calculate_transformations.h"

class OpenglPointProcessing{
public:

    OpenglPointProcessing(std::string window_name);
    ~OpenglPointProcessing();

    Shader shader;
    unsigned int VBO, VAO;

    std::string w_name;

    int screenWidth;
    int screenHeight;

    CalTransform c_trans;



    typedef struct{
        GLfloat x, y, z;
        GLfloat r, g, b, a;
    } Vertex;

    void init_opengl();
    static void framebuffer_size_callback(GLFWwindow* window, int width, int height);
    void processInput();

    GLFWwindow * window;
    void clear_window();
    void draw_points(Vertex v, GLfloat size);
    void drawPointsDemo(int width, int height);
    void draw_point_3d(PointCloud & pt_cld, GLfloat size);
    void draw_point_global(std::vector<PointCloud> & global_cloud, GLfloat size);

    void draw_axis(float line_length, float line_width);

    void plot_3d_points(PointCloud & pt_cld);
    void plot_global_points(std::vector<PointCloud> & g_pt_cld, std::vector<float> & state);
    void terminate();

    glm::mat4 eigen_mat4_to_glm_mat4(Eigen::Matrix4f & e_mat4);
};