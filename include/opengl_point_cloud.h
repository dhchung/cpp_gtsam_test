#include <iostream>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>


#include "point_cloud.h"


class OpenglPointCloud{
public:
    OpenglPointCloud();
    ~OpenglPointCloud();


    void framebuffer_size_callback(GLFWwindow* window, int width, int height);
    void processInput(GLFWwindow * window);

    int InitializeOpenGL();

    void show_point_cloud(PointCloud & pt_cld);
    void show_all_point_cloud();


};