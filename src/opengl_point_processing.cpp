#include "opengl_point_processing.h"


OpenglPointProcessing::OpenglPointProcessing(std::string window_name){
    w_name = window_name;
}
OpenglPointProcessing::~OpenglPointProcessing(){

}

void OpenglPointProcessing::init_opengl(){
    glfwInit(); //Initialize GLFW
    window = glfwCreateWindow(640,480, "Test opengl", NULL, NULL);
    if(window == NULL) {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
    }
    
    glfwMakeContextCurrent(window); //Tell GLFW to setup "window context" as primary context for current thread
    glEnable(GL_POINT_SMOOTH);
    glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

}

void OpenglPointProcessing::framebuffer_size_callback(GLFWwindow* window, int width, int height) { // Change viewport if the user change the size of the window
    glViewport(0, 0, width, height);
}

void OpenglPointProcessing::processInput() {
    if(glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, true);
    }
}

void OpenglPointProcessing::plot_3d_points(PointCloud & pt_cld){

    float ratio;
    int width, height;
    glfwGetFramebufferSize(window, &width, &height);
    ratio = (float) width / (float) height;
    glViewport(0, 0, width, height);
    glClear(GL_COLOR_BUFFER_BIT);


    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    // glOrtho(-ratio, ratio, -1.f, 1.f, 1.f, -1.f);


    glOrtho(-2, 2, -2.f, 2.f, 10.f, -10.f);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    draw_point_3d(pt_cld, 3.0f);
    
    glfwSwapBuffers(window);
    glfwPollEvents();   
}



void OpenglPointProcessing::plot_global_points(std::vector<PointCloud> & g_pt_cld){
    
}

void OpenglPointProcessing::terminate(){
    glfwTerminate();
}

void OpenglPointProcessing::clear_window(){
    processInput();
    glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
}

void OpenglPointProcessing::draw_points(Vertex v, GLfloat size){
    glPointSize(size);
    glBegin(GL_POINTS);
    glColor4f(v.r, v.b, v.g, v.a);
    glVertex3f(v.x, v.y, v.z);
    glEnd();
}


void OpenglPointProcessing::draw_point_3d(PointCloud & pt_cld, GLfloat size){
    for(int p_idx=0; p_idx < pt_cld.point_cloud.cols(); ++p_idx) {
        Vertex v1 = {pt_cld.point_cloud(0,p_idx)/1000.0f,
                     pt_cld.point_cloud(1,p_idx)/1000.0f,
                     pt_cld.point_cloud(2,p_idx)/1000.0f,
                     float(pt_cld.point_color(0,p_idx))/255.0f,
                     float(pt_cld.point_color(1,p_idx))/255.0f,
                     float(pt_cld.point_color(2,p_idx))/255.0f,
                     1.0f};
        draw_points(v1, size);
    }
};