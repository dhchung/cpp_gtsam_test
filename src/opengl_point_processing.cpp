#include "opengl_point_processing.h"


OpenglPointProcessing::OpenglPointProcessing(std::string window_name){
    w_name = window_name;
}
OpenglPointProcessing::~OpenglPointProcessing(){

}

void OpenglPointProcessing::init_opengl(){
    glfwInit(); //Initialize GLFW
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR,3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR,3); //OpenGL 3.3
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); //Use only core profiles of opengl
    //Setting up GLFW : https://www.glfw.org/docs/latest/window.html#window_hints

    //Making window
    window = glfwCreateWindow(800,600, w_name.c_str(), NULL, NULL);
    if(window == NULL) {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
    }
    
    glfwMakeContextCurrent(window); //Tell GLFW to setup "window context" as primary context for current thread

    //GLAD : take care of the OpenGL function pointer : Need to initialize it before calling OpenGL functions
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) { //glfwGetProcAddress : Get correct functions regarding to OS or compile environment
        std::cout<< "Failed to initialize GLAD" << std::endl;
    }

    glViewport(0,0,800,600); // Setup the viewport size (location of the viewport in the window, width and height of the window)
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
}

void OpenglPointProcessing::framebuffer_size_callback(GLFWwindow* window, int width, int height) { // Change viewport if the user change the size of the window
    glViewport(0, 0, width, height);
}

void OpenglPointProcessing::processInput() {
    if(glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, true);
    }
}

void OpenglPointProcessing::plot_3d_points(PointCloud * pt_cld){
    clear_window();

    glColor3f(1,0,0);                        // 정점의 색은 빨간색
    glBegin(GL_POINTS);                // 점만 찍어낸다.
    glVertex2f(0.0, 0.5);
    glVertex2f(-0.5, -0.5);
    glVertex2f(0.5, -0.5);
    glEnd();
    glFlush();    

    glfwSwapBuffers(window); //e.g. double buffer -> Swap the color buffer
    glfwPollEvents(); // Check any events(keyboard or mouse), update the window status, and callback the functions  

}



void OpenglPointProcessing::plot_global_points(std::vector<PointCloud> * g_pt_cld){
    
}

void OpenglPointProcessing::terminate(){
    glfwTerminate();
}

void OpenglPointProcessing::clear_window(){
    processInput();
    glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
}

