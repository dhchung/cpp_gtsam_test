#include "opengl_point_processing.h"


OpenglPointProcessing::OpenglPointProcessing(std::string window_name){
    w_name = window_name;
}
OpenglPointProcessing::~OpenglPointProcessing(){

}

void OpenglPointProcessing::init_opengl(){
    glfwInit(); //Initialize GLFW
    glfwWindowHint(GLFW_SAMPLES, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR,3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR,3); //OpenGL 3.3
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); //Use only core profiles of opengl


    window = glfwCreateWindow(640,480, w_name.c_str(), NULL, NULL);
    if(window == NULL) {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
    }
    
    glfwMakeContextCurrent(window); //Tell GLFW to setup "window context" as primary context for current thread

    // glewExperimental = true;

    Shader ourShader("shaders/pointcloud.vert", "shaders/pointcloud.frag");

    // GLuint l_programId = LoadShaders("shaders/pointcloud.vert", "shaders/pointcloud.frag");

    // glDisable(GL_DEPTH_TEST);
    // glEnable(GL_BLEND);
    // glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	// GLuint l_mvpMatrixId = glGetUniformLocation(l_programId, "MVP");
    // glUseProgram(l_programId);

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
    // glClear(GL_COLOR_BUFFER_BIT);
    clear_window();


    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-ratio, ratio, -1.f, 1.f, 10.f, -10.f);

    gluLookAt(-1.0, -1.0, -1.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    draw_point_3d(pt_cld, 3.0f);
    draw_axis(0.3, 2);
    
    glfwSwapBuffers(window);
    glfwPollEvents();   
}



void OpenglPointProcessing::plot_global_points(std::vector<PointCloud> & g_pt_cld){
    float ratio;
    int width, height;
    glfwGetFramebufferSize(window, &width, &height);
    ratio = (float) width / (float) height;
    glViewport(0, 0, width, height);
    // glClear(GL_COLOR_BUFFER_BIT);
    clear_window();


    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-ratio, ratio, -1.f, 1.f, 10.f, -10.f);
    gluLookAt(-1.0, -1.0, -1.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0);


    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();



    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    draw_point_global(g_pt_cld, 3.0f);
    draw_axis(0.3, 2);
    
    glfwSwapBuffers(window);
    glfwPollEvents();   
}

void OpenglPointProcessing::draw_point_global(std::vector<PointCloud> & global_cloud, GLfloat size){

}

void OpenglPointProcessing::terminate(){
    glfwTerminate();
}

void OpenglPointProcessing::clear_window(){
    processInput();
    glClearColor(28.0/255.0, 40.0/255.0, 79.0/255.0, 1.0f);
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

void OpenglPointProcessing::draw_axis(float line_length, float line_width){
    glLineWidth(line_width);
    glBegin(GL_LINES);
    //x
    glColor4f(1.0f,0.0f,0.0f,1.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(line_length, 0.0f, 0.0f);
    //y
    glColor4f(0.0f,1.0f,0.0f,1.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, line_length, 0.0f);
    //z
    glColor4f(0.0f,0.0f,1.0f,1.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, line_length);
    glEnd();
}