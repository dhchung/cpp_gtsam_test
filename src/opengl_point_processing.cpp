#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include "opengl_point_processing.h"


OpenglPointProcessing::OpenglPointProcessing(std::string window_name){
    w_name = window_name;
    screenWidth = 1600;
    screenHeight = 1200;

    camera = new Camera(glm::vec3(-2.0f, 0.0f, 1.0f), glm::vec3(0.0f,0.0f,-1.0f));

    lastX = (float)screenWidth/2.0f;
    lastY = (float)screenHeight/2.0f;
    firstMouse = true;

    deltaTime = 0.0f;
    lastFrame = 0.0f;
    

    imgs.clear();
}

OpenglPointProcessing::~OpenglPointProcessing(){

}

void OpenglPointProcessing::init_opengl(){
    glfwInit(); //Initialize GLFW
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR,3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR,3); //OpenGL 3.3
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); //Use only core profiles of opengl


    window = glfwCreateWindow(screenWidth,screenHeight, w_name.c_str(), NULL, NULL);

    glfwSetWindowUserPointer(window, this);


    if(window == NULL) {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
    }
    

    glfwMakeContextCurrent(window); //Tell GLFW to setup "window context" as primary context for current thread

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) { //glfwGetProcAddress : Get correct functions regarding to OS or compile environment
        std::cout<< "Failed to initialize GLAD" << std::endl;
    }

    shader.InitShader("shaders/vertex_shader.vs", "shaders/fragment_shader.fs");

    // glDisable(GL_DEPTH_TEST);
    // glEnable(GL_BLEND);
    // glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	// GLuint l_mvpMatrixId = glGetUniformLocation(l_programId, "MVP");
    // glUseProgram(l_programId);

    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);
    glGenVertexArrays(1, &VAO);
    glEnable(GL_DEPTH_TEST);  

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


    int data_size = pt_cld.point_cloud.cols();

    float vertices[6*data_size];
   
    for(int i = 0; i<data_size; ++i){
        for(int j = 0; j<3; ++j){
            vertices[i*6+j] = pt_cld.point_cloud(j,i)/1000.0f;
        }
        for(int j = 3; j<6; ++j){
            vertices[i*6+j] = pt_cld.point_color(j-3,i)/255.0f;
        }
    }

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);


    //Model and camera;

    glm::mat4 model = glm::mat4(1.0f);
    glm::mat4 view = glm::mat4(1.0f);
    glm::mat4 projection = glm::mat4(1.0f);

    projection = glm::perspective(glm::radians(45.0f),
                                    float(screenWidth)/float(screenHeight), 
                                    0.1f, 
                                    100.0f);

    view = glm::lookAt(glm::vec3(-1.0f, -1.0f, -1.0f),
                       glm::vec3(1.0f, 0.0f, 0.0f),
                       glm::vec3(0.0f, 0.0f, -1.0f));

    clear_window();
    shader.use();
    shader.setMat4("model", model);
    shader.setMat4("view", view);
    shader.setMat4("projection", projection);
    
    glBindVertexArray(VAO);
    glPointSize(3.0);
    glDrawArrays(GL_POINTS, 0, data_size);
    glBindVertexArray(0);

    glfwSwapBuffers(window);
    glfwPollEvents();
}

glm::mat4 OpenglPointProcessing::eigen_mat4_to_glm_mat4(Eigen::Matrix4f & e_mat4){

    glm::mat4 result;
    for(int i=0;i<4;++i){
        for(int j=0;j<4;++j){
            result[j][i] = e_mat4(i,j);
        }
    }
    return result;
}


void OpenglPointProcessing::plot_global_points(std::vector<PointCloud> & g_pt_cld,
                                               std::vector<float> & state,
                                               int & idx){

    clear_window();

    glm::mat4 model = glm::mat4(1.0f);
    glm::mat4 view = glm::mat4(1.0f);
    glm::mat4 projection = glm::mat4(1.0f);


    view = glm::lookAt(glm::vec3(state[0]-2, state[1]+2, 1),
                    glm::vec3(state[0]+1, state[1], state[2]),
                    glm::vec3(0.0f, 0.0f, -1.0f));


    for(int data_id = 0; data_id<g_pt_cld.size(); ++data_id){

        int data_size = g_pt_cld[data_id].point_cloud.cols();

        float vertices[6*data_size];
    
        for(int i = 0; i<data_size; ++i){
            for(int j = 0; j<3; ++j){
                vertices[i*6+j] = g_pt_cld[data_id].point_cloud(j,i)/1000.0f;
            }
            for(int j = 3; j<6; ++j){
                vertices[i*6+j] = g_pt_cld[data_id].point_color(j-3,i)/255.0f;
            }
        }

        glBindVertexArray(VAO);
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);

        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));
        glEnableVertexAttribArray(1);

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);


        //Model and camera;


        Eigen::Matrix4f cur_state;
        
        c_trans.xyzrpy2t(g_pt_cld[data_id].state, &cur_state);

        model = eigen_mat4_to_glm_mat4(cur_state);

        projection = glm::perspective(glm::radians(45.0f),
                                        float(screenWidth)/float(screenHeight), 
                                        0.1f, 
                                        100.0f);



        shader.use();
        shader.setMat4("model", model);
        shader.setMat4("view", view);
        shader.setMat4("projection", projection);
        
        glBindVertexArray(VAO);
        glPointSize(3.0);
        glDrawArrays(GL_POINTS, 0, data_size);
        glBindVertexArray(0);

    }

    glfwSwapBuffers(window);
    glfwPollEvents();        

    // GLubyte * pixels = new GLubyte[3*screenWidth*screenHeight];
    // glReadPixels(0, 0, screenWidth, screenHeight, GL_RGB, GL_UNSIGNED_BYTE, pixels);
    // std::string image_title = "image/img"+std::to_string(idx)+".png";
    // stbi_flip_vertically_on_write(true);
    // stbi_write_png(image_title.c_str(), screenWidth, screenHeight, 3, pixels, screenWidth*3);
    // delete[] pixels;

}

void OpenglPointProcessing::draw_point_global(std::vector<PointCloud> & g_pt_cld, GLfloat size){


    glfwSetCursorPosCallback(window, mouse_callback);
    glfwSetScrollCallback(window, scroll_callback);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    while(!glfwWindowShouldClose(window)){
        processInput_end();
        glClearColor(28.0/255.0, 40.0/255.0, 79.0/255.0, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);  

        glm::mat4 model = glm::mat4(1.0f);
        glm::mat4 view = glm::mat4(1.0f);
        glm::mat4 projection = glm::mat4(1.0f);

        shader.use();
        projection = glm::perspective(glm::radians(camera->Zoom), (float)screenWidth/2/(float)screenHeight, 0.1f, 100.0f);
        view = camera->GetViewMatrix();



        for(int data_id = 0; data_id<g_pt_cld.size(); ++data_id){

            int data_size = g_pt_cld[data_id].point_cloud.cols();

            float vertices[6*data_size];
        
            for(int i = 0; i<data_size; ++i){
                for(int j = 0; j<3; ++j){
                    vertices[i*6+j] = g_pt_cld[data_id].point_cloud(j,i)/1000.0f;
                }
                for(int j = 3; j<6; ++j){
                    vertices[i*6+j] = g_pt_cld[data_id].point_color(j-3,i)/255.0f;
                }
            }

            glBindVertexArray(VAO);
            glBindBuffer(GL_ARRAY_BUFFER, VBO);
            glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)0);
            glEnableVertexAttribArray(0);

            glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));
            glEnableVertexAttribArray(1);

            glBindBuffer(GL_ARRAY_BUFFER, 0);
            glBindVertexArray(0);


            //Model and camera;


            Eigen::Matrix4f cur_state;
            
            c_trans.xyzrpy2t(g_pt_cld[data_id].state, &cur_state);

            model = eigen_mat4_to_glm_mat4(cur_state);

            projection = glm::perspective(glm::radians(45.0f),
                                            float(screenWidth)/float(screenHeight), 
                                            0.1f, 
                                            100.0f);

            shader.setMat4("model", model);
            shader.setMat4("view", view);
            shader.setMat4("projection", projection);
            
            glBindVertexArray(VAO);
            glPointSize(3.0);
            glDrawArrays(GL_POINTS, 0, data_size);
            glBindVertexArray(0);

        }

        glfwSwapBuffers(window);
        glfwPollEvents();        
    }
}

void OpenglPointProcessing::terminate(){
    glfwTerminate();
}

void OpenglPointProcessing::clear_window(){
    processInput();
    glClearColor(28.0/255.0, 40.0/255.0, 79.0/255.0, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);  
}


void OpenglPointProcessing::draw_axis(float line_length, float line_width){
    // glLineWidth(line_width);
    // glBegin(GL_LINES);
    // //x
    // glColor4f(1.0f,0.0f,0.0f,1.0f);
    // glVertex3f(0.0f, 0.0f, 0.0f);
    // glVertex3f(line_length, 0.0f, 0.0f);
    // //y
    // glColor4f(0.0f,1.0f,0.0f,1.0f);
    // glVertex3f(0.0f, 0.0f, 0.0f);
    // glVertex3f(0.0f, line_length, 0.0f);
    // //z
    // glColor4f(0.0f,0.0f,1.0f,1.0f);
    // glVertex3f(0.0f, 0.0f, 0.0f);
    // glVertex3f(0.0f, 0.0f, line_length);
    // glEnd();
}

void OpenglPointProcessing::processInput_end(){
    if(glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, true);
    }

    float currentFrame = glfwGetTime();
    deltaTime = currentFrame - lastFrame;
    lastFrame = currentFrame;

    if(glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS) {
        camera->MovementSpeed = camera->FastSpeed;
    } else {
        camera->MovementSpeed = camera->OriginalSpeed;
    }

    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) {
        camera->ProcessKeyboard(FORWARD, deltaTime);
    }
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS){
        camera->ProcessKeyboard(BACKWARD, deltaTime);
    }    
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) {
        camera->ProcessKeyboard(LEFT, deltaTime);
    }
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) {
        camera->ProcessKeyboard(RIGHT, deltaTime);
    }

}

void OpenglPointProcessing::mouse_callback(GLFWwindow * window, double xpos, double ypos) {

    OpenglPointProcessing *ogl_pointer =
         static_cast<OpenglPointProcessing*>(glfwGetWindowUserPointer(window));
    ogl_pointer->mouse_callback_function(xpos, ypos);

}

void OpenglPointProcessing::scroll_callback(GLFWwindow * window, double xoffset, double yoffset) {
    OpenglPointProcessing *ogl_pointer =
         static_cast<OpenglPointProcessing*>(glfwGetWindowUserPointer(window));
    ogl_pointer->scroll_callback_function(xoffset, yoffset);
}

void OpenglPointProcessing::mouse_callback_function(double xpos, double ypos) {
    if(firstMouse) {
        lastX = xpos;
        lastY = ypos;
        firstMouse = false;
    }

    float xoffset = xpos-lastX;
    float yoffset = lastY - ypos;
    lastX = xpos;
    lastY = ypos;

    camera->ProcessMouseMovement(xoffset, yoffset);
}

void OpenglPointProcessing::scroll_callback_function(double xoffset, double yoffset){
    camera->ProcessMouseScroll(yoffset);
}


void OpenglPointProcessing::insertImages(cv::Mat & img){
    imgs.push_back(img);
    std::cout<<"Image is pushed back : "<<static_cast<int>(imgs.size())<<"imgs"<<std::endl;
}


void OpenglPointProcessing::draw_plane_global(gtsam::Values & results){

    glfwSetCursorPosCallback(window, mouse_callback);
    glfwSetScrollCallback(window, scroll_callback);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    while(!glfwWindowShouldClose(window)){
        processInput_end();
        glClearColor(28.0/255.0, 40.0/255.0, 79.0/255.0, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);  

        glm::mat4 model = glm::mat4(1.0f);
        glm::mat4 view = glm::mat4(1.0f);
        glm::mat4 projection = glm::mat4(1.0f);

        shader.use();
        projection = glm::perspective(glm::radians(camera->Zoom), (float)screenWidth/2/(float)screenHeight, 0.1f, 100.0f);
        view = camera->GetViewMatrix();



        for(int data_id = 0; results.size(); ++data_id){

            gtsamexample::StatePlane state = results.at<gtsamexample::StatePlane>(data_id);

            Eigen::Vector3f line_0(focal_length, -img_center_x, -img_center_y);
            Eigen::Vector3f line_1(focal_length, img_center_x, -img_center_y);
            Eigen::Vector3f line_2(focal_length, -img_center_x, img_center_y);
            Eigen::Vector3f line_3(focal_length, img_center_x, img_center_y);
            Eigen::Vector4f plane_param = Eigen::Vector4f(state.nx, state.ny, state.nz, state.d);

            Eigen::Vector3f loc_0 = line_0 * (-state.d/(line_0.transpose()*plane_param.segment(0,2)));
            Eigen::Vector3f loc_1 = line_1 * (-state.d/(line_1.transpose()*plane_param.segment(0,2)));
            Eigen::Vector3f loc_2 = line_2 * (-state.d/(line_2.transpose()*plane_param.segment(0,2)));
            Eigen::Vector3f loc_3 = line_3 * (-state.d/(line_3.transpose()*plane_param.segment(0,2)));

            int height = imgs[data_id].rows;
            int width = imgs[data_id].cols;
            cv::Mat img;
            cv::cvtColor(imgs[data_id], img, cv::COLOR_BGR2RGB);
            cv::flip(img, img, -1);
            unsigned char* image = img.data;

            float vertices[] = {
                //Position                              //TexCoord
                loc_0(0),   loc_0(1),   loc_0(2),       0.0f,   1.0f,
                loc_1(0),   loc_1(1),   loc_1(2),       1.0f,   1.0f,
                loc_2(0),   loc_2(1),   loc_2(2),       0.0f,   0.0f,
                loc_3(0),   loc_3(1),   loc_3(2),       1.0f,   0.0f
            };

            unsigned int indices[] = {
                0,  1,  2,
                1,  2,  3
            };

            glBindVertexArray(VAO);
            glBindBuffer(GL_ARRAY_BUFFER, VBO);
            glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), &indices, GL_STATIC_DRAW);

            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5*sizeof(float), (void*)0);
            glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5*sizeof(float), (void*)(3*sizeof(float)));
            glEnableVertexAttribArray(0);
            glEnableVertexAttribArray(1);

            unsigned int texture;
            glGenTextures(1, &texture);
            glBindTexture(GL_TEXTURE_2D, texture);

            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

            if(image){
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, image);
            }


            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)0);
            glEnableVertexAttribArray(0);

            glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));
            glEnableVertexAttribArray(1);

            glBindBuffer(GL_ARRAY_BUFFER, 0);
            glBindVertexArray(0);


            //Model and camera;


            Eigen::Matrix4f cur_state;
            
            c_trans.xyzrpy2t(g_pt_cld[data_id].state, &cur_state);

            model = eigen_mat4_to_glm_mat4(cur_state);

            projection = glm::perspective(glm::radians(45.0f),
                                            float(screenWidth)/float(screenHeight), 
                                            0.1f, 
                                            100.0f);

            shader.setMat4("model", model);
            shader.setMat4("view", view);
            shader.setMat4("projection", projection);
            
            glBindVertexArray(VAO);
            glPointSize(3.0);
            glDrawArrays(GL_POINTS, 0, data_size);
            glBindVertexArray(0);

        }

        glfwSwapBuffers(window);
        glfwPollEvents();        
    }
}