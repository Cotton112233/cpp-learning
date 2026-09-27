#include "viewer.hpp"

#include <GLFW/glfw3.h>

#include <iostream>

Viewer::Viewer(const mjModel* model)
{
    if (glfwInit() != GLFW_TRUE)
    {
        std::cout << "Failed to initialize GLFW.\n";
        return;
    }

    glfw_initialized_ = true;
    window_ = glfwCreateWindow(1000, 700, "MuJoCo 2-DOF", nullptr, nullptr);

    if (window_ == nullptr)
    {
        std::cout << "Failed to create GLFW window.\n";
        return;
    }

    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);

    mjv_defaultCamera(&camera_);
    mjv_defaultOption(&option_);
    mjv_defaultScene(&scene_);
    mjr_defaultContext(&context_);

    mjv_makeScene(model, &scene_, 1000);
    mjr_makeContext(model, &context_, mjFONTSCALE_150);

    camera_.lookat[0] = 0.0;
    camera_.lookat[1] = 0.0;
    camera_.lookat[2] = 1.0;
    camera_.distance = 2.8;
    camera_.azimuth = 135.0;
    camera_.elevation = -20.0;
}

Viewer::~Viewer()
{
    if (window_ != nullptr)
    {
        mjr_freeContext(&context_);
        mjv_freeScene(&scene_);
        glfwDestroyWindow(window_);
    }

    if (glfw_initialized_)
    {
        glfwTerminate();
    }
}

bool Viewer::isReady() const
{
    return window_ != nullptr;
}

bool Viewer::isOpen() const
{
    return window_ != nullptr && glfwWindowShouldClose(window_) == GLFW_FALSE;
}

void Viewer::render(const mjModel* model, mjData* data)
{
    int width = 0;
    int height = 0;
    glfwGetFramebufferSize(window_, &width, &height);

    const mjrRect viewport{0, 0, width, height};

    mjv_updateScene(
        model,
        data,
        &option_,
        nullptr,
        &camera_,
        mjCAT_ALL,
        &scene_
    );

    mjr_render(viewport, &scene_, &context_);
    glfwSwapBuffers(window_);
    glfwPollEvents();

    if (glfwGetKey(window_, GLFW_KEY_ESCAPE) == GLFW_PRESS)
    {
        glfwSetWindowShouldClose(window_, GLFW_TRUE);
    }
}
