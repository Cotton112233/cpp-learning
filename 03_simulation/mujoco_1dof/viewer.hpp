#pragma once

#include <mujoco/mujoco.h>

struct GLFWwindow;

class Viewer
{
public:
    explicit Viewer(const mjModel* model);
    ~Viewer();

    Viewer(const Viewer&) = delete;
    Viewer& operator=(const Viewer&) = delete;

    bool isReady() const;
    bool isOpen() const;
    void render(const mjModel* model, mjData* data);

private:
    GLFWwindow* window_ = nullptr;
    mjvCamera camera_{};
    mjvOption option_{};
    mjvScene scene_{};
    mjrContext context_{};
    bool glfw_initialized_ = false;
};
