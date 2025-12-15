# Custom MJPC Tasks

This directory contains custom tasks for MuJoCo MPC (MJPC). Custom tasks allow you to define new control problems without modifying the core MJPC codebase.

## Directory Structure

Each custom task should follow this structure:
```
custom_tasks/
└── <task_name>/
    ├── CMakeLists.txt           # Build configuration
    ├── task/
    │   ├── task.h               # Task class declaration
    │   ├── task.cc              # Task residual implementation
    │   └── task.xml             # MJPC task definition (sensors, costs, keyframes)
    └── model/
        ├── scene_interactive.xml # Scene with robot + environment
        └── assets/              # Meshes, textures, etc.
```

## Creating a New Custom Task

### 1. Create Directory Structure

```bash
mkdir -p mjpc/tasks/custom_tasks/my_task/task
mkdir -p mjpc/tasks/custom_tasks/my_task/model
```

### 2. Implement Task Class (`task.h` and `task.cc`)

Your task class must inherit from `mjpc::Task` and implement the residual function:

```cpp
// task.h
#ifndef MJPC_TASKS_CUSTOM_TASKS_MY_TASK_TASK_H_
#define MJPC_TASKS_CUSTOM_TASKS_MY_TASK_TASK_H_

#include <string>
#include <mujoco/mujoco.h>
#include "mjpc/task.h"

namespace mjpc::custom_tasks::my_task {

class MyTask : public Task {
 public:
  std::string Name() const override;
  std::string XmlPath() const override;

  class ResidualFn : public mjpc::BaseResidualFn {
   public:
    explicit ResidualFn(const MyTask* task) : mjpc::BaseResidualFn(task) {}
    void Residual(const mjModel* model, const mjData* data,
                  double* residual) const override;
  };

  MyTask() : residual_(this) {}

 protected:
  std::unique_ptr<mjpc::ResidualFn> ResidualLocked() const override {
    return std::make_unique<ResidualFn>(this);
  }
  ResidualFn* InternalResidual() override { return &residual_; }

 private:
  ResidualFn residual_;
};

}  // namespace mjpc::custom_tasks::my_task

#endif  // MJPC_TASKS_CUSTOM_TASKS_MY_TASK_TASK_H_
```

```cpp
// task.cc
#include "mjpc/tasks/custom_tasks/my_task/task/task.h"
#include <mujoco/mujoco.h>
#include "mjpc/utilities.h"

namespace mjpc::custom_tasks::my_task {

std::string MyTask::Name() const { return "MyTask"; }

std::string MyTask::XmlPath() const {
  return GetModelPath("my_task/task/task.xml");
}

void MyTask::ResidualFn::Residual(const mjModel* model,
                                  const mjData* data,
                                  double* residual) const {
  int counter = 0;
  
  // Read sensors defined in task.xml and compute residuals
  double* my_sensor = SensorByName(model, data, "MySensor");
  mju_copy(&residual[counter], my_sensor, 3);
  counter += 3;
}

}  // namespace mjpc::custom_tasks::my_task
```

### 3. Create Task XML (`task.xml`)

Define sensors, costs, and keyframes:

```xml
<mujoco model="MyTask">
  <!-- Include MJPC common settings -->
  <include file="../../../common.xml"/>
  
  <!-- Include your scene -->
  <include file="../model/scene_interactive.xml"/>
  
  <custom>
    <!-- Planner settings -->
    <numeric name="agent_planner" data="1" />
    <numeric name="agent_horizon" data="0.6" />
    <numeric name="agent_timestep" data="0.01" />
    
    <!-- Sampling parameters -->
    <numeric name="sampling_trajectories" data="64"/>
    <numeric name="sampling_spline_points" data="8" />
    <numeric name="sampling_exploration" data="0.10" />
    <numeric name="gradient_spline_points" data="8" />
  </custom>
  
  <sensor>
    <!-- Cost terms (user sensors) -->
    <user name="MySensor" dim="3" user="2 1 0 5 0.01"/>
    
    <!-- Frame sensors for computing residuals -->
    <framepos name="my_pos" objtype="site" objname="my_site"/>
    <framequat name="my_quat" objtype="site" objname="my_site"/>
    
    <!-- Trace for visualization -->
    <framepos name="trace0" objtype="site" objname="my_site"/>
  </sensor>
  
  <keyframe>
    <key name="home" 
         qpos="0 0 0" 
         ctrl="0 0 0"
         mpos="0 0 0"
         mquat="1 0 0 0"/>
  </keyframe>
</mujoco>
```

### 4. Create CMakeLists.txt

```cmake
add_library(my_task OBJECT)

target_sources(
  my_task
  PUBLIC task/task.h
  PRIVATE task/task.cc
)

target_include_directories(
  my_task
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../../../..
)

target_link_libraries(
  my_task
  PRIVATE libmjpc
          mujoco::mujoco
)

# Copy model files to build directory
file(GLOB MY_TASK_MODEL_FILES "${CMAKE_CURRENT_SOURCE_DIR}/model/*")
file(COPY ${MY_TASK_MODEL_FILES}
     DESTINATION "${CMAKE_BINARY_DIR}/mjpc/tasks/custom_tasks/my_task/model/")

file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/task/task.xml"
     DESTINATION "${CMAKE_BINARY_DIR}/mjpc/tasks/custom_tasks/my_task/task/")
```

### 5. Register Task in MJPC

Edit `mjpc/tasks/tasks.cc`:

```cpp
// Add include at top
#include "mjpc/tasks/custom_tasks/my_task/task/task.h"

// Add to GetTasks() function
std::vector<std::shared_ptr<Task>> GetTasks() {
  return {
      // ... existing tasks ...
      std::make_shared<custom_tasks::my_task::MyTask>(),
  };
}
```

Edit `mjpc/tasks/CMakeLists.txt`:

```cmake
# Add near the end, before other task subdirectories or in alphabetical order
add_subdirectory(custom_tasks/my_task)
```

### 6. Build and Test

```bash
# Rebuild MJPC
cd build
ninja

# Test with binary
./bin/mjpc --task MyTask

# Test with Python
python -c "from mujoco_mpc import agent as agent_lib; a = agent_lib.Agent('MyTask', model=None)"
```

## Example: FrankaReach Task

See the `franka_reach/` directory for a complete working example of:
- Position and orientation tracking
- Torque-controlled robot arm
- Interactive mocap target
- Trajectory visualization with trace sensor

## Key Points

1. **Sensor Names**: Sensor names in `task.xml` must match names used in `SensorByName()` calls in C++
2. **Include Paths**: Use `../../../common.xml` from `task/task.xml` to include MJPC common settings
3. **GetModelPath**: Use `GetModelPath("my_task/task/task.xml")` - path is relative to build directory
4. **Python Integration**: Set `model=None` when creating Agent to use registered C++ task (avoids gRPC size limits)
5. **Cost Weights**: Use the exact sensor names from XML when calling `agent.set_cost_weights()`

## Troubleshooting

- **"Weight 'X' not found"**: Sensor name mismatch between XML and Python
- **"undefined reference to vtable"**: Task not added to CMakeLists.txt
- **XML parse errors**: Check include paths are correct relative to file location
- **Mesh not found**: Verify meshdir path and that assets are copied to build directory
