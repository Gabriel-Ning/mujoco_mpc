# Franka Reach MJPC Task
add_library(mjpc_franka_reach_task STATIC
  task/task.cc
)

target_link_libraries(mjpc_franka_reach_task
  mjpc_task
  mujoco::mujoco
)

target_include_directories(mjpc_franka_reach_task
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${MJPC_SOURCE_DIR}
)
