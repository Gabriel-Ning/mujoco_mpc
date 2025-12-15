// Copyright 2024 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "mjpc/tasks/custom_tasks/franka_reach/task/task.h"

#include <string>
#include <mujoco/mujoco.h>
#include "mjpc/task.h"
#include "mjpc/utilities.h"

namespace mjpc {

std::string FrankaReach::Name() const { 
  return "FrankaReach"; 
}

std::string FrankaReach::XmlPath() const {
  return GetModelPath("custom_tasks/franka_reach/task/task.xml");
}

// ---------- Residuals for Franka reach task ---------
//   Number of residuals: 3
//     Residual (0): end-effector position - target position
//     Residual (1): end-effector orientation - target orientation
//     Residual (2): control effort
// ----------------------------------------------------
void FrankaReach::ResidualFn::Residual(const mjModel* model, 
                                       const mjData* data,
                                       double* residual) const {
  int counter = 0;

  // ---------- Residual (0): Position tracking ----------
  double* eef_pos = SensorByName(model, data, "eef_pos");
  double* target_pos = SensorByName(model, data, "target_pos");
  mju_sub3(residual + counter, eef_pos, target_pos);
  counter += 3;

  // ---------- Residual (1): Orientation tracking ----------
  double* eef_quat = SensorByName(model, data, "eef_quat");
  double* target_quat = SensorByName(model, data, "target_quat");
  
  // Normalize quaternions to be safe
  mju_normalize4(eef_quat);
  mju_normalize4(target_quat);
  
  // Compute orientation error
  mju_subQuat(residual + counter, target_quat, eef_quat);
  counter += 3;

  // ---------- Residual (2): Control effort ----------
  mju_copy(residual + counter, data->ctrl, model->nu);
  counter += model->nu;

  // Sensor dimension sanity check
  int user_sensor_dim = 0;
  for (int i = 0; i < model->nsensor; i++) {
    if (model->sensor_type[i] == mjSENS_USER) {
      user_sensor_dim += model->sensor_dim[i];
    }
  }
  if (user_sensor_dim != counter) {
    mju_error_i(
        "mismatch between total user-sensor dimension "
        "and actual length of residual %d",
        counter);
  }
}

}  // namespace mjpc
