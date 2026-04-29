// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.utils;

import choreo.auto.AutoRoutine;
import java.util.List;
import java.util.function.Supplier;

public class AutoConfig {
  public final String name;
  public final Supplier<AutoRoutine> routine;
  public final List<String> trajectories;

  public AutoConfig(String name, Supplier<AutoRoutine> routine, List<String> trajectories) {
    this.name = name;
    this.routine = routine;
    this.trajectories = trajectories;
  }
}
