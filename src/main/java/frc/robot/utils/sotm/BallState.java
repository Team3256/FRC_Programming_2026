// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.utils.sotm;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;

public class BallState {
  public Pose3d pose;
  public Translation3d velocity;
  public Translation3d omega; // rad/s

  public BallState(Pose3d position, Translation3d velocity, Translation3d omega) {
    this.pose = position;
    this.velocity = velocity;
    this.omega = omega;
  }
}
