// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.sotm;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public final class ShotCalculatorConstants {

  public static final Transform3d ROBOT_TO_SHOOTER =
      new Transform3d(new Translation3d(-0.24, 0.0, 0.5), Rotation3d.kZero);

  // distance to shooter speed
  // meters to RPS
  public static final InterpolatingDoubleTreeMap DISTANCE_TO_SHOOTER_SPEED =
      new InterpolatingDoubleTreeMap();

  // distance to pivot angle
  // meters to radians
  public static final InterpolatingDoubleTreeMap DISTANCE_TO_PIVOT_ANGLE =
      new InterpolatingDoubleTreeMap();

  static {

    // distance to shooter speed
    // meters to RPS
    DISTANCE_TO_SHOOTER_SPEED.put(2.07, 7.0);
    DISTANCE_TO_SHOOTER_SPEED.put(4.92, 9.0);


    // distance to pivot angle
    // meters to radians
    DISTANCE_TO_PIVOT_ANGLE.put(1.0, Math.toRadians(45));
    DISTANCE_TO_PIVOT_ANGLE.put(2.0, Math.toRadians(45));
    DISTANCE_TO_PIVOT_ANGLE.put(3.0, Math.toRadians(45));
    DISTANCE_TO_PIVOT_ANGLE.put(4.0, Math.toRadians(45));
  }

}
