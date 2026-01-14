// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.robot.subsystems.swerve.generated.TunerConstants;

public final class SwerveConstants {
  // LinearVelocity is a vector, so we need to get the magnitude
  public static final double MaxSpeed = TunerConstants.kSpeedAt12Volts.magnitude();
  public static final double MaxAngularRate = 1.5 * Math.PI;
  public static final double SlowMaxSpeed = MaxSpeed * 0.3;
  public static final double SlowMaxAngular = MaxAngularRate * 0.3;

  // source 1, 5 degree overshoot for weight drag from elevator
  // overshoot if weight concentrated = (0.0872665)
  public static final Rotation2d sourceLeft1 = new Rotation2d(0.559 + (3 * 0.0872665));
  public static final Rotation2d sourceRight2 = new Rotation2d(2.46091);

  // climb and processor - close preset doubles as climb target facing cage, same for far presets
  public static final Rotation2d processorClose = new Rotation2d(0);
  public static final Rotation2d processorFar = new Rotation2d(Math.PI);

  // barge targets
  public static final Rotation2d bargeClose = new Rotation2d((Math.PI / 2));
  public static final Rotation2d bargeFar = new Rotation2d(((Math.PI / 2) + Math.PI));

  // time to force stop the command so we can regain control of rotation
  public static final double aziTimeout = 1.2;

  // Physics constants
  public static final Mass robotMass = Pounds.of(120);
  public static final MomentOfInertia robotMOI = KilogramSquareMeters.of(36);

  // Module Locations
  public static final Translation2d frontLeft =
      new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY);
  public static final Translation2d frontRight =
      new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY);
  public static final Translation2d backLeft =
      new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY);
  public static final Translation2d backRight =
      new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY);
}
