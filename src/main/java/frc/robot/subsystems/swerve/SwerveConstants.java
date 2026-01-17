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
