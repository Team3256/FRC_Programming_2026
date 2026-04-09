// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.robot.subsystems.swerve.generated.TunerConstants;

public final class SwerveConstants {
  // LinearVelocity is a vector, so we need to get the magnitude
  public static final double deadbandMultiplier = 0.15;
  public static final double MaxSpeed = TunerConstants.kSpeedAt12Volts.magnitude();
  public static final double MaxAngularRate = 1.5 * Math.PI;
  public static final double SlowMaxSpeed = MaxSpeed * 0.3;
  public static final double SlowMaxAngular = MaxAngularRate * 0.3;

  public static final double SuperSlowMaxSpeed = MaxSpeed * 0.15;


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

  // TODO: adjust each for overshoot
  public static final class AzimuthTargets {

    // TODO: tune these jawns
    public static final double aziKP = 6.0;
    public static final double aziKi = 0.0;
    public static final double aziKD = 0.0;
    public static final double timeout = 0.3;
    public static final Rotation2d bump = new Rotation2d(Math.toRadians(45));
  }

  public static final class BumpTargets {

    public static Pose2d BOTTOM_BUMP =
        new Pose2d(
            3.3690896034240723, 2.46048903465271, Rotation2d.fromDegrees(45.744063302312063968));

    public static Pose2d TOP_BUMP =
        new Pose2d(
            3.4050545692443848, 5.535476207733154, Rotation2d.fromDegrees(45.744063302312063968));
  }
}
