// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.sotm;

import static frc.robot.subsystems.shooterpivot.ShooterPivotConstants.robotToTurret;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.FieldConstants;
import frc.robot.subsystems.shooterpivot.ShooterPivotConstants.*;
import frc.robot.utils.AllianceFlip;

public class ShotCalculator {
  private static ShotCalculator instance;

  private final LinearFilter turretAngleFilter =
      LinearFilter.movingAverage((int) (0.1 / 0.02)); // loop period in seconds
  private final LinearFilter hoodAngleFilter = LinearFilter.movingAverage((int) (0.1 / 0.02));

  private Rotation2d lastTurretAngle;
  private double lastHoodAngle;
  private Rotation2d turretAngle;
  private double hoodAngle = Double.NaN;
  private double turretVelocity;
  private double hoodVelocity;

  public static ShotCalculator getInstance() {
    if (instance == null) instance = new ShotCalculator();
    return instance;
  }

  public record ShootingParameters(
      boolean isValid, Rotation2d turretAngle, double turretVelocity, double flywheelSpeed) {}

  // Cache parameters
  private ShootingParameters latestParameters = null;

  private static double minDistance;
  private static double maxDistance;
  private static double phaseDelay;
  private static final InterpolatingTreeMap<Double, Rotation2d> shotHoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap shotFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  static {
    minDistance = 1.34;
    maxDistance = 5.60;
    phaseDelay = 0.03;

    shotHoodAngleMap.put(1.34, Rotation2d.fromDegrees(19.0));
    shotHoodAngleMap.put(1.78, Rotation2d.fromDegrees(19.0));
    shotHoodAngleMap.put(2.17, Rotation2d.fromDegrees(24.0));
    shotHoodAngleMap.put(2.81, Rotation2d.fromDegrees(27.0));
    shotHoodAngleMap.put(3.82, Rotation2d.fromDegrees(29.0));
    shotHoodAngleMap.put(4.09, Rotation2d.fromDegrees(30.0));
    shotHoodAngleMap.put(4.40, Rotation2d.fromDegrees(31.0));
    shotHoodAngleMap.put(4.77, Rotation2d.fromDegrees(32.0));
    shotHoodAngleMap.put(5.57, Rotation2d.fromDegrees(32.0));
    shotHoodAngleMap.put(5.60, Rotation2d.fromDegrees(35.0));

    shotFlywheelSpeedMap.put(1.34, 210.0);
    shotFlywheelSpeedMap.put(1.78, 220.0);
    shotFlywheelSpeedMap.put(2.17, 220.0);
    shotFlywheelSpeedMap.put(2.81, 230.0);
    shotFlywheelSpeedMap.put(3.82, 250.0);
    shotFlywheelSpeedMap.put(4.09, 255.0);
    shotFlywheelSpeedMap.put(4.40, 260.0);
    shotFlywheelSpeedMap.put(4.77, 265.0);
    shotFlywheelSpeedMap.put(5.57, 275.0);
    shotFlywheelSpeedMap.put(5.60, 290.0);

    timeOfFlightMap.put(5.68, 1.16);
    timeOfFlightMap.put(4.55, 1.12);
    timeOfFlightMap.put(3.15, 1.11);
    timeOfFlightMap.put(1.88, 1.09);
    timeOfFlightMap.put(1.38, 0.90);
  }

  public ShootingParameters getParameters() {

    // Provide pose data if it exists
    if (latestParameters != null) {
      return latestParameters;
    }

    // Calculate estimated pose while accounting for phase delay
    Pose2d estimatedPose = Pose2d.kZero;
    ChassisSpeeds robotRelativeVelocity = new ChassisSpeeds();
    estimatedPose =
        estimatedPose.exp(
            new Twist2d(
                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

    // Calculate distance from turret to target
    Translation2d target = AllianceFlip.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
    Pose2d turretPosition =
        estimatedPose.transformBy(
            new Transform2d(
                robotToTurret.getTranslation().toTranslation2d(),
                robotToTurret.getRotation().toRotation2d()));
    double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());

    // Calculate field relative turret velocity
    ChassisSpeeds robotVelocity =
        ChassisSpeeds.fromRobotRelativeSpeeds(new ChassisSpeeds(), Pose2d.kZero.getRotation());
    double robotAngle = estimatedPose.getRotation().getRadians();
    double turretVelocityX =
        robotVelocity.vxMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
                * (robotToTurret.getY() * Math.cos(robotAngle)
                    - robotToTurret.getX() * Math.sin(robotAngle));
    double turretVelocityY =
        robotVelocity.vyMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
                * (robotToTurret.getX() * Math.cos(robotAngle)
                    - robotToTurret.getY() * Math.sin(robotAngle));

    // Account for imparted velocity by robot (turret) to offset
    double timeOfFlight;
    Pose2d lookaheadPose = turretPosition;
    double lookaheadTurretToTargetDistance = turretToTargetDistance;
    for (int i = 0; i < 20; i++) {
      timeOfFlight = timeOfFlightMap.get(lookaheadTurretToTargetDistance);
      double offsetX = turretVelocityX * timeOfFlight;
      double offsetY = turretVelocityY * timeOfFlight;
      lookaheadPose =
          new Pose2d(
              turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              turretPosition.getRotation());
      lookaheadTurretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
    }

    // Calculate parameters accounted for imparted velocity
    turretAngle = target.minus(lookaheadPose.getTranslation()).getAngle();
    hoodAngle = shotHoodAngleMap.get(lookaheadTurretToTargetDistance).getRadians();
    if (lastTurretAngle == null) lastTurretAngle = turretAngle;
    if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;
    turretVelocity =
        turretAngleFilter.calculate(turretAngle.minus(lastTurretAngle).getRadians() / 0.02);
    hoodVelocity = hoodAngleFilter.calculate((hoodAngle - lastHoodAngle) / 0.02);
    lastTurretAngle = turretAngle;
    lastHoodAngle = hoodAngle;
    latestParameters =
        new ShootingParameters(
            lookaheadTurretToTargetDistance >= minDistance
                && lookaheadTurretToTargetDistance <= maxDistance,
            turretAngle,
            turretVelocity,
            shotFlywheelSpeedMap.get(lookaheadTurretToTargetDistance));

    return latestParameters;
  }

  public void clearShootingParameters() {
    latestParameters = null;
  }
}
