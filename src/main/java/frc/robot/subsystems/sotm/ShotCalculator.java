// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.sotm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.FieldConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ShotCalculator {

  private static double phaseDelay;
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();
  private final Supplier<Pose2d> robotPoseSupplier;
  private final Supplier<ChassisSpeeds> robotVelocitySupplier;
  private final Transform2d robotToTurret;

  private Pose2d lookaheadPose;

  private Translation2d target = FieldConstants.Hub.topCenterPoint.toTranslation2d();

  public ShotCalculator(
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<ChassisSpeeds> robotVelocitySupplier,
      Transform2d robotToTurret) {
    this.robotPoseSupplier = robotPoseSupplier;
    this.robotVelocitySupplier = robotVelocitySupplier;
    this.robotToTurret = robotToTurret;
    phaseDelay = 0.03;
    timeOfFlightMap.put(5.68, 1.16);
    timeOfFlightMap.put(4.55, 1.12);
    timeOfFlightMap.put(3.15, 1.11);
    timeOfFlightMap.put(1.88, 1.09);
    timeOfFlightMap.put(1.38, 0.90);
  }

  // Suppliers for pose and velocit

  /** Call this in a periodic loop */
  public void periodic() {
    if (robotPoseSupplier == null || robotVelocitySupplier == null) return;

    Pose2d robotPose = robotPoseSupplier.get();
    ChassisSpeeds robotVelocity = robotVelocitySupplier.get();

    // Phase delay
    Pose2d estimatedPose =
        robotPose.exp(
            new Twist2d(
                robotVelocity.vxMetersPerSecond * phaseDelay,
                robotVelocity.vyMetersPerSecond * phaseDelay,
                robotVelocity.omegaRadiansPerSecond * phaseDelay));

    // Turret position
    Pose2d turretPosition = estimatedPose.transformBy(robotToTurret);

    // Target

    // Distance from turret to target
    double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());

    // Field-relative turret velocity
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

    // Iteratively calculate lookahead pose
    Pose2d lookaheadPose = turretPosition;
    double timeOfFlight = 0.0;
    for (int i = 0; i < 20; i++) {
      timeOfFlight = timeOfFlightMap.get(turretToTargetDistance);
      double offsetX = turretVelocityX * timeOfFlight;
      double offsetY = turretVelocityY * timeOfFlight;
      lookaheadPose =
          new Pose2d(
              turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              turretPosition.getRotation());
    }

    this.lookaheadPose = lookaheadPose;

    Logger.recordOutput("ShotCalculator/LookaheadPose", lookaheadPose);
    Logger.recordOutput("ShotCalculator/TurretToTargetDistance", turretToTargetDistance);
  }

  public Pose2d getLookaheadPose() {
    return lookaheadPose;
  }

  public void setTarget(Translation2d target) {
    this.target = target;
  }
}
