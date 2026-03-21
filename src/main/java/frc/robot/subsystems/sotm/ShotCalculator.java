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

  private static final double funnyNum = 0;
  private static final InterpolatingDoubleTreeMap timeOfFlightMapHub =
      new InterpolatingDoubleTreeMap() {
        {
          put(1.322, 0.88);
          put(1.826, 1.0);
          put(2.247, 1.01);
          put(2.765, 1.1);
          put(3.135, 1.0);
          put(3.501, 1.02);
          put(3.901, 1.09);
          put(4.017, 1.12);
          put(4.560, 1.2);
          put(4.857, 1.27);
          put(5.616, 1.2);
          put(5.837, 1.27);
        }
      };
  private static final InterpolatingDoubleTreeMap timeOfFlightMapFeed =
      new InterpolatingDoubleTreeMap() {
        {
          put(4.38, 1.12);
          put(6.11, 1.2);
          put(8.0, 1.3);
          put(9.42, 1.3);
          put(10.8, 1.55);
          put(12.834, 1.7);
        }
      };
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
    phaseDelay = 0.05;
  }

  // Suppliers for pose and velocit

  /** Call this in a periodic loop */
  public void periodic() {
    if (robotPoseSupplier == null || robotVelocitySupplier == null) return;

    InterpolatingDoubleTreeMap timeOfFlightMap = timeOfFlightMapHub;
    if (!(target.equals(FieldConstants.Hub.topCenterPoint.toTranslation2d())
        || target.equals(FieldConstants.Hub.oppTopCenterPoint.toTranslation2d()))) {
      timeOfFlightMap = timeOfFlightMapFeed;
    }

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
    double currentDistance = turretToTargetDistance; // starting estimate

    for (int i = 0; i < 20; i++) {
      double timeOfFlight = timeOfFlightMap.get(currentDistance) - funnyNum;

      double offsetX = turretVelocityX * timeOfFlight;
      double offsetY = turretVelocityY * timeOfFlight;

      // update lookahead pose
      lookaheadPose =
          new Pose2d(
              turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              turretPosition.getRotation());

      // let's update the distance this time guys...
      currentDistance = target.getDistance(lookaheadPose.getTranslation());
    }

    this.lookaheadPose = lookaheadPose;

    Logger.recordOutput("ShotCalculator/LookaheadPose", lookaheadPose);
    Logger.recordOutput("ShotCalculator/TurretToTargetDistance", turretToTargetDistance);
  }

  public Pose2d getLookaheadPose() {
    return lookaheadPose;
  }

  public double getDistance() {
    return lookaheadPose.getTranslation().getDistance(target);
  }

  public void setTarget(Translation2d target) {
    this.target = target;
  }
}
