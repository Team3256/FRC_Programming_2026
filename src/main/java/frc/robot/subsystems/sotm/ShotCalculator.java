// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.sotm;

import static frc.robot.subsystems.sotm.ShotCalculatorConstants.DISTANCE_TO_SHOOTER_SPEED;
import static frc.robot.subsystems.sotm.ShotCalculatorConstants.ROBOT_TO_SHOOTER;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.utils.sotm.ChassisAccelerations;
import frc.robot.utils.sotm.ShootOnTheFlyCalculator;
import frc.robot.utils.sotm.ShootOnTheFlyCalculator.InterceptSolution;
import org.littletonrobotics.junction.Logger;

// stores current target and actively computes effective target
public class ShotCalculator extends SubsystemBase {
  private final CommandSwerveDrivetrain drivetrain;

  private Pose3d currentEffectiveTargetPose = Pose3d.kZero;
  private double currentEffectiveYaw;
  private InterceptSolution currentInterceptSolution;

  // private Pose3d targetLocation = new Pose3d(FieldConstants.Hub.topCenterPoint, new
  // Rotation3d(Rotation2d.kZero));

  private Pose3d targetLocation;

  private double targetDistance = 0.0;

  // Shooter flywheel target speed
  private double targetSpeedRps = 8;

  public ShotCalculator(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
  }

  @Override
  public void periodic() {
    Pose2d drivetrainPose = drivetrain.getState().Pose;

    targetDistance =
        drivetrainPose.getTranslation().getDistance(targetLocation.toPose2d().getTranslation());
    targetSpeedRps = DISTANCE_TO_SHOOTER_SPEED.get(targetDistance);

    Pose3d shooterPose = new Pose3d(drivetrainPose).plus(ROBOT_TO_SHOOTER);

    ChassisSpeeds drivetrainSpeeds = drivetrain.getState().Speeds;
    ChassisAccelerations drivetrainAccelerations = drivetrain.getFieldRelativeAccelerations();

    currentInterceptSolution =
        ShootOnTheFlyCalculator.solveShootOnTheFly(
            shooterPose,
            targetLocation,
            drivetrainSpeeds,
            drivetrainAccelerations,
            targetSpeedRps,
            5,
            0.01);

    currentEffectiveTargetPose = currentInterceptSolution.effectiveTargetPose();
    currentEffectiveYaw = currentInterceptSolution.requiredYaw();

    Logger.recordOutput("ShotCalculator/EffectiveTargetPose", currentEffectiveTargetPose);
    Logger.recordOutput("ShotCalculator/EffectiveYaw", currentEffectiveYaw);
    Logger.recordOutput("ShotCalculator/TargetDistance", targetDistance);
    Logger.recordOutput("ShotCalculator/TargetLocation", targetLocation);
    Logger.recordOutput("ShotCalculator/TargetSpeedRps", targetSpeedRps);
  }

  public void setTarget(Pose3d targetLocation) {
    this.targetLocation = targetLocation;
  }

  public Pose3d getCurrentEffectiveTargetPose() {
    return currentEffectiveTargetPose;
  }

  public double getCurrentEffectiveYaw() {
    return currentEffectiveYaw;
  }

  public InterceptSolution getInterceptSolution() {
    return currentInterceptSolution;
  }

  public Rotation2d getCurrentPivotAngle() {
    return new Rotation2d(
        currentInterceptSolution != null ? currentInterceptSolution.launchPitchRad() : 0.0);
  }

  public double getCurrentShooterSpeed() {
    return targetSpeedRps;
  }

  public double getTargetDistance() {
    return targetDistance;
  }
}
