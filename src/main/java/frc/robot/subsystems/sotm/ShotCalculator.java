package frc.robot.subsystems.sotm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.FieldConstants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.utils.DisableSubsystem;
import frc.robot.utils.LoggedTracer;
import org.littletonrobotics.junction.Logger;

public class ShotCalculator extends DisableSubsystem {
  private final CommandSwerveDrivetrain drivetrain;

  private Pose3d currentEffectiveTargetPose = new Pose3d();
  private double currentEffectiveYaw = 0.0;
  private double currentPivotAngle = 0.0;
  private double currentShooterSpeed = 0.0;

  private Pose3d targetLocation = new Pose3d(FieldConstants.Processor.centerFace);
  private double targetDistance = 0.0;

  private final Transform3d robotToShooter = ShotCalculatorConstants.ROBOT_TO_SHOOTER;

  private ChassisSpeeds previousSpeeds = new ChassisSpeeds();
  private double previousTime = 0.0;

  public ShotCalculator(boolean enabled, CommandSwerveDrivetrain drivetrain) {
    super(enabled);
    this.drivetrain = drivetrain;
  }

  @Override
  public void periodic() {
    super.periodic();

    double currentTime = Logger.getTimestamp() / 1e6;
    double dt = previousTime > 0 ? currentTime - previousTime : 0.02;

    Pose2d robotPose = drivetrain.getState().Pose;
    ChassisSpeeds robotSpeeds = drivetrain.getState().Speeds;

    ChassisAccelerations robotAccelerations = new ChassisAccelerations(
        robotSpeeds,
        previousSpeeds,
        dt
    );

    Pose3d shooterPose = new Pose3d(robotPose).plus(robotToShooter);

    targetDistance = shooterPose.getTranslation()
        .getDistance(targetLocation.getTranslation());

    ShotSolution solution = solveShootOnTheMoveImproved(
        shooterPose,
        targetLocation,
        robotSpeeds,
        robotAccelerations,
        targetDistance
    );

    if (solution != null) {
      currentEffectiveTargetPose = solution.effectiveTargetPose;
      currentEffectiveYaw = solution.requiredYaw;
      currentPivotAngle = solution.pivotAngle;
      currentShooterSpeed = solution.shooterSpeed;
    }

    previousSpeeds = robotSpeeds;
    previousTime = currentTime;

    Logger.recordOutput("ShotCalculator/EffectiveTargetPose", currentEffectiveTargetPose);
    Logger.recordOutput("ShotCalculator/EffectiveYaw", currentEffectiveYaw);
    Logger.recordOutput("ShotCalculator/PivotAngle", currentPivotAngle);
    Logger.recordOutput("ShotCalculator/ShooterSpeed", currentShooterSpeed);
    Logger.recordOutput("ShotCalculator/TargetDistance", targetDistance);
    Logger.recordOutput("ShotCalculator/TargetLocation", targetLocation);
    Logger.recordOutput("ShotCalculator/RobotAccelX", robotAccelerations.axMetersPerSecondSquared);
    Logger.recordOutput("ShotCalculator/RobotAccelY", robotAccelerations.ayMetersPerSecondSquared);

    LoggedTracer.record("ShotCalculator");
  }

  private ShotSolution solveShootOnTheMoveImproved(
      Pose3d shooterPose,
      Pose3d targetPose,
      ChassisSpeeds robotSpeeds,
      ChassisAccelerations robotAccelerations,
      double distance) {

    double targetSpeedRps = ShotCalculatorConstants.DISTANCE_TO_SHOOTER_SPEED.get(distance);

    BallPhysics.ShotSolution initialSol;
    try {
      initialSol = BallPhysics.solveBallisticWithSpeed(
          shooterPose,
          targetPose,
          targetSpeedRps);
    } catch (IllegalArgumentException e) {
      return null;
    }

    if (initialSol.flightTimeSeconds() == 0) {
      return null;
    }

    double t = initialSol.flightTimeSeconds();
    Pose3d effectiveTarget = targetPose;

    for (int i = 0; i < ShotCalculatorConstants.kMaxIterations; i++) {
      double dx = robotSpeeds.vxMetersPerSecond * t;
      double dy = robotSpeeds.vyMetersPerSecond * t;

      effectiveTarget = new Pose3d(
          targetPose.getX() - dx,
          targetPose.getY() - dy,
          targetPose.getZ(),
          targetPose.getRotation());

      BallPhysics.ShotSolution newSol;
      try {
        newSol = BallPhysics.solveBallisticWithSpeed(
            shooterPose,
            effectiveTarget,
            targetSpeedRps);
      } catch (IllegalArgumentException e) {
        break;
      }

      if (Math.abs(newSol.flightTimeSeconds() - t) < ShotCalculatorConstants.kConvergenceThreshold) {
        Translation3d shooterPos = shooterPose.getTranslation();
        Translation3d effectivePos = effectiveTarget.getTranslation();
        double deltaX = effectivePos.getX() - shooterPos.getX();
        double deltaY = effectivePos.getY() - shooterPos.getY();
        double requiredYaw = Math.atan2(deltaY, deltaX);

        return new ShotSolution(
            effectiveTarget,
            requiredYaw,
            newSol.launchPitchRad(),
            targetSpeedRps
        );
      }

      t = newSol.flightTimeSeconds();
    }

    if (effectiveTarget != null) {
      Translation3d shooterPos = shooterPose.getTranslation();
      Translation3d effectivePos = effectiveTarget.getTranslation();
      double deltaX = effectivePos.getX() - shooterPos.getX();
      double deltaY = effectivePos.getY() - shooterPos.getY();
      double requiredYaw = Math.atan2(deltaY, deltaX);

      try {
        BallPhysics.ShotSolution finalSol = BallPhysics.solveBallisticWithSpeed(
            shooterPose,
            effectiveTarget,
            targetSpeedRps);

        return new ShotSolution(
            effectiveTarget,
            requiredYaw,
            finalSol.launchPitchRad(),
            targetSpeedRps
        );
      } catch (IllegalArgumentException e) {
        return null;
      }
    }

    return null;
  }

  @SuppressWarnings("unused")
  private ShotSolution solveShootOnTheMove(
      Pose3d shooterPose,
      Pose3d targetPose,
      ChassisSpeeds robotSpeeds,
      double distance) {

    double flightTime = estimateFlightTime(distance);

    for (int i = 0; i < ShotCalculatorConstants.kMaxIterations; i++) {
      Translation3d futureRobotPosition = predictRobotPosition(
          shooterPose.getTranslation(),
          robotSpeeds,
          flightTime
      );

      Translation3d effectiveTarget = targetPose.getTranslation();

      double newDistance = futureRobotPosition.getDistance(effectiveTarget);

      double newFlightTime = estimateFlightTime(newDistance);

      if (Math.abs(newFlightTime - flightTime) < ShotCalculatorConstants.kConvergenceThreshold) {
        double dx = effectiveTarget.getX() - futureRobotPosition.getX();
        double dy = effectiveTarget.getY() - futureRobotPosition.getY();
        double requiredYaw = Math.atan2(dy, dx);

        double dz = effectiveTarget.getZ() - futureRobotPosition.getZ();
        double horizontalDistance = Math.hypot(dx, dy);
        double pivotAngle = Math.atan2(dz, horizontalDistance);
        double shooterSpeed = calculateShooterSpeed(newDistance, pivotAngle);

        return new ShotSolution(
            new Pose3d(effectiveTarget, targetPose.getRotation()),
            requiredYaw,
            pivotAngle,
            shooterSpeed
        );
      }

      flightTime = newFlightTime;
    }

    return null;
  }

  private Translation3d predictRobotPosition(
      Translation3d currentPosition,
      ChassisSpeeds speeds,
      double time) {

    double dx = speeds.vxMetersPerSecond * time;
    double dy = speeds.vyMetersPerSecond * time;

    return new Translation3d(
        currentPosition.getX() + dx,
        currentPosition.getY() + dy,
        currentPosition.getZ()
    );
  }

  private double estimateFlightTime(double distance) {
    double estimatedVelocity = 10.0;
    return distance / estimatedVelocity;
  }

  private double calculateShooterSpeed(double distance, double angle) {
    double sinTwoTheta = Math.sin(2 * angle);
    if (Math.abs(sinTwoTheta) < 0.01) {
      sinTwoTheta = 0.01;
    }

    double speedSquared = (ShotCalculatorConstants.kGravity * distance) / sinTwoTheta;
    return Math.sqrt(Math.max(0, speedSquared));
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

  public double getCurrentPivotAngle() {
    return currentPivotAngle;
  }

  public double getCurrentShooterSpeed() {
    return currentShooterSpeed;
  }

  public double getTargetDistance() {
    return targetDistance;
  }

  private static class ShotSolution {
    public final Pose3d effectiveTargetPose;
    public final double requiredYaw;
    public final double pivotAngle;
    public final double shooterSpeed;

    public ShotSolution(
        Pose3d effectiveTargetPose,
        double requiredYaw,
        double pivotAngle,
        double shooterSpeed) {
      this.effectiveTargetPose = effectiveTargetPose;
      this.requiredYaw = requiredYaw;
      this.pivotAngle = pivotAngle;
      this.shooterSpeed = shooterSpeed;
    }
  }
}
