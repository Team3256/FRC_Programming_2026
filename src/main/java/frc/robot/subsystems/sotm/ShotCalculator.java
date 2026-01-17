package frc.robot.subsystems.sotm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.FieldConstants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.utils.DisableSubsystem;
import frc.robot.utils.LoggedTracer;
import frc.robot.subsystems.sotm.ShotCalculatorConstants;
import org.littletonrobotics.junction.Logger;

public class ShotCalculator extends DisableSubsystem {
  private final CommandSwerveDrivetrain drivetrain;

// current values
  private Pose3d currentEffectiveTargetPose = new Pose3d();
  private double currentEffectiveYaw = 0.0;
  private double currentPivotAngle = 0.0;
  private double currentShooterSpeed = 0.0;

// target values
  private Pose3d targetLocation = new Pose3d(FieldConstants.Processor.centerFace);
  private double targetDistance = 0.0;

  private final Transform3d robotToShooter = ShotCalculatorConstants.ROBOT_TO_SHOOTER;


  public ShotCalculator(boolean enabled, CommandSwerveDrivetrain drivetrain) {
    super(enabled);
    this.drivetrain = drivetrain;
  }

  @Override
  public void periodic() {
    super.periodic();

    Pose2d robotPose = drivetrain.getState().Pose;
    ChassisSpeeds robotSpeeds = drivetrain.getState().Speeds;

    Pose3d shooterPose = new Pose3d(robotPose).plus(robotToShooter);

    targetDistance = shooterPose.getTranslation()
        .getDistance(targetLocation.getTranslation());

    ShotSolution solution = solveShootOnTheMove(
        shooterPose,
        targetLocation,
        robotSpeeds,
        targetDistance
    );

    if (solution != null) {
      currentEffectiveTargetPose = solution.effectiveTargetPose;
      currentEffectiveYaw = solution.requiredYaw;
      currentPivotAngle = solution.pivotAngle;
      currentShooterSpeed = solution.shooterSpeed;
    }

    Logger.recordOutput("ShotCalculator/EffectiveTargetPose", currentEffectiveTargetPose);
    Logger.recordOutput("ShotCalculator/EffectiveYaw", currentEffectiveYaw);
    Logger.recordOutput("ShotCalculator/PivotAngle", currentPivotAngle);
    Logger.recordOutput("ShotCalculator/ShooterSpeed", currentShooterSpeed);
    Logger.recordOutput("ShotCalculator/TargetDistance", targetDistance);
    Logger.recordOutput("ShotCalculator/TargetLocation", targetLocation);

    LoggedTracer.record("ShotCalculator");
  }

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
    double estimatedVelocity = 10.0; // m/s
    return distance / estimatedVelocity;
  }


  private double calculateShooterSpeed(double distance, double angle) {
    // v = sqrt((g * d) / sin(2 * theta))

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
