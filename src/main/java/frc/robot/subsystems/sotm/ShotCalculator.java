package frc.robot.subsystems.sotm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.utils.sotm.ChassisAccelerations;
import frc.robot.utils.sotm.ShootOnTheFlyCalculator;
import frc.robot.utils.sotm.ShootOnTheFlyCalculator.InterceptSolution;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.sotm.ShotCalculatorConstants.ROBOT_TO_SHOOTER;
import static frc.robot.subsystems.sotm.ShotCalculatorConstants.DISTANCE_TO_SHOOTER_SPEED;

// stores current target and actively computes effective target
public class ShotCalculator extends SubsystemBase {
    private final CommandSwerveDrivetrain drivetrain;

    private Pose3d currentEffectiveTargetPose = Pose3d.kZero;
    private double currentEffectiveYaw;
    private InterceptSolution currentInterceptSolution;

    private Pose3d targetLocation = new Pose3d(FieldConstants.Processor.centerFace);
    private double targetDistance = 0.0;
    private double targetSpeedRps = 8;

    private ChassisSpeeds previousSpeeds = new ChassisSpeeds();
    private double previousTime = 0.0;

    public ShotCalculator(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void periodic() {
        double currentTime = Logger.getTimestamp() / 1e6;
        double dt = previousTime > 0 ? currentTime - previousTime : 0.02;

        Pose2d drivetrainPose = drivetrain.getState().Pose;
        ChassisSpeeds drivetrainSpeeds = drivetrain.getState().Speeds;

        ChassisAccelerations drivetrainAccelerations = new ChassisAccelerations(
                drivetrainSpeeds,
                previousSpeeds,
                dt
        );

        targetDistance = drivetrainPose.getTranslation().getDistance(targetLocation.toPose2d().getTranslation());
        targetSpeedRps = DISTANCE_TO_SHOOTER_SPEED.get(targetDistance);

        Pose3d shooterPose = new Pose3d(drivetrainPose).plus(ROBOT_TO_SHOOTER);

        currentInterceptSolution = ShootOnTheFlyCalculator.solveShootOnTheFly(
                shooterPose,
                targetLocation,
                drivetrainSpeeds,
                drivetrainAccelerations,
                targetSpeedRps,
                5,
                0.01
        );

        currentEffectiveTargetPose = currentInterceptSolution.effectiveTargetPose();
        currentEffectiveYaw = currentInterceptSolution.requiredYaw();

        previousSpeeds = drivetrainSpeeds;
        previousTime = currentTime;

        Logger.recordOutput("ShotCalculator/EffectiveTargetPose", currentEffectiveTargetPose);
        Logger.recordOutput("ShotCalculator/EffectiveYaw", currentEffectiveYaw);
        Logger.recordOutput("ShotCalculator/TargetDistance", targetDistance);
        Logger.recordOutput("ShotCalculator/TargetLocation", targetLocation);
        Logger.recordOutput("ShotCalculator/TargetSpeedRps", targetSpeedRps);
    }

    public void setTarget(Pose3d targetLocation, double targetSpeedRps) {
        this.targetLocation = targetLocation;
        this.targetSpeedRps = targetSpeedRps;
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

    public double getCurrentPivotAngle() {
        return currentInterceptSolution != null ? currentInterceptSolution.launchPitchRad() : 0.0;
    }

    public double getCurrentShooterSpeed() {
        return targetSpeedRps;
    }

    public double getTargetDistance() {
        return targetDistance;
    }
}
