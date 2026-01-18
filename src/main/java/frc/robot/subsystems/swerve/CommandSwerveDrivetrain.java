// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.*;

import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.swerve.generated.TunerConstants;
import frc.robot.subsystems.swerve.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.utils.LoggedTracer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements Subsystem so it can easily
 * be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {

  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  @AutoLogOutput private double lastGyroYawRads = 0;
  @AutoLogOutput private double accumGyroYawRads = 0;
  @AutoLogOutput private double averageWheelPosition = 0;
  @AutoLogOutput private double[] startWheelPositions = new double[4];
  @AutoLogOutput private double currentEffectiveWheelRadius = 0;

  private ChassisSpeeds previousSpeeds = new ChassisSpeeds();
  private double previousTime = 0.0;

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean m_hasAppliedOperatorPerspective = false;

  /** Swerve request to apply during field-centric path following */
  private final SwerveRequest.ApplyFieldSpeeds m_pathApplyFieldSpeeds =
      new SwerveRequest.ApplyFieldSpeeds()
          .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

  private final PIDController xController = new PIDController(5.0, 0.0, 0);
  private final PIDController yController = new PIDController(5.0, 0.0, 0);
  private final PIDController headingController = new PIDController(5, 0, 0);

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param modules Constants for each specific module
   */
  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set to
   *     0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
   * @param modules Constants for each specific module
   */
  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants,
      double odometryUpdateFrequency,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, odometryUpdateFrequency, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set to
   *     0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
   * @param odometryStandardDeviation The standard deviation for odometry calculation in the form
   *     [x, y, theta]ᵀ, with units in meters and radians
   * @param visionStandardDeviation The standard deviation for vision calculation in the form [x, y,
   *     theta]ᵀ, with units in meters and radians
   * @param modules Constants for each specific module
   */
  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants,
      double odometryUpdateFrequency,
      Matrix<N3, N1> odometryStandardDeviation,
      Matrix<N3, N1> visionStandardDeviation,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(
        drivetrainConstants,
        odometryUpdateFrequency,
        odometryStandardDeviation,
        visionStandardDeviation,
        modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  public Command pidToPose(Supplier<Pose2d> target) {
    if (target == null) {
      System.out.println("**** pidToPose disabled because of null target");
      return Commands.none();
    }
    xController.setTolerance(.02);
    yController.setTolerance(.02);
    headingController.setTolerance(Math.toRadians(.1));
    return run(() -> {
          Logger.recordOutput("AutoAlign/Target", target.get());
          Logger.recordOutput("AutoAlign/Running", true);
          this.setControl(
              m_pathApplyFieldSpeeds.withSpeeds(
                  new ChassisSpeeds(
                      xController.calculate(this.getState().Pose.getX(), target.get().getX()),
                      yController.calculate(this.getState().Pose.getY(), target.get().getY()),
                      headingController.calculate(
                          this.getState().Pose.getRotation().getRadians(),
                          target.get().getRotation().getRadians()))));
        })
        .finallyDo(
            () -> {
              Logger.recordOutput("AutoAlign/Running", false);
            });
  }

  /**
   * Creates a new auto factory for this drivetrain.
   *
   * @return AutoFactory for this drivetrain
   */
  public AutoFactory createAutoFactory() {
    return createAutoFactory((sample, isStart) -> {});
  }

  public void trajLogger(Trajectory<SwerveSample> sample, boolean isStart) {
    Logger.recordOutput(this.getClass().getSimpleName() + "/Choreo/TrajPoses", sample.getPoses());
  }

  /**
   * Creates a new auto factory for this drivetrain with the given trajectory logger.
   *
   * @param trajLogger Logger for the trajectory
   * @return AutoFactory for this drivetrain
   */
  public AutoFactory createAutoFactory(TrajectoryLogger<SwerveSample> trajLogger) {
    return new AutoFactory(
        () -> this.getState().Pose, this::resetPose, this::followPath, true, this, trajLogger);
  }

  /**
   * Returns a command that applies the specified control request to this swerve drivetrain.
   *
   * @param requestSupplier Function returning the request to apply
   * @return Command to run
   */
  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  public Command applyRequest(SwerveRequest swerveRequest) {
    return run(() -> this.setControl(swerveRequest));
  }

  /**
   * Follows the given field-centric path sample with PID.
   *
   * @param sample Sample along the path to follow
   */
  public void followPath(SwerveSample sample) {
    headingController.enableContinuousInput(-Math.PI, Math.PI);

    var pose = this.getState().Pose;

    var targetSpeeds = sample.getChassisSpeeds();
    targetSpeeds.vxMetersPerSecond += xController.calculate(pose.getX(), sample.x);
    targetSpeeds.vyMetersPerSecond += yController.calculate(pose.getY(), sample.y);
    targetSpeeds.omegaRadiansPerSecond +=
        headingController.calculate(pose.getRotation().getRadians(), sample.heading);

    Logger.recordOutput("Choreo/targetSpeeds", sample.getChassisSpeeds());
    Logger.recordOutput("Choreo/targetPose", sample.getPose());

    setControl(
        m_pathApplyFieldSpeeds
            .withSpeeds(targetSpeeds)
            .withWheelForceFeedforwardsX(sample.moduleForcesX())
            .withWheelForceFeedforwardsY(sample.moduleForcesY()));
  }

  @Override
  public void periodic() {
    /*
     * Periodically try to apply the operator perspective.
     * If we haven't applied the operator perspective before, then we should apply
     * it regardless of DS state.
     * This allows us to correct the perspective in case the robot code restarts
     * mid-match.
     * Otherwise, only check and apply the operator perspective if the DS is
     * disabled.
     * This ensures driving behavior doesn't change until an explicit disable event
     * occurs during testing.
     */
    if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance()
          .ifPresent(
              allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
              });
    }

    double currentTime = Logger.getTimestamp() / 1e6;
    ChassisSpeeds currentSpeeds = getState().Speeds;

    if (previousTime > 0) {
      double dt = currentTime - previousTime;
    }

    previousSpeeds = currentSpeeds;
    previousTime = currentTime;

     LoggedTracer.record(this.getClass().getSimpleName());
  }

  public void addPhotonEstimate(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    // if (!questNav.connected()) {
    // if (DriverStation.isDisabled() || !questNav.connected()) {
    this.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    // }

    // }
  }

  public frc.robot.utils.sotm.ChassisAccelerations getFieldRelativeAccelerations() {
    double currentTime = Logger.getTimestamp() / 1e6;
    double dt = previousTime > 0 ? currentTime - previousTime : 0.02;

    ChassisSpeeds currentSpeeds = getState().Speeds;

    return new frc.robot.utils.sotm.ChassisAccelerations(
        currentSpeeds,
        previousSpeeds,
        dt
    );
  }

  /**
   * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
   * while still accounting for measurement noise.
   *
   * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
   * @param timestampSeconds The timestamp of the vision measurement in seconds.
   */
  @Override
  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
    super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
  }

  /**
   * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
   * while still accounting for measurement noise.
   *
   * <p>Note that the vision measurement standard deviations passed into this method will continue
   * to apply to future measurements until a subsequent call to {@link
   * #setVisionMeasurementStdDevs(Matrix)} or this method.
   *
   * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
   * @param timestampSeconds The timestamp of the vision measurement in seconds.
   * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement in the form
   *     [x, y, theta]ᵀ, with units in meters and radians.
   */
  @Override
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    super.addVisionMeasurement(
        visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
  }

  private void startSimThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - m_lastSimTime;
              m_lastSimTime = currentTime;

              /* use the measured time delta, get battery voltage from WPILib */
              updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }

  /**
   * Runs the wheel radius characterization routine.
   *
   * @param omegaDirection Direction of the robot to turn either 1 or -1
   */
  public Command wheelRadiusCharacterization(double omegaDirection) {

    /* wheel radius characterization schtuffs */
    final DoubleSupplier m_gyroYawRadsSupplier =
        () -> Units.degreesToRadians(getPigeon2().getYaw().getValueAsDouble());
    // () -> getState().Pose.getRotation().getRadians();
    final SlewRateLimiter m_omegaLimiter = new SlewRateLimiter(0.5);
    final SwerveRequest.RobotCentric m_characterizationReq =
        new SwerveRequest.RobotCentric()
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);
    final double m_characterizationSpeed = 1.5;

    var initialize =
        runOnce(
            () -> {
              lastGyroYawRads = m_gyroYawRadsSupplier.getAsDouble();
              accumGyroYawRads = 0;
              currentEffectiveWheelRadius = 0;
              averageWheelPosition = 0;
              for (int i = 0; i < getModules().length; i++) {
                var pos = getModules()[i].getPosition(true);
                startWheelPositions[i] =
                    pos.distanceMeters * TunerConstants.kDriveRotationsPerMeter;
              }
              m_omegaLimiter.reset(0);
            });

    var executeEnd =
        runEnd(
            () -> {
              setControl(
                  m_characterizationReq.withRotationalRate(
                      m_omegaLimiter.calculate(m_characterizationSpeed * omegaDirection)));
              accumGyroYawRads +=
                  MathUtil.angleModulus(m_gyroYawRadsSupplier.getAsDouble() - lastGyroYawRads);
              lastGyroYawRads = m_gyroYawRadsSupplier.getAsDouble();
              averageWheelPosition = 0;
              double[] wheelPositions = new double[4];
              for (int i = 0; i < getModules().length; i++) {
                var pos = getModules()[i].getPosition(true);
                wheelPositions[i] = pos.distanceMeters * TunerConstants.kDriveRotationsPerMeter;
                averageWheelPosition += Math.abs(wheelPositions[i] - startWheelPositions[i]);
              }
              averageWheelPosition = averageWheelPosition / 4.0;
              currentEffectiveWheelRadius =
                  (accumGyroYawRads * TunerConstants.kDriveRadius) / averageWheelPosition;
              // System.out.println("effective wheel radius: " + currentEffectiveWheelRadius);
              System.out.println("Average Wheel Position: " + averageWheelPosition);
            },
            () -> {
              setControl(m_characterizationReq.withRotationalRate(0));
              if (Math.abs(accumGyroYawRads) <= Math.PI * 2.0) {
                System.out.println(
                    "not enough data for characterization "
                        + accumGyroYawRads
                        + "\navgWheelPos: "
                        + averageWheelPosition
                        + "radians");
              } else {
                System.out.println(
                    "effective wheel radius: "
                        + currentEffectiveWheelRadius
                        + " inches"
                        + "\naccumGryoYawRads: "
                        + accumGyroYawRads
                        + "radians"
                        + "\navgWheelPos: "
                        + averageWheelPosition
                        + "radians");
              }
            });

    return Commands.sequence(initialize, executeEnd);
  }
}
