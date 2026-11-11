// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.AutoConstants.kRecoveryHeadingErrorThresholdRadians;
import static frc.robot.Constants.AutoConstants.kRecoveryHeadingToleranceRadians;
import static frc.robot.Constants.AutoConstants.kRecoveryTranslationErrorThresholdMeters;
import static frc.robot.Constants.AutoConstants.kRecoveryTranslationToleranceMeters;

import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoFactory;
import choreo.auto.SwerveTrajectoryRecoveryConfig;
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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.sim.MapleSimSwerveDrivetrain;
import frc.robot.sim.RebuiltFieldSim;
import frc.robot.sim.SimConstants;
import frc.robot.subsystems.swerve.generated.TunerConstants;
import frc.robot.subsystems.swerve.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.utils.LoggedTracer;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements Subsystem so it can easily
 * be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {

  private Notifier m_simNotifier = null;
  private MapleSimSwerveDrivetrain m_mapleSim = null;
  private boolean m_wasAutonomousEnabled = false;
  private double m_autonomousStartTime = Double.NEGATIVE_INFINITY;

  /**
   * How long after auto starts a resetPose still places the simulated robot: long enough for the
   * auto's initial odometry reset, short enough that later resets only reset odometry.
   */
  private static final double kSimPlacementWindowSeconds = 0.25;

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

  private final PIDController xController = new PIDController(8.0, 0.0, 0);
  private final PIDController yController = new PIDController(8.0, 0.0, 0);
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
    super(
        drivetrainConstants,
        MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(modules));
    if (Constants.currentMode == Constants.Mode.SIM) {
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
    super(
        drivetrainConstants,
        odometryUpdateFrequency,
        MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(modules));
    if (Constants.currentMode == Constants.Mode.SIM) {
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
   *     [x, y, theta], with units in meters and radians
   * @param visionStandardDeviation The standard deviation for vision calculation in the form [x, y,
   *     theta], with units in meters and radians
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
        MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(modules));
    if (Constants.currentMode == Constants.Mode.SIM) {
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
            () -> this.getState().Pose, this::resetPose, this::followPath, true, this, trajLogger)
        .withSwerveTrajectoryRecovery(
            new SwerveTrajectoryRecoveryConfig(
                kRecoveryTranslationErrorThresholdMeters,
                kRecoveryHeadingErrorThresholdRadians,
                kRecoveryTranslationToleranceMeters,
                kRecoveryHeadingToleranceRadians));
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
    boolean autonomousEnabled = DriverStation.isAutonomousEnabled();
    if (autonomousEnabled && !m_wasAutonomousEnabled) {
      m_autonomousStartTime = Timer.getFPGATimestamp();
    }
    m_wasAutonomousEnabled = autonomousEnabled;

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

    // getState() is a snapshot from the odometry thread; log it here, on the main thread, since
    // AdvantageKit's Logger isn't thread-safe (registerTelemetry runs on the odometry thread)
    var state = getState();
    Logger.recordOutput("Drive/Pose", state.Pose);
    Logger.recordOutput("Drive/RobotRelativeSpeeds", state.Speeds);
    Logger.recordOutput("Drive/ModuleStates", state.ModuleStates);
    Logger.recordOutput("Drive/ModuleTargets", state.ModuleTargets);
    Logger.recordOutput("Drive/ModulePositions", state.ModulePositions);
    Logger.recordOutput("Drive/OdometryFrequency", 1.0 / state.OdometryPeriod);
    Logger.recordOutput("Drive/FailedDaqs", state.FailedDaqs);

    previousSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(state.Speeds, state.RawHeading);
    previousTime = state.Timestamp;

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

  public ChassisSpeeds getFieldRelativeSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(getState().Speeds, getState().Pose.getRotation());
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
   *     [x, y, theta], with units in meters and radians.
   */
  @Override
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    super.addVisionMeasurement(
        visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
  }

  /**
   * Runs the drivetrain in maple-sim instead of CTRE's simple sim, so the robot slips, collides
   * with the field and drifts from its odometry like a real one. The arena steps faster than the
   * robot loop so the 250 Hz CTRE PID behaves.
   */
  private void startSimThread() {
    RebuiltFieldSim.createArena();
    m_mapleSim =
        new MapleSimSwerveDrivetrain(
            SimConstants.kArenaPeriod,
            SimConstants.kRobotMassWithBumpers,
            SimConstants.kBumperLength,
            SimConstants.kBumperLength,
            SimConstants.kDriveMotor,
            SimConstants.kSteerMotor,
            SimConstants.kWheelCOF,
            new Translation2d[] {
              SwerveConstants.frontLeft,
              SwerveConstants.frontRight,
              SwerveConstants.backLeft,
              SwerveConstants.backRight
            },
            getPigeon2(),
            getModules(),
            TunerConstants.FrontLeft,
            TunerConstants.FrontRight,
            TunerConstants.BackLeft,
            TunerConstants.BackRight);
    resetPose(SimConstants.kStartingPose);

    m_simNotifier = new Notifier(m_mapleSim::update);
    m_simNotifier.setName("MapleSim");
    m_simNotifier.startPeriodic(SimConstants.kArenaPeriod.in(Seconds));
  }

  /** The maple-sim drivetrain, when running in simulation. */
  public Optional<MapleSimSwerveDrivetrain> getMapleSim() {
    return Optional.ofNullable(m_mapleSim);
  }

  /**
   * Resets the pose estimate. In simulation, a reset while disabled or right as auto starts also
   * places the simulated robot there, the way the drive team sets the robot down before a match.
   * Resets later on only reset odometry, as on the real robot.
   */
  @Override
  public void resetPose(Pose2d pose) {
    boolean placingRobot =
        DriverStation.isDisabled()
            || Timer.getFPGATimestamp() - m_autonomousStartTime < kSimPlacementWindowSeconds;
    if (m_mapleSim != null && placingRobot) {
      // The arena steps on the sim notifier thread while holding its own lock
      synchronized (SimulatedArena.getInstance()) {
        m_mapleSim.mapleSimDrive.setSimulationWorldPose(pose);
      }
    }
    super.resetPose(pose);
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
