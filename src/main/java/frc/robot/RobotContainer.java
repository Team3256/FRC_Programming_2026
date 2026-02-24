// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.subsystems.swerve.SwerveConstants.*;

import choreo.auto.AutoChooser;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.AutoRoutines;
import frc.robot.sim.SimMechs;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIOSim;
import frc.robot.subsystems.feeder.FeederIOTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.intakepivot.IntakePivot;
import frc.robot.subsystems.intakepivot.IntakePivotIOSim;
import frc.robot.subsystems.intakepivot.IntakePivotIOTalonFX;
import frc.robot.subsystems.intakerollers.IntakeRollers;
import frc.robot.subsystems.intakerollers.IntakeRollersIOSim;
import frc.robot.subsystems.intakerollers.IntakeRollersIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.shooterpivot.ShooterPivot;
import frc.robot.subsystems.shooterpivot.ShooterPivotIOSim;
import frc.robot.subsystems.shooterpivot.ShooterPivotIOTalonFX;
import frc.robot.subsystems.sotm.ShotCalculator;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.generated.TunerConstants;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretConstants;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.turret.TurretIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.utils.MappedXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public final MappedXboxController m_driverController =
      new MappedXboxController(ControllerConstants.kDriverControllerPort, "Driver");

  private final Telemetry logger =
      new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));

  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  private final AutoRoutines m_autoRoutines;

  private final Turret turret =
      new Turret(true, Utils.isSimulation() ? new TurretIOSim() : new TurretIOTalonFX());

  private final ShooterPivot shooterPivot =
      new ShooterPivot(
          true, Utils.isSimulation() ? new ShooterPivotIOSim() : new ShooterPivotIOTalonFX());
  private final Shooter shooter =
      new Shooter(true, Utils.isSimulation() ? new ShooterIOSim() : new ShooterIOTalonFX());

  private final IntakeRollers intakeRollers =
      new IntakeRollers(
          true, Utils.isSimulation() ? new IntakeRollersIOSim() : new IntakeRollersIOTalonFX());

  private final IntakePivot intakePivot =
      new IntakePivot(
          true, Utils.isSimulation() ? new IntakePivotIOSim() : new IntakePivotIOTalonFX());
  private final Indexer indexer =
      new Indexer(true, Utils.isSimulation() ? new IndexerIOSim() : new IndexerIOTalonFX());
  private final Feeder feeder =
      new Feeder(true, Utils.isSimulation() ? new FeederIOSim() : new FeederIOTalonFX());

  private final Vision vision =
      new Vision(
          drivetrain::addVisionMeasurement,
          new VisionIOPhotonVision(
              VisionConstants.frontRightCam, VisionConstants.robotToFrontRightCam),
          new VisionIOPhotonVision(VisionConstants.backLeftCam, VisionConstants.robotToBackLeftCam),
          new VisionIOPhotonVision(
              VisionConstants.backRightCam, VisionConstants.robotToBackRightCam),
          new VisionIOPhotonVision(
              VisionConstants.frontLeftCam, VisionConstants.robotToFrontLeftCam));

  private final ShotCalculator shotCalculator =
      new ShotCalculator(
          () -> drivetrain.getState().Pose,
          drivetrain::getFieldRelativeSpeeds,
          TurretConstants.driveBaseToTurret);
  private final Superstructure superstructure =
      new Superstructure(
          indexer,
          shooterPivot,
          shooter,
          intakeRollers,
          intakePivot,
          feeder,
          turret,
          shotCalculator,
          () -> drivetrain.getState().Pose);

  /// sim file for intakepivot needs to be added -- seems like its not been merged yet

  private AutoChooser autoChooser = new AutoChooser();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Configure the trigger bindings
    CommandScheduler.getInstance().registerSubsystem(drivetrain);
    m_autoRoutines =
        new AutoRoutines(
            drivetrain.createAutoFactory(drivetrain::trajLogger), drivetrain, superstructure);
    configureController();
    configureChoreoAutoChooser();
    if (Utils.isSimulation()) {
      SimMechs.getInstance().publishToNT();
    }
  }

  private void configureChoreoAutoChooser() {

    // Add options to the chooser

    autoChooser.addCmd("Wheel Radius Change", () -> drivetrain.wheelRadiusCharacterization(1));

    autoChooser.addRoutine("Top 1 Cycle Mid", m_autoRoutines::topMidNoClimbAuto);
    autoChooser.addRoutine("Top Depot Outpost", m_autoRoutines::topDepotOutpostAuto);

    SmartDashboard.putData("auto chooser", autoChooser);

    // Schedule the selected auto during the autonomous period
    RobotModeTriggers.autonomous().onTrue(autoChooser.selectedCommandScheduler());
  }

  private void configureController() {
    // Request to drive normally using input for both translation and rotation
    SwerveRequest.FieldCentric drive =
        new SwerveRequest.FieldCentric()
            .withDeadband(deadbandMultiplier * MaxSpeed)
            .withRotationalRate(deadbandMultiplier * MaxAngularRate);

    // Request to control translation, with rotation being controlled by a heading controller
    SwerveRequest.FieldCentricFacingAngle azimuth =
        new SwerveRequest.FieldCentricFacingAngle().withDeadband(deadbandMultiplier * MaxSpeed);

    // Heading controller to control azimuth rotations
    azimuth.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    azimuth.HeadingController.setPID(
        AzimuthTargets.aziKP, AzimuthTargets.aziKi, AzimuthTargets.aziKD);

    // Default Drive

    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(-m_driverController.getLeftY() * MaxSpeed)
                    .withVelocityY(-m_driverController.getLeftX() * MaxSpeed)
                    .withRotationalRate(-m_driverController.getRightX() * MaxAngularRate)));

    // Slow

    m_driverController
        .leftBumper()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    drive
                        .withVelocityX(-m_driverController.getLeftY() * SlowMaxSpeed)
                        .withVelocityY(-m_driverController.getLeftX() * SlowMaxSpeed)
                        .withRotationalRate(
                            -m_driverController.getTriggerAxes() * SlowMaxAngular)));

    // Bump

    m_driverController
        .a()
        .onTrue(
            drivetrain.applyRequest(
                azimuth
                    .withVelocityX(-m_driverController.getLeftY())
                    .withVelocityY(-m_driverController.getLeftX())
                    .withTargetDirection(AzimuthTargets.bump)));

    // Zero Swerve

    m_driverController.y().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    // Home?

    m_driverController.b().onTrue(superstructure.setState(Superstructure.StructureState.IDLE));

    // Jitter

    m_driverController
        .povUp()
        .onTrue(superstructure.setState(Superstructure.StructureState.JITTER_INTAKE));

    // FEED placeholder - povUp

    // Shoot

    m_driverController
        .povRight()
        .onTrue(superstructure.setState(Superstructure.StructureState.SHOOT));

    // Intake

    m_driverController
        .povDown()
        .onTrue(superstructure.setState(Superstructure.StructureState.INTAKE));

    // Climb placeholder - x

    // Bump Azi

    m_driverController
        .a()
        .onTrue(
            drivetrain.applyRequest(
                azimuth
                    .withVelocityX(-m_driverController.getRightY())
                    .withVelocityY(-m_driverController.getRightX())
                    .withTargetDirection(AzimuthTargets.bump)));

    // Cardinal Azimuth

    new Trigger(() -> (m_driverController.getRightY() > 0.5)) // Up
        .onTrue(
            drivetrain
                .applyRequest(
                    () ->
                        azimuth
                            .withVelocityX(-m_driverController.getLeftY() * MaxSpeed)
                            .withVelocityY(-m_driverController.getLeftX() * MaxSpeed)
                            .withTargetDirection(Rotation2d.kZero))
                .withTimeout(AzimuthTargets.timeout));

    new Trigger(() -> (m_driverController.getRightY() < -0.5)) // Down
        .onTrue(
            drivetrain
                .applyRequest(
                    () ->
                        azimuth
                            .withVelocityX(-m_driverController.getLeftY() * MaxSpeed)
                            .withVelocityY(-m_driverController.getLeftX() * MaxSpeed)
                            .withTargetDirection(new Rotation2d(Math.toRadians(180))))
                .withTimeout(AzimuthTargets.timeout));

    new Trigger(() -> (m_driverController.getRightX() > 0.5)) // Right
        .onTrue(
            drivetrain
                .applyRequest(
                    () ->
                        azimuth
                            .withVelocityX(-m_driverController.getLeftY() * MaxSpeed)
                            .withVelocityY(-m_driverController.getLeftX() * MaxSpeed)
                            .withTargetDirection(new Rotation2d(Math.toRadians(90))))
                .withTimeout(AzimuthTargets.timeout));

    new Trigger(() -> (m_driverController.getRightX() < -0.5)) // Left
        .onTrue(
            drivetrain
                .applyRequest(
                    () ->
                        azimuth
                            .withVelocityX(-m_driverController.getLeftY() * MaxSpeed)
                            .withVelocityY(-m_driverController.getLeftX() * MaxSpeed)
                            .withTargetDirection(new Rotation2d(Math.toRadians(270))))
                .withTimeout(AzimuthTargets.timeout));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public void periodic() {
    shotCalculator.periodic();
    superstructure.periodic();
  }
}
