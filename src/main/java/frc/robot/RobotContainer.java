// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.subsystems.swerve.SwerveConstants.*;

import choreo.Choreo;
import choreo.auto.AutoChooser;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
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
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveConstants.AzimuthTargets;
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
  public final MappedXboxController m_operatorController =
      new MappedXboxController(ControllerConstants.kOperatorControllerPort, "Operator");

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

    
final Field2d m_field = new Field2d();

  /// sim file for intakepivot needs to be added -- seems like its not been merged yet

  private AutoChooser autoChooser = new AutoChooser();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    SmartDashboard.putData("Field", m_field);
    m_field.getObject("TopBumpCross").setPoses(
    Choreo.loadTrajectory("TopBumpCross").get().getPoses()
);
    m_field.getObject("BottomBumpCross").setPoses(
    Choreo.loadTrajectory("BottomBumpCross").get().getPoses()
);
    // Configure the trigger bindings
    CommandScheduler.getInstance().registerSubsystem(drivetrain);
    m_autoRoutines =
        new AutoRoutines(
            drivetrain.createAutoFactory(drivetrain::trajLogger), drivetrain, superstructure);
    configureSwerve();
    configureChoreoAutoChooser();
    configureOperatorBinds();
    if (Utils.isSimulation()) {
      SimMechs.getInstance().publishToNT();
    }
  }

  private void configureOperatorBinds() {

    m_operatorController.a().onTrue(superstructure.setState(Superstructure.StructureState.SHOOT));
    m_operatorController.b().onTrue(superstructure.setState(Superstructure.StructureState.IDLE));
    m_operatorController.x().onTrue(superstructure.setState(Superstructure.StructureState.INTAKE));
    m_operatorController
        .y()
        .onTrue(superstructure.setState(Superstructure.StructureState.JITTER_INTAKE));
  }

  private void configureChoreoAutoChooser() {

    // Add options to the chooser

    autoChooser.addCmd("Wheel Radius Change", () -> drivetrain.wheelRadiusCharacterization(1));

    autoChooser.addRoutine("Top 1 Cycle Mid", m_autoRoutines::topBumpMidHub);
    autoChooser.addRoutine("Top Depot Outpost", m_autoRoutines::topTrenchDepotOutpostHub);

    SmartDashboard.putData("auto chooser", autoChooser);

    // Schedule the selected auto during the autonomous period
    RobotModeTriggers.autonomous().onTrue(autoChooser.selectedCommandScheduler());
  }

  private void configureSwerve() {

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

    // Default Swerve Command, run periodically every 20ms
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(-m_driverController.getLeftY() * MaxSpeed)
                    .withVelocityY(-m_driverController.getLeftX() * MaxSpeed)
                    .withRotationalRate(-m_driverController.getRightX() * MaxAngularRate)));

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

    m_driverController
        .rightBumper()
        .onTrue(
            drivetrain.applyRequest(
                azimuth
                    .withVelocityX(-m_driverController.getLeftY())
                    .withVelocityY(-m_driverController.getLeftX())
                    .withTargetDirection(AzimuthTargets.bump)));

    // sets the heading to wherever the robot is facing
    m_driverController.y().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    //  new Trigger(() -> isAbove == true &&
    // m_driverController.a().getAsBoolean()).whileTrue(drivetrain.pidToPose(() ->
    // SwerveConstants.BumpTargets.TOP_BUMP));
    // new Trigger(() -> isAbove == false &&
    // m_driverController.a().getAsBoolean()).whileTrue(drivetrain.pidToPose(() ->
    // SwerveConstants.BumpTargets.BOTTOM_BUMP));

    Command topSequence =
        drivetrain
            .pidToPose(() -> SwerveConstants.BumpTargets.TOP_BUMP)
            .until(() -> closeEnoughToStart(m_autoRoutines.getInitialPose("TopBumpCross")))
            .andThen(m_autoRoutines.topBumpCrossCmd());

    Command bottomSequence =
        drivetrain
            .pidToPose(() -> SwerveConstants.BumpTargets.BOTTOM_BUMP)
            .until(() -> closeEnoughToStart(m_autoRoutines.getInitialPose("BottomBumpCross")))
            .andThen(m_autoRoutines.bottomBumpCrossCmd());

    m_driverController
        .a()
        .onTrue(
            Commands.either(
                topSequence,
                bottomSequence,
                () -> drivetrain.getState().Pose.getY() > 4.042979717254639));

    SmartDashboard.putData("My TopCommand", topSequence);
    SmartDashboard.putData("My BottomCommand", bottomSequence);

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  private boolean closeEnoughToStart(Pose2d targetStartPose) {
    Pose2d current = drivetrain.getState().Pose;

    double distance = current.getTranslation().getDistance(targetStartPose.getTranslation());

    return distance < 0.15; // 15 cm tolerance
  }

  public void periodic() {
    shotCalculator.periodic();
    superstructure.periodic();
    m_field.setRobotPose(drivetrain.getState().Pose);
    ;
  }
}
