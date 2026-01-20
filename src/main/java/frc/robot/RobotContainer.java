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
import choreo.util.ChoreoAllianceFlipUtil;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.InternalButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.AutoRoutines;
import frc.robot.sim.SimMechs;
//import frc.robot.subsystems.Superstructure;
//import frc.robot.subsystems.Superstructure.StructureState;
import frc.robot.subsystems.shooterpivot.ShooterPivot;
import frc.robot.subsystems.shooterpivot.ShooterPivotConstants;
import frc.robot.subsystems.shooterpivot.ShooterPivotIOSim;
import frc.robot.subsystems.shooterpivot.ShooterPivotIOTalonFX;
import frc.robot.subsystems.intakerollers.IntakeRollers;
import frc.robot.subsystems.intakerollers.IntakeRollersIOSim;
import frc.robot.subsystems.intakerollers.IntakeRollersIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.generated.TunerConstants;
/* 
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.utils.MappedXboxController; 
*/
import frc.robot.utils.MappedXboxController;

import java.util.List;

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
      new MappedXboxController(ControllerConstants.kDriverControllerPort, "driver");
  public final MappedXboxController m_operatorController =
      new MappedXboxController(ControllerConstants.kOperatorControllerPort, "operator");

  private final Telemetry logger =
      new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));

  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  private final Shooter shooter =
      new Shooter(
          true, Utils.isSimulation() ? new ShooterIOSim() : new ShooterIOTalonFX());

  private final IntakeRollers intakeRollers =
      new IntakeRollers(
          true,
          Utils.isSimulation()
              ? new IntakeRollersIOSim()
              : new IntakeRollersIOTalonFX());

  private final ShooterPivot shooterPivot = 
      new ShooterPivot(
          true, 
          Utils.isSimulation() 
              ? new ShooterPivotIOSim() 
              : new ShooterPivotIOTalonFX());

private final IntakePivot intakePivot =
      new IntakePivot(
          true, Utils.isSimulation() ? new IntakePivotIOSim() : new IntakePivotIOTalonFX());

 
/* NOT DONE
  private final Superstructure superstructure =
      new Superstructure(
          shooterPivot, intakeRollers, shooter, () -> drivetrain.getState().Pose);
*/

 private final Vision vision =
      new Vision(
          drivetrain::addPhotonEstimate,
          Utils.isSimulation()
              ? new VisionIOPhotonVisionSim(
                  VisionConstants.leftCam,
                  VisionConstants.robotToLeftCam,
                  () -> drivetrain.getState().Pose)
              : new VisionIOPhotonVision(VisionConstants.leftCam, VisionConstants.robotToLeftCam),
          Utils.isSimulation()
              ? new VisionIOPhotonVisionSim(
                  VisionConstants.rightCam,
                  VisionConstants.robotToRightCam,
                  () -> drivetrain.getState().Pose)
              : new VisionIOPhotonVision(
                  VisionConstants.rightCam, VisionConstants.robotToRightCam));
 
  

  private final AutoRoutines m_autoRoutines;
  private AutoChooser autoChooser = new AutoChooser();

// DO WE NEED AUTO ALIGN???

  //private final Trigger autoAlignedTrigger =
      //new Trigger(() -> AutoAim.isInToleranceCoral(drivetrain.getState().Pose));

  ///private final InternalButton autoAlignRunning = new InternalButton();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureOperatorBinds();
    /* 
    m_autoRoutines =
        new AutoRoutines(
            drivetrain.createAutoFactory(drivetrain::trajLogger), drivetrain, superstructure) */;
    configureChoreoAutoChooser();
    CommandScheduler.getInstance().registerSubsystem(drivetrain);
    configureSwerve();
    /* 
    if (Utils.isSimulation()) {
      SimMechs.getInstance().publishToNT();
    }*/
  } 

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  public Pose2d getPose() {
    return drivetrain.getState().Pose;
  }
/* 
  public Pose2d getClosestAlignment() {
    return drivetrain
        .getState()
        .Pose
        .nearest(
            List.of(
                new Pose2d(7.197261810302734, 2.482149600982666, Rotation2d.fromDegrees(270)),
                ChoreoAllianceFlipUtil.flip(
                    new Pose2d(7.197261810302734, 2.482149600982666, Rotation2d.fromDegrees(270))),
                new Pose2d(7.228638648986816, 4.02569580078125, Rotation2d.fromDegrees(270)),
                ChoreoAllianceFlipUtil.flip(
                    new Pose2d(7.228638648986816, 4.02569580078125, Rotation2d.fromDegrees(270))),
                new Pose2d(7.197261810302734, 5.6082072257995605, Rotation2d.fromDegrees(270)),
                ChoreoAllianceFlipUtil.flip(
                    new Pose2d(
                        7.197261810302734, 5.6082072257995605, Rotation2d.fromDegrees(270)))));
  }
*/

  private void configureOperatorBinds() {
    return ;
    
    m_operatorController.a().onTrue(shooterPivot.setPosition(ShooterPivotConstants.stowPosition));
    m_operatorController.y().onTrue(shooter.setVelocity(15,15));
    m_operatorController.x().onTrue(intakeRollers.setVelocity(15));
    m_operatorController.b().onTrue(intakePivot.setPosition(0));
    
  }


  private void configureChoreoAutoChooser() {

    // Add options to the chooser
    autoChooser.addCmd("Wheel Radius Change", () -> drivetrain.wheelRadiusCharacterization(1));
    autoChooser.addRoutine("Mobility Left", m_autoRoutines::mobilityLeft);
    autoChooser.addRoutine("Mobility Right", m_autoRoutines::mobilityRight);
    SmartDashboard.putData("auto chooser", autoChooser);

    // Schedule the selected auto during the autonomous period
    RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
  }

  private void configureSwerve() {

    // Request to drive normally using input for both translation and rotation
    SwerveRequest.FieldCentric drive =
        new SwerveRequest.FieldCentric()
            .withDeadband(0.15 * MaxSpeed)
            .withRotationalRate(0.15 * MaxAngularRate);

    // Request to control translation, with rotation being controlled by a heading controller
    SwerveRequest.FieldCentricFacingAngle azimuth =
        new SwerveRequest.FieldCentricFacingAngle().withDeadband(0.15 * MaxSpeed);

    // Heading controller to control azimuth rotations
    azimuth.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    azimuth.HeadingController.setPID(6, 0, 0);

    // Default Swerve Command, run periodically every 20ms
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(-m_driverController.getLeftY() * MaxSpeed)
                    .withVelocityY(-m_driverController.getLeftX() * MaxSpeed)
                    .withRotationalRate(-m_driverController.getRightX() * MaxAngularRate)));

    m_driverController
        .a("Brake / Slow Mode")
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    drive
                        .withVelocityX(-m_driverController.getLeftY() * SlowMaxSpeed)
                        .withVelocityY(-m_driverController.getLeftX() * SlowMaxSpeed)
                        .withRotationalRate(-m_driverController.getRightX() * SlowMaxAngular)));

    m_driverController
        .leftBumper("Azimuth Left Source")
        .onTrue(
            drivetrain
                .applyRequest(
                    () ->
                        azimuth
                            .withVelocityY(-m_driverController.getLeftX() * MaxSpeed)
                            .withVelocityX(-m_driverController.getLeftY() * MaxSpeed)
                            .withTargetDirection(sourceLeft1))
                .withTimeout(aziTimeout));

    m_driverController
        .rightBumper("Azimuth Right Source")
        .onTrue(
            drivetrain
                .applyRequest(
                    () ->
                        azimuth
                            .withVelocityY(-m_driverController.getLeftX() * MaxSpeed)
                            .withVelocityX(-m_driverController.getLeftY() * MaxSpeed)
                            .withTargetDirection(sourceRight2))
                .withTimeout(aziTimeout));

    m_driverController
        .povUp("Processor Close, Climb Facing cage")
        .onTrue(
            drivetrain
                .applyRequest(
                    () ->
                        azimuth
                            .withVelocityY(-m_driverController.getLeftX() * MaxSpeed)
                            .withVelocityX(-m_driverController.getLeftY() * MaxSpeed)
                            .withTargetDirection(processorClose))
                .withTimeout(aziTimeout));

    m_driverController
        .povDown("Processor Far, Climb Facing DS")
        .onTrue(
            drivetrain
                .applyRequest(
                    () ->
                        azimuth
                            .withVelocityY(-m_driverController.getLeftX() * MaxSpeed)
                            .withVelocityX(-m_driverController.getLeftY() * MaxSpeed)
                            .withTargetDirection(processorFar))
                .withTimeout(aziTimeout));

    // Azimuth Barge Close

    // sets the heading to wherever the robot is facing
    // do this with the elevator side of the robot facing YOU
    m_driverController.y("Zero Heading").onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

  }

  public void periodic() {
    /*
    // Logger.recordOutput(
    // "Stick Angle Radians",
    // Math.atan2(m_driverController.getRightY(), m_driverController.getRightX()));
    // Logger.recordOutput(
    // "AutoAim/Targets/Coral",
    // Stream.of(CoralTargets.values())
    // .map((target) -> CoralTargets.getRobotTargetLocation(target.location))
    // .toArray(Pose2d[]::new));
    // // Log locations of all autoaim targets
    // Logger.recordOutput(
    // "AutoAim/Targets/Algae",
    // Stream.of(AlgaeIntakeTargets.values())
    // .map((target) -> AlgaeIntakeTargets.getRobotTargetLocation(target.location))
    // .toArray(Pose2d[]::new));
    //
    // Logger.recordOutput(
    // "AutoAim/Targets/SourceIntakes",
    // Stream.of(SourceIntakeTargets.values())
    // .map((target) -> SourceIntakeTargets.getRobotTargetLocation(target.location))
    // .toArray(Pose2d[]::new));
    //
    // Logger.recordOutput(
    // "AutoAim/CoralTarget",
    // CoralTargets.getClosestTarget(drivetrain.getState().Pose));
    // Logger.recordOutput(
    // "AutoAim/LeftHandedCoralTarget",
    // CoralTargets.getHandedClosestTarget(drivetrain.getState().Pose, true));
    // Logger.recordOutput(
    // "AutoAim/RightHandedCoralTarget",
    // CoralTargets.getHandedClosestTarget(drivetrain.getState().Pose, false));
    // Logger.recordOutput(
    // "AutoAim/NameOfLHCoralTarget",
    // CoralTargets.getHandedClosestTargetE(drivetrain.getState().Pose,
    // true).name());
    // Logger.recordOutput(
    // "AutoAim/NameOfRHCoralTarget",
    // CoralTargets.getHandedClosestTargetE(drivetrain.getState().Pose,
    // false).name());
    // Logger.recordOutput(
    // "AutoAim/AlgaeIntakeTarget",
    // AlgaeIntakeTargets.getClosestTarget(drivetrain.getState().Pose));
    */
    //superstructure.periodic();
  }
}