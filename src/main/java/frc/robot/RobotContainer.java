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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.ControllerConstants;
import frc.robot.sim.SimMechs;
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

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public final CommandXboxController m_driverController =
      new CommandXboxController(ControllerConstants.kDriverControllerPort);
  public final CommandXboxController m_operatorController =
      new CommandXboxController(ControllerConstants.kOperatorControllerPort);

  private final Telemetry logger =
      new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));

  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  private final Shooter shooter;
  private final ShooterPivot shooterPivot;
  private final ShotCalculator shotCalculator;
  private final IntakePivot groundIntakePivot;
  private final IntakeRollers groundIntake;

  /// sim file for intakepivot needs to be added -- seems like its not been merged yet

  private AutoChooser autoChooser = new AutoChooser();

  //  private final Limelight limelight = new Limelight("limelight");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    if (Utils.isSimulation()) {
      shooter = new Shooter(false, new ShooterIOSim());
      shooterPivot = new ShooterPivot(true, new ShooterPivotIOSim());
    } else {
      shooter = new Shooter(false, new ShooterIOTalonFX());
      shooterPivot = new ShooterPivot(true, new ShooterPivotIOTalonFX());
    } 

    if (Utils.isSimulation()) {
      groundIntakePivot = new IntakePivot(true, new IntakePivotIOSim());
      groundIntake = new IntakeRollers(true, new IntakeRollersIOSim());
    } else {
      groundIntakePivot = new IntakePivot(true, new IntakePivotIOTalonFX());
      groundIntake = new IntakeRollers(true, new IntakeRollersIOTalonFX());
    }
    

    shotCalculator = new ShotCalculator(drivetrain);

    // Configure the trigger bindings
    configureOperatorBinds();
    configureChoreoAutoChooser();
    CommandScheduler.getInstance().registerSubsystem(drivetrain);
    configureSwerve();
    if (Utils.isSimulation()) {
      SimMechs.getInstance().publishToNT();
    }
  }

  private void configureOperatorBinds() {

    
  //  m_operatorController
   //     .a()
     //   .onTrue(shooter.setVoltage(12));  

   m_operatorController
        .a()
        .onTrue(shooterPivot.setVoltage(12));

    m_operatorController
       .b()
       .onTrue(groundIntakePivot.setVoltage(12));

   // m_operatorController
     //   .b()
     //   .onTrue(groundIntake.setVoltage(12));
  }

  private void configureChoreoAutoChooser() {

    // Add options to the chooser
    autoChooser.addCmd("Wheel Radius Change", () -> drivetrain.wheelRadiusCharacterization(1));

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
        .a()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    drive
                        .withVelocityX(-m_driverController.getLeftY() * SlowMaxSpeed)
                        .withVelocityY(-m_driverController.getLeftX() * SlowMaxSpeed)
                        .withRotationalRate(-m_driverController.getRightX() * SlowMaxAngular)));

    // sets the heading to wherever the robot is facing
    m_driverController.y().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public void periodic() {}
}
