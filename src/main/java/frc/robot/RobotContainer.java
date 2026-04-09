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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
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
import frc.robot.subsystems.led.Led;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.shooterpivot.ShooterPivot;
import frc.robot.subsystems.shooterpivot.ShooterPivotIOSim;
import frc.robot.subsystems.shooterpivot.ShooterPivotIOTalonFX;
import frc.robot.subsystems.sotm.ShotCalculator;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
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
import java.util.Set;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {

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

  private final Led led = new Led();

  private final Superstructure superstructure =
      new Superstructure(
          led,
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

  private static final String[] BUMP_TRAJECTORIES =
      new String[] {
        "OutpostRedBumpForwardCross",
        "OutpostRedBumpBackCross",
        "OutpostBlueBumpForwardCross",
        "OutpostBlueBumpBackCross",
        "DepotRedBumpForwardCross",
        "DepotRedBumpBackCross",
        "DepotBlueBumpForwardCross",
        "DepotBlueBumpBackCross"
      };

  @SuppressWarnings("unchecked")
  private Supplier<Command>[] BUMP_CMDS;

  public RobotContainer() {
    CommandScheduler.getInstance().registerSubsystem(drivetrain);
    m_autoRoutines =
        new AutoRoutines(
            drivetrain.createAutoFactory(drivetrain::trajLogger), drivetrain, superstructure);

    BUMP_CMDS =
        (Supplier<Command>[])
            new Supplier[] {
              m_autoRoutines::outpostRedBumpForwardCrossCmd,
              m_autoRoutines::outpostRedBumpBackCrossCmd,
              m_autoRoutines::outpostBlueBumpForwardCrossCmd,
              m_autoRoutines::outpostBlueBumpBackCrossCmd,
              m_autoRoutines::depotRedBumpForwardCrossCmd,
              m_autoRoutines::depotRedBumpBackCrossCmd,
              m_autoRoutines::depotBlueBumpForwardCrossCmd,
              m_autoRoutines::depotBlueBumpBackCrossCmd
            };

    configureSwerve();
    configureChoreoAutoChooser();
    configureOperatorBinds();
    if (Utils.isSimulation()) {
      SimMechs.getInstance().publishToNT();
    }
  }

  public Led getLed() {
    return led;
  }

  private void configureOperatorBinds() {
    m_operatorController.a().onTrue(superstructure.setState(Superstructure.StructureState.SHOOT));
    m_operatorController.b().onTrue(superstructure.setState(Superstructure.StructureState.IDLE));
    m_operatorController.x().onTrue(superstructure.setState(Superstructure.StructureState.INTAKE));
    m_operatorController
        .y()
        .onTrue(superstructure.setState(Superstructure.StructureState.JITTER_INTAKE));

    m_operatorController
        .povUp()
        .onTrue(
            Commands.runOnce(
                () ->
                    TurretConstants.driveBaseToTurret =
                        TurretConstants.driveBaseToTurret.plus(
                            new Transform2d(0, .1, Rotation2d.kZero))));
    m_operatorController
        .povDown()
        .onTrue(
            Commands.runOnce(
                () ->
                    TurretConstants.driveBaseToTurret =
                        TurretConstants.driveBaseToTurret.plus(
                            new Transform2d(0, -.1, Rotation2d.kZero))));
    m_operatorController
        .povRight()
        .onTrue(
            Commands.runOnce(
                () ->
                    TurretConstants.driveBaseToTurret =
                        TurretConstants.driveBaseToTurret.plus(
                            new Transform2d(0.1, 0, Rotation2d.kZero))));
    m_operatorController
        .povLeft()
        .onTrue(
            Commands.runOnce(
                () ->
                    TurretConstants.driveBaseToTurret =
                        TurretConstants.driveBaseToTurret.plus(
                            new Transform2d(0.1, 0, Rotation2d.kZero))));
  }

  private void configureChoreoAutoChooser() {
    autoChooser.addCmd("Wheel Radius Change", () -> drivetrain.wheelRadiusCharacterization(1));
    autoChooser.addRoutine(
        "Top 1 Cycle Directional Intake", m_autoRoutines::topBumpDirectionalIntake);

    autoChooser.addRoutine(
        "Bottom Directional Intake", m_autoRoutines::bottomBumpDirectionalIntake);
    autoChooser.addRoutine("Top Depot Outpost", m_autoRoutines::topTrenchDepotOutpostHub);
    autoChooser.addRoutine("DepotSteal", m_autoRoutines::depotStealAuto);
    autoChooser.addRoutine("OutpostSteal", m_autoRoutines::outpostStealAuto);
    autoChooser.addRoutine("SOTM", m_autoRoutines::depotBumpSOTM);
    autoChooser.addRoutine("Preload", m_autoRoutines::preloadAuto);
    autoChooser.addRoutine("Depot SOTM Directional", m_autoRoutines::topBumpDirectionalIntakeSOTM);
    SmartDashboard.putData("auto chooser", autoChooser);
    RobotModeTriggers.autonomous().onTrue(autoChooser.selectedCommandScheduler());
  }

  private void configureSwerve() {
    SwerveRequest.FieldCentric drive =
        new SwerveRequest.FieldCentric()
            .withDeadband(deadbandMultiplier * MaxSpeed)
            .withRotationalRate(deadbandMultiplier * MaxAngularRate);

    SwerveRequest.FieldCentricFacingAngle azimuth =
        new SwerveRequest.FieldCentricFacingAngle().withDeadband(deadbandMultiplier * MaxSpeed);

    azimuth.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    azimuth.HeadingController.setPID(
        AzimuthTargets.aziKP, AzimuthTargets.aziKi, AzimuthTargets.aziKD);

    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(
                        -(Math.signum(m_driverController.getLeftY())
                                * Math.pow(m_driverController.getLeftY(), 2))
                            * MaxSpeed)
                    .withVelocityY(
                        -(Math.signum(m_driverController.getLeftX())
                                * Math.pow(m_driverController.getLeftX(), 2))
                            * MaxSpeed)
                    .withRotationalRate(-m_driverController.getRightX() * MaxAngularRate)));

    m_driverController
        .leftBumper()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    drive
                        .withVelocityX(
                            -(Math.signum(m_driverController.getLeftY())
                                    * Math.pow(m_driverController.getLeftY(), 2))
                                * SlowMaxSpeed)
                        .withVelocityY(
                            -(Math.signum(m_driverController.getLeftX())
                                    * Math.pow(m_driverController.getLeftX(), 2))
                                * SlowMaxSpeed)
                        .withRotationalRate(-m_driverController.getRightX() * SlowMaxAngular)));

    m_driverController.povRight().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    m_driverController.povUp().onTrue(superstructure.addShootMultiplier(.02));
    m_driverController.povDown().onTrue(superstructure.addShootMultiplier(-0.02));

    m_driverController
        .rightTrigger()
        .onTrue(superstructure.setState(Superstructure.StructureState.UNJAM))
        .onFalse(superstructure.setState(Superstructure.StructureState.IDLE));

    m_driverController.a().onTrue(superstructure.setState(Superstructure.StructureState.SHOOT));
    m_driverController.b().onTrue(superstructure.setState(Superstructure.StructureState.IDLE));
    m_driverController.x().onTrue(superstructure.setState(Superstructure.StructureState.INTAKE));
    m_driverController
        .y()
        .onTrue(superstructure.setState(Superstructure.StructureState.JITTER_INTAKE));

    drivetrain.registerTelemetry(logger::telemeterize);

    SmartDashboard.putData("Choose command", selectBumpCrossCommand());
    SmartDashboard.putData("Run", selectBumpCrossCommand());
  }

  public Command selectBumpCrossCommand() {
    return Commands.defer(
        () -> {
          Translation2d current = drivetrain.getState().Pose.getTranslation();

          int closestIdx = 0;
          double closestDist = Double.MAX_VALUE;
          for (int i = 0; i < BUMP_TRAJECTORIES.length; i++) {
            double dist =
                current.getDistance(
                    m_autoRoutines.getInitialPose(BUMP_TRAJECTORIES[i]).getTranslation());
            if (dist < closestDist) {
              closestDist = dist;
              closestIdx = i;
            }
          }

          return buildBumpCrossSequence(BUMP_TRAJECTORIES[closestIdx], BUMP_CMDS[closestIdx]);
        },
        Set.of(drivetrain));
  }

  public Command buildBumpCrossSequence(String routineName, Supplier<Command> routineCmd) {
    return drivetrain
        .pidToPose(() -> m_autoRoutines.getInitialPose(routineName))
        .until(() -> closeEnoughToStart(m_autoRoutines.getInitialPose(routineName)))
        .andThen(routineCmd.get());
  }

  public boolean closeEnoughToStart(Pose2d targetStartPose) {
    Pose2d current = drivetrain.getState().Pose;
    double distance = current.getTranslation().getDistance(targetStartPose.getTranslation());
    return distance < 0.15;
  }

  public void periodic() {
    shotCalculator.periodic();
    superstructure.periodic();

    Logger.recordOutput("Turret Offsets", TurretConstants.driveBaseToTurret);
  }
}
