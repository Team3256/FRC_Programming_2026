// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems;

import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.FieldConstants;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intakepivot.IntakePivot;
import frc.robot.subsystems.intakerollers.IntakeRollers;
import frc.robot.subsystems.led.Led;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooterpivot.ShooterPivot;
import frc.robot.subsystems.sotm.ShotCalculator;
import frc.robot.subsystems.turret.Turret;
import frc.robot.utils.LoggedTracer;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Superstructure {
  public enum StructureState {
    INTAKE,
    SHOOT,
    SHOOT_AND_INTAKE,
    JITTER_INTAKE,
    JITTER_AND_SHOOT,
    HOME,
    UNJAM,
    IDLE,
    CLIMB,
    CANCEL_ALL,
    REV,
  }

  private StructureState state = StructureState.IDLE;
  private StructureState prevState = StructureState.IDLE;

  private Map<StructureState, Trigger> stateTriggers = new HashMap<StructureState, Trigger>();

  private Map<StructureState, Trigger> prevStateTriggers = new HashMap<StructureState, Trigger>();

  private final Timer stateTimer = new Timer();

  private final Indexer indexer;
  private final ShooterPivot shooterPivot;
  private final Shooter shooter;
  private final IntakeRollers intakeRollers;
  private final IntakePivot intakePivot;
  private final Feeder feeder;
  private final Turret turret;

  private final Led led;

  private final ShotCalculator shotCalculator;

  private final Supplier<Pose2d> robotPoseSupplier;

  private final Trigger targetBlueHub = new Trigger(this::targetBlueHub);
  private final Trigger targetRedHub = new Trigger(this::targetRedHub);

  private final Trigger feedTopCorner =
      targetBlueHub.or(targetRedHub).negate().and(this::isRobotTopHalf);
  private final Trigger feedBottomCorner =
      targetBlueHub.or(targetRedHub).or(feedTopCorner).negate();

  private final Translation2d topCorner = new Translation2d(1.5, 6.8);
  private final Translation2d bottomCorner = new Translation2d(1.5, 1.5);

  private double velMultiplier = .95;

  private Pose2d target =
      new Pose2d(FieldConstants.Hub.topCenterPoint.toTranslation2d(), Rotation2d.kZero);

  public Superstructure(
      Led led,
      Indexer indexer,
      ShooterPivot shooterPivot,
      Shooter shooter,
      IntakeRollers intakeRollers,
      IntakePivot intakePivot,
      Feeder feeder,
      Turret turret,
      ShotCalculator shotCalculator,
      Supplier<Pose2d> robotPoseSupplier) {
    this.led = led;
    this.indexer = indexer;
    this.shooterPivot = shooterPivot;
    this.shooter = shooter;
    this.intakeRollers = intakeRollers;
    this.intakePivot = intakePivot;
    this.feeder = feeder;
    this.turret = turret;
    this.shotCalculator = shotCalculator;
    this.robotPoseSupplier = robotPoseSupplier;

    stateTimer.start();

    for (StructureState state : StructureState.values()) {
      stateTriggers.put(state, new Trigger(() -> this.state == state));
    }
    for (StructureState state : StructureState.values()) {
      prevStateTriggers.put(state, new Trigger(() -> this.prevState == state));
    }

    configStateTransitions();
    configureLed();
  }

  public void configStateTransitions() {

    new Trigger(DriverStation::isEnabled)
        .debounce(.05)
        .whileTrue(turret.pointToPose(shotCalculator::getLookaheadPose, () -> target));

    targetBlueHub.onTrue(changeTarget(FieldConstants.Hub.topCenterPoint.toTranslation2d()));

    targetRedHub.onTrue(changeTarget(FieldConstants.Hub.oppTopCenterPoint.toTranslation2d()));

    feedTopCorner.onTrue(
        changeTarget(
            () -> getAllianceBlue() ? topCorner : ChoreoAllianceFlipUtil.flip(bottomCorner)));
    feedBottomCorner.onTrue(
        changeTarget(
            () -> getAllianceBlue() ? bottomCorner : ChoreoAllianceFlipUtil.flip(topCorner)));

    targetRedHub
        .or(targetBlueHub)
        .and(DriverStation::isEnabled)
        .whileTrue(shooterPivot.shootHub(shotCalculator::getDistance));
    feedTopCorner
        .or(feedBottomCorner)
        .and(DriverStation::isEnabled)
        .whileTrue(shooterPivot.feedCorner(shotCalculator::getDistance));

    stateTriggers
        .get(StructureState.SHOOT)
        .or(stateTriggers.get(StructureState.SHOOT_AND_INTAKE))
        .or(stateTriggers.get(StructureState.JITTER_AND_SHOOT))
        .and(targetRedHub.or(targetBlueHub))
        .onTrue(shooter.shootHub(shotCalculator::getDistance, () -> velMultiplier));
    stateTriggers
        .get(StructureState.SHOOT)
        .or(stateTriggers.get(StructureState.SHOOT_AND_INTAKE))
        .or(stateTriggers.get(StructureState.JITTER_AND_SHOOT))
        .and(feedTopCorner.or(feedBottomCorner))
        .onTrue(shooter.feedCorner(shotCalculator::getDistance));

    stateTriggers
        .get(StructureState.SHOOT)
        .or(stateTriggers.get(StructureState.SHOOT_AND_INTAKE))
        .and(shooter.reachedVelocity)
        .or(stateTriggers.get(StructureState.JITTER_AND_SHOOT))
        .debounce(.2)
        .onTrue(indexer.startShooting())
        .onTrue(feeder.startFeeding());

    stateTriggers
        .get(StructureState.SHOOT)
        .and(prevStateTriggers.get(StructureState.INTAKE))
        .or(
            stateTriggers
                .get(StructureState.INTAKE)
                .and(
                    prevStateTriggers
                        .get(StructureState.SHOOT)
                        .or(prevStateTriggers.get(StructureState.JITTER_AND_SHOOT))))
        .onTrue(this.setState(StructureState.SHOOT_AND_INTAKE));

    stateTriggers
        .get(StructureState.INTAKE)
        .or(stateTriggers.get(StructureState.SHOOT_AND_INTAKE))
        .onTrue(intakeRollers.setVoltage(8))
        .onTrue(intakePivot.goToGroundIntake());

    stateTriggers
        .get(StructureState.JITTER_INTAKE)
        .or(stateTriggers.get(StructureState.JITTER_AND_SHOOT))
        .onTrue(intakePivot.jitterIntake())
        .onTrue(intakeRollers.setVoltage(8));

    stateTriggers
        .get(StructureState.JITTER_INTAKE)
        .and(
            prevStateTriggers
                .get(StructureState.SHOOT)
                .or(prevStateTriggers.get(StructureState.SHOOT_AND_INTAKE)))
        .onTrue(this.setState(StructureState.JITTER_AND_SHOOT));

    stateTriggers.get(StructureState.UNJAM).onTrue(feeder.unjam());

    stateTriggers
        .get(StructureState.IDLE)
        .onTrue(intakeRollers.off())
        .onTrue(shooter.off())
        .onTrue(indexer.off())
        .onTrue(feeder.off())
        .onTrue(intakePivot.off());

    // Kills all subsystems
    stateTriggers
        .get(StructureState.CANCEL_ALL)
        .onTrue(intakeRollers.off())
        .onTrue(intakePivot.off())
        .onTrue(shooter.off())
        .onTrue(shooterPivot.off())
        .onTrue(indexer.off())
        .onTrue(feeder.off());

    stateTriggers
        .get(StructureState.HOME)
        .onTrue(intakePivot.setPosition(0))
        .onTrue(shooterPivot.setPosition(0));

    //
    stateTriggers
        .get(StructureState.REV)
        .whileTrue(shooter.shootHub(shotCalculator::getDistance, () -> velMultiplier));
  }

  private void configureLed() {
    RobotModeTriggers.teleop()
        .and(stateTriggers.get(StructureState.INTAKE))
        .onTrue(led.strobeOrange());

    RobotModeTriggers.teleop()
        .and(
            stateTriggers
                .get(StructureState.SHOOT)
                .or(stateTriggers.get(StructureState.JITTER_AND_SHOOT)))
        .onTrue(led.setGreen());

    RobotModeTriggers.teleop()
        .and(stateTriggers.get(StructureState.SHOOT_AND_INTAKE))
        .onTrue(led.halfOrangeHalfGreen());

    RobotModeTriggers.teleop().and(stateTriggers.get(StructureState.IDLE)).onTrue(led.setRed());
  }

  // call manually
  public void periodic() {

    Logger.recordOutput("Superstructure/State", this.state.toString());
    Logger.recordOutput("Superstructure/PrevState", this.prevState.toString());
    Logger.recordOutput("Superstructure/StateTime", this.stateTimer.get());

    Logger.recordOutput("Superstructure/VelMultiplier", this.velMultiplier);

    //    Logger.recordOutput(
    //        "Superstructure/Hub",
    //        stateTriggers.get(StructureState.SHOOT).and(targetRedHub.or(targetBlueHub)));
    //    Logger.recordOutput("Superstructure/TargetRedHub", targetRedHub);
    //    Logger.recordOutput("Superstructure/TargetBlueHub", targetBlueHub);
    //    Logger.recordOutput("Superstructure/FeedTopCorner", feedTopCorner);
    //    Logger.recordOutput("Superstructure/FeedBottomCorner", feedBottomCorner);
    Logger.recordOutput("Superstructure/Target", target);

    LoggedTracer.record(this.getClass().getSimpleName());
  }

  private Command changeTarget(Translation2d target) {
    return changeTarget(() -> target);
  }

  private Command changeTarget(Supplier<Translation2d> target) {
    return Commands.runOnce(
            () -> {
              this.target = new Pose2d(target.get(), Rotation2d.kZero);
              shotCalculator.setTarget(target.get());
            })
        .ignoringDisable(true);
  }

  private boolean getAllianceBlue() {
    return DriverStation.getAlliance()
        .orElse(DriverStation.Alliance.Blue)
        .equals(DriverStation.Alliance.Blue);
  }

  private boolean targetBlueHub() {
    return (robotPoseSupplier.get().getX() < 4 && getAllianceBlue());
  }

  private boolean targetRedHub() {
    return (robotPoseSupplier.get().getX() > 12.5 && !getAllianceBlue());
  }

  private boolean isRobotTopHalf() {
    return robotPoseSupplier.get().getY() > 4;
  }

  public Command setState(StructureState state) {
    return Commands.runOnce(
        () -> {
          this.prevState = this.state == state ? this.prevState : this.state;
          this.state = state;
          this.stateTimer.restart();
        });
  }

  public Command addShootMultiplier(double amt) {
    return Commands.runOnce(() -> this.velMultiplier += amt);
  }

  public StructureState getState() {
    return this.state;
  }

  public StructureState getPrevState() {
    return this.prevState;
  }
}
