// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems;

import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.FieldConstants;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intakepivot.IntakePivot;

import frc.robot.subsystems.intakerollers.IntakeRollers;

import frc.robot.subsystems.shooter.Shooter;

import frc.robot.subsystems.shooterpivot.ShooterPivot;
import frc.robot.utils.LoggedTracer;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Superstructure {
  public enum StructureState {
    INTAKE,
    SHOOT,
    HOME,
    IDLE,
    CLIMB,
    CANCEL_ALL,
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
  private final Climb climb;
  private final Feeder feeder; 

  private final Supplier<Pose2d> robotPoseSupplier;

  public Superstructure(
      Indexer indexer,
      ShooterPivot shooterPivot,
      Shooter shooter,
      IntakeRollers intakeRollers,
      IntakePivot intakePivot,
      Climb climb,
      Feeder feeder,
    //  ShotCalculator shotCalculator;
      Supplier<Pose2d> robotPoseSupplier) {
    this.indexer = indexer;
    this.shooterPivot = shooterPivot;
    this.shooter = shooter;
    this.intakeRollers = intakeRollers;
    this.intakePivot = intakePivot;
    this.climb = climb;
    this.feeder = feeder;
   // this.shotCalculator = shotCalculator; 
    this.robotPoseSupplier = robotPoseSupplier;
  
// intake --> indexer --> feeder --> shooter

    stateTimer.start();

    for (StructureState state : StructureState.values()) {
      stateTriggers.put(state, new Trigger(() -> this.state == state));
    }
    for (StructureState state : StructureState.values()) {
      prevStateTriggers.put(state, new Trigger(() -> this.prevState == state));
    }

    configStateTransitions();
  }

  public void configStateTransitions() {
    stateTriggers.get(StructureState.IDLE);

    // shoot fuel, dk how shooterpivot will work
    stateTriggers.get(StructureState.SHOOT).onTrue(shooter.setVoltage(12));
      
    // intake fuel with rollers, unsure how pivot will work yet....
    stateTriggers.get(StructureState.INTAKE).onTrue(intakeRollers.setVoltage(12));

    // climb
    stateTriggers.get(StructureState.CLIMB).onTrue(climb.setPosition(0.5));


    stateTriggers.get(StructureState.IDLE)
        .onTrue(intakeRollers.off())
        .onTrue(shooter.off());

    // Kills all subsystems
    stateTriggers
        .get(StructureState.CANCEL_ALL)
        .onTrue(climb.off())
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
    
  }

  // call manually
  public void periodic() {
    Logger.recordOutput("Superstructure/State", this.state.toString());
    Logger.recordOutput("Superstructure/PrevState", this.prevState.toString());
    Logger.recordOutput("Superstructure/StateTime", this.stateTimer.get());

    LoggedTracer.record(this.getClass().getSimpleName());
  }

  public Command setState(StructureState state) {
    return Commands.runOnce(
        () -> {
          this.prevState = this.state == state ? this.prevState : this.state;
          this.state = state;
          this.stateTimer.restart();
        });
  }

  public StructureState getState() {
    return this.state;
  }

  public StructureState getPrevState() {
    return this.prevState;
  }

}