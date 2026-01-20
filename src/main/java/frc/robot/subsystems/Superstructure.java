// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.
/* 
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utils.LoggedTracer;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class Superstructure {
  public enum StructureState {
    IDLE,
    SHOOT,
    INTAKE,
    PRESOURCE,
    SOURCE,
    BARGE,
    SCORE_ALGAE,
    CLIMB,
    PROCESSOR,
    SCORE_CORAL,
    PREHOME,
    HOME,
    CANCEL_ALL,
    AUTO
  }

  public static enum ManipulatorSide {
    LEFT,
    RIGHT
  }

  private ManipulatorSide manipulatorSide = ManipulatorSide.RIGHT;

  private StructureState state = StructureState.IDLE;
  private StructureState prevState = StructureState.IDLE;

  private Map<StructureState, Trigger> stateTriggers = new HashMap<StructureState, Trigger>();

  private Map<StructureState, Trigger> prevStateTriggers = new HashMap<StructureState, Trigger>();

  private final Trigger rightManipulatorSide =
      new Trigger(() -> this.manipulatorSide == ManipulatorSide.RIGHT);

  private final Timer stateTimer = new Timer();

 

  public Superstructure(Elevator elevator, EndEffector endEffector, Arm arm) {
    this.elevator = elevator;
    this.endEffector = endEffector;
    this.arm = arm;

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
    stateTriggers.get(StructureState.IDLE).onTrue(endEffector.coralOff());
    // Move elevator and reef to L1, no safety limits since arm is still safe
    stateTriggers
        .get(StructureState.L1)
        .onTrue(elevator.toReefLevel(0))
        .onTrue(arm.toReefLevel(0, () -> true));

    stateTriggers.get(StructureState.CLIMB).onTrue(elevator.toHome()).onTrue(arm.toClimb());

    // L2 and L3 are same arm position so they are put together, once again no safety limits
    stateTriggers.get(StructureState.L2).onTrue(elevator.toReefLevel(1));
    stateTriggers.get(StructureState.L3).onTrue(elevator.toReefLevel(2));
    stateTriggers
        .get(StructureState.L2)
        .or(stateTriggers.get(StructureState.L3))
        .onTrue(arm.toReefLevel(1, () -> true));

    // L4 reef level, no safety limits
    stateTriggers
        .get(StructureState.L4)
        .onTrue(elevator.toReefLevel(3))
        .and(elevator.reachedPosition)
        .debounce(.03)
        .onTrue(arm.toReefLevel(2, rightManipulatorSide));

    // Scoring coral, depending on previous state it changes endEffector velocity
    stateTriggers
        .get(StructureState.SCORE_CORAL)
        .and(prevStateTriggers.get(StructureState.L1))
        .onTrue(endEffector.setL1Velocity(rightManipulatorSide));
    stateTriggers
        .get(StructureState.SCORE_CORAL)
        .and(prevStateTriggers.get(StructureState.L2).or(prevStateTriggers.get(StructureState.L3)))
        .onTrue(endEffector.setL2L3Velocity(rightManipulatorSide));
    stateTriggers
        .get(StructureState.SCORE_CORAL)
        .and(prevStateTriggers.get(StructureState.L4))
        .onTrue(endEffector.setL4Voltage(rightManipulatorSide));

    // Dealgae levels, no safety limits (yet, since they might need to be retuned)
    stateTriggers
        .get(StructureState.DEALGAE_L2)
        .onTrue(elevator.toDealgaeLevel(0))
        .onTrue(arm.toDealgaeLevel(0, () -> true));

    stateTriggers
        .get(StructureState.DEALGAE_L3)
        .onTrue(elevator.toDealgaeLevel(1))
        .onTrue(arm.toDealgaeLevel(1, rightManipulatorSide));
    stateTriggers
        .get(StructureState.DEALGAE_L2)
        .or(stateTriggers.get(StructureState.DEALGAE_L3))
        .onTrue(endEffector.setAlgaeIntakeVelocity());

    // Arm needs to wrap 180, so elevator has to be safe before we fully move
    stateTriggers
        .get(StructureState.PRESOURCE)
        .onTrue(arm.toSourceLevel())
        .and(arm.isSafePosition)
        .onTrue(this.setState(StructureState.SOURCE));
    // We can move towards the direction up to a safe position if the elevator is not safe yet
    // Once the elevator reaches source position, we start move the arm around
    stateTriggers
        .get(StructureState.SOURCE)
        .onTrue(elevator.setPosition(ElevatorConstants.sourcePosition.in(Rotations)))
        .onTrue(endEffector.setSourceVelocity())
        .and(endEffector.coralBeamBreak)
        .onTrue(this.setState(StructureState.PREHOME));

    // Random filler for now
    stateTriggers
        .get(StructureState.BARGE)
        .onTrue(elevator.toBargePosition())
        .onTrue(arm.toBargeLevel(rightManipulatorSide));

    stateTriggers
        .get(StructureState.PROCESSOR)
        .onTrue(elevator.toProcessorPosition())
        .and(elevator.reachedPosition)
        .debounce(.04) // wait two loop times
        .onTrue(arm.toProcessorLevel());

    stateTriggers.get(StructureState.SCORE_ALGAE).onTrue(endEffector.setAlgaeOuttakeVoltage());

    // Turn coral motor off (helpful for transitioning from SCORE_CORAL), do not turn algae motor
    // off since you might be holding one
    stateTriggers.get(StructureState.PREHOME).onTrue(endEffector.coralOff());

    // Different version of prehome specifically for transitioning from SOURCE, because the arm
    // needs to move a specific direction

    stateTriggers
        .get(StructureState.PREHOME)
        .and(prevStateTriggers.get(StructureState.SCORE_ALGAE))
        .onTrue(endEffector.algaeOff());

    stateTriggers
        .get(StructureState.PREHOME)
        .and(prevStateTriggers.get(StructureState.SCORE_CORAL))
        .onTrue(arm.toHome())
        .and(arm.reachedPosition)
        .debounce(.025)
        .onTrue(this.setState(StructureState.HOME));

    // Since everything else is non-source and arm doesn't need to be towards the bellypan, you can
    // assume that moving the arm towards home is safe and that you don't need to move the elevator.
    stateTriggers
        .get(StructureState.PREHOME)
        .and(prevStateTriggers.get(StructureState.SCORE_CORAL).negate())
        .onTrue(arm.toHome())
        .and(arm.isSafePosition)
        .onTrue(this.setState(StructureState.HOME));

    // Once arm is safe, the elevator can also home, once everything is done we can go to the IDLE
    // state.
    // As a safety feature, the HOME state is only valid if the previous state was PREHOME ensuring
    // that you don't skip steps.

    stateTriggers
        .get(StructureState.HOME)
        .and(prevStateTriggers.get(StructureState.PREHOME))
        .onTrue(elevator.toHome())
        .onTrue(arm.toHome())
        .and(arm.reachedPosition)
        .and(elevator.reachedPosition)
        .onTrue(this.setState(StructureState.IDLE));

    stateTriggers
        .get(StructureState.CANCEL_ALL)
        .onTrue(elevator.off())
        .onTrue(arm.off())
        .onTrue(endEffector.algaeOff())
        .onTrue(endEffector.coralOff());
  }

  public Trigger coralBeamBreak() {
    return endEffector.coralBeamBreak;
  }

  public Trigger algaeBeamBreak() {
    return endEffector.algaeBeamBreak;
  }

  public Trigger climbState() {
    return stateTriggers.get(StructureState.CLIMB);
  }

  // call manually
  public void periodic() {
    Logger.recordOutput(
        "Superstructure/ManipulatorSide", this.manipulatorSide.toString()); // TODO: remove
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

  public Command setManipulatorSide(ManipulatorSide side) {
    return Commands.runOnce(
        () -> {
          this.manipulatorSide = side;
          // set manipulator side
        });
  }
}
  */