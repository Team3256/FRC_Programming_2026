// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.StructureState;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class AutoRoutines {

  private final AutoFactory m_factory;

  // subsystems
  private final Superstructure m_superstructure;

  private final AutoCommands m_autoCommands;

  private final CommandSwerveDrivetrain m_drivetrain;

  public AutoRoutines(
      AutoFactory factory,
      AutoCommands mAutoCommands,
      CommandSwerveDrivetrain drivetrain,
      Superstructure superstructure) {
    m_factory = factory;
    m_autoCommands = mAutoCommands;
    m_drivetrain = drivetrain; // subsystems
    m_superstructure = superstructure;
  }

  public AutoRoutine forward() {
    final AutoRoutine routine = m_factory.newRoutine("forward");
    final AutoTrajectory forward = routine.trajectory("forward");
    routine.active().onTrue(forward.resetOdometry().andThen(forward().cmd()));
    return routine;
  }

  public AutoRoutine depotNoClimb() {
    final AutoRoutine routine = m_factory.newRoutine("depotOutpost"); // no climb
    final AutoTrajectory depot = routine.trajectory("depot");
    routine
        .active()
        .onTrue(depot.resetOdometry().andThen(Commands.waitSeconds(2)).andThen(depot.cmd()));
    depot
        .atTime("Start")
        .onTrue(
            m_superstructure
                .setState(StructureState.REV)
                .andThen(Commands.waitSeconds(2))
                .andThen(m_superstructure.setState(StructureState.SHOOT)));
    depot.atTime("Intake").onTrue(m_superstructure.setState(StructureState.INTAKE));
    depot
        .atTime("Shoot")
        .onTrue(
            m_superstructure
                .setState(StructureState.REV)
                .andThen(Commands.waitSeconds(2))
                .andThen(m_superstructure.setState(StructureState.SHOOT)));
    depot.atTime("Shoot2").onTrue(m_superstructure.setState(StructureState.SHOOT));

    return routine;
  }

  public AutoRoutine depotClimb() {
    final AutoRoutine routine = m_factory.newRoutine("depotOutpostClimb");
    final AutoTrajectory depotClimb = routine.trajectory("depotClimb");
    routine
        .active()
        .onTrue(
            depotClimb.resetOdometry().andThen(Commands.waitSeconds(2)).andThen(depotClimb.cmd()));
    depotClimb
        .atTime("Start")
        .onTrue(
            m_superstructure
                .setState(StructureState.REV)
                .andThen(Commands.waitSeconds(2))
                .andThen(m_superstructure.setState(StructureState.SHOOT)));
    depotClimb.atTime("Intake").onTrue(m_superstructure.setState(StructureState.INTAKE));
    depotClimb
        .atTime("Shoot")
        .onTrue(
            m_superstructure
                .setState(StructureState.REV)
                .andThen(Commands.waitSeconds(2))
                .andThen(m_superstructure.setState(StructureState.SHOOT)));
    depotClimb.atTime("Shoot2").onTrue(m_superstructure.setState(StructureState.SHOOT));
    depotClimb.atTime("Climb").onTrue(m_superstructure.setState(StructureState.CLIMB));

    return routine;
  }

  private static class AutoCommands {

    public void AutoCommands() { // void for now, fill subsystems
    }
  }
}
