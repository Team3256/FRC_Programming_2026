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
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class AutoRoutines {

  private final AutoFactory m_factory;

  // subsystems

  private final AutoCommands m_autoCommands;

  private final CommandSwerveDrivetrain m_drivetrain;

  public AutoRoutines(
      AutoFactory factory, AutoCommands mAutoCommands, CommandSwerveDrivetrain drivetrain) {
    m_factory = factory;
    m_autoCommands = mAutoCommands;
    m_drivetrain = drivetrain; // subsystems
  }

  public AutoRoutine forward() {
    final AutoRoutine routine = m_factory.newRoutine("forward");
    final AutoTrajectory forward = routine.trajectory("forward");
    routine.active().onTrue(forward.resetOdometry().andThen(forward().cmd()));
    return routine;
  }

  public AutoRoutine depotNoClimb() {
    final AutoRoutine routine = m_factory.newRoutine("depotOutpost"); // no climb
    final AutoTrajectory depot = routine.trajectory("depotClimb");
    routine
        .active()
        .onTrue(depot.resetOdometry().andThen(Commands.waitSeconds(2)).andThen(depot.cmd()));
    // depot.atTimeBeforeEnd(0.5).onTrue(m_superstructure)
    return routine;
  }

  private static class AutoCommands {

    public void AutoCommands() { // void for now, fill subsystems
    }
  }
}
