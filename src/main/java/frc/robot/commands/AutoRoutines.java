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


  public AutoRoutine topMidNoClimb() {
    final AutoRoutine routine = m_factory.newRoutine("topmidnoclimb");
    final AutoTrajectory topMid = routine.trajectory("TopMidNoClimb");
    routine.active().onTrue(topMid.resetOdometry().andThen(topMid.cmd()));

    topMid.atTime("Intake").onTrue(m_superstructure.setState(StructureState.INTAKE));
    topMid.atTime("StopIntake").onTrue(m_superstructure.setState(StructureState.IDLE));
    topMid.atTime("Rev").onTrue(m_superstructure.setState(StructureState.REV));
    topMid.atTime("Shoot").onTrue(m_superstructure.setState(StructureState.SHOOT));

    return routine;

    }

  private static class AutoCommands {

    public void AutoCommands() { // void for now, fill subsystems
    }
  }
}
