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

  private static class AutoCommands {

    public void AutoCommands() { // void for now, fill subsystems
    }
  }
  public AutoRoutine MidMid2x() {
    final AutoRoutine routine = m_factory.newRoutine("MidMid2x");
    final AutoTrajectory midMid2x = routine.trajectory("MidMid2x");
    
    routine
        .active()
        .onTrue(midMid2x.resetOdometry().andThen(Commands.waitSeconds(0)).andThen(midMid2x.cmd()));
    midMid2x.atTime("Start").onTrue(m_superstructure.setState(StructureState.REV).andThen(Commands.waitSeconds(2)).andThen(m_superstructure.setState(StructureState.SHOOT)));
    midMid2x.atTime("Intake1").onTrue(m_superstructure.setState(StructureState.INTAKE));
    midMid2x.atTime("Home1").onTrue(m_superstructure.setState(StructureState.REV));
    midMid2x.atTime("Shoot1").onTrue(m_superstructure.setState(StructureState.SHOOT));
    midMid2x.atTime("Intake2").onTrue(m_superstructure.setState(StructureState.INTAKE));
    midMid2x.atTime("Home2").onTrue(m_superstructure.setState(StructureState.REV));
    midMid2x.atTime("Shoot2").onTrue(m_superstructure.setState(StructureState.SHOOT));

    return routine;
  }
}
