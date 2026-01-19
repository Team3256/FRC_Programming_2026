// Copyright (c) 2026 FRC 3256
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
  private final CommandSwerveDrivetrain m_drivetrain;
  private final Superstructure m_superstructure;

  public AutoRoutines(
      AutoFactory factory, CommandSwerveDrivetrain drivetrain, Superstructure superstructure) {
    m_factory = factory;
    m_drivetrain = drivetrain;
    m_superstructure = superstructure;
  }

  public AutoRoutine mobilityLeft() {
    final AutoRoutine routine = m_factory.newRoutine("mobilityLeft");
    final AutoTrajectory mobilityTop = routine.trajectory("MobilityTop");
    routine.active().onTrue(mobilityTop.resetOdometry().andThen(mobilityTop.cmd()));
    return routine;
  }

  public AutoRoutine mobilityRight() {
    final AutoRoutine routine = m_factory.newRoutine("mobilityRight");
    final AutoTrajectory mobilityBottom = routine.trajectory("MobilityBottom");
    routine.active().onTrue(mobilityBottom.resetOdometry().andThen(mobilityBottom.cmd()));
    return routine;
  }
}