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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.StructureState;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class AutoRoutines {

  private final AutoFactory m_factory;

  // subsystems
  private final Superstructure m_superstructure;
  private final CommandSwerveDrivetrain m_drivetrain;

  public AutoRoutines(
      AutoFactory factory, CommandSwerveDrivetrain drivetrain, Superstructure superstructure) {
    m_factory = factory;
    m_drivetrain = drivetrain; // subsystems
    m_superstructure = superstructure;
  }

  public AutoRoutine forwardTraj() {
    final AutoRoutine routine = m_factory.newRoutine("forwardTest");
    final AutoTrajectory forward = routine.trajectory("forward");
    routine.active().whileTrue(Commands.print("yest").repeatedly());
    routine.active().onTrue(forward.resetOdometry().andThen(forward.cmd()));
    return routine;
  }

  public Command forwardCmd() {
    return Commands.sequence(
        m_factory.resetOdometry("forward"), m_factory.trajectoryCmd("forward"));
  }

  public AutoRoutine topBumpMidHub() {
    final AutoRoutine routine = m_factory.newRoutine("TopBumpMidHub");
    final AutoTrajectory topMidNoClimb = routine.trajectory("TopBumpMidHub");
    routine.active().onTrue(topMidNoClimb.resetOdometry().andThen(topMidNoClimb.cmd()));

    topMidNoClimb.atTime("Intake").onTrue(m_superstructure.setState(StructureState.INTAKE));
    topMidNoClimb
        .atTime("StopIntake")
        .onTrue(
            m_superstructure
                .setState(StructureState.IDLE)
                .andThen(m_superstructure.setState(StructureState.REV)));

    topMidNoClimb
        .atTime("Shoot")
        .onTrue(
            Commands.waitSeconds(1.5)
                .andThen(m_superstructure.setState(StructureState.SHOOT))
                .andThen(Commands.waitSeconds(1))
                .andThen(m_superstructure.setState(StructureState.JITTER_AND_SHOOT)));

    return routine;
  }

  public AutoRoutine topTrenchDepotOutpostHub() {
    final AutoRoutine routine = m_factory.newRoutine("TopTrenchDepotOutpostHub");
    final AutoTrajectory topDepotOutpost = routine.trajectory("TopTrenchDepotOutpostHub");
    final AutoTrajectory outpostPt2 = routine.trajectory("OutPostMove");

    routine.active().onTrue(topDepotOutpost.resetOdometry().andThen(topDepotOutpost.cmd()));

    topDepotOutpost.atTime("Intake").onTrue(m_superstructure.setState(StructureState.INTAKE));

    topDepotOutpost.atTime("Rev").onTrue(m_superstructure.setState(StructureState.REV));

    topDepotOutpost
        .atTime("Shoot")
        .onTrue(
            Commands.waitSeconds(1.5)
                .andThen(m_superstructure.setState(StructureState.SHOOT))
                .andThen(Commands.waitSeconds(1))
                .andThen(m_superstructure.setState(StructureState.JITTER_AND_SHOOT)));

    topDepotOutpost.doneDelayed(3).onTrue(outpostPt2.cmd());

    return routine;
  }

 public AutoRoutine topBumpDirectionalIntake() {
    final AutoRoutine routine = m_factory.newRoutine("TopBumpDirectionalIntake");
    final AutoTrajectory topBumpDirectionalIntakeAuto = routine.trajectory("TopBumpDirectionalIntake");
    routine.active().onTrue(topBumpDirectionalIntakeAuto.resetOdometry().andThen(topBumpDirectionalIntakeAuto.cmd()));

    topBumpDirectionalIntakeAuto.atTime("Intake").onTrue(m_superstructure.setState(StructureState.INTAKE));
    topBumpDirectionalIntakeAuto
        .atTime("StopIntake")
        .onTrue(
            m_superstructure
                .setState(StructureState.IDLE)
                .andThen(m_superstructure.setState(StructureState.REV)));

       topBumpDirectionalIntakeAuto
        .atTime("Shoot")
        .onTrue(
            Commands.waitSeconds(1.5)
                .andThen(m_superstructure.setState(StructureState.SHOOT))
                .andThen(Commands.waitSeconds(1))
                .andThen(m_superstructure.setState(StructureState.JITTER_AND_SHOOT)));   

  return routine;
  }


}
