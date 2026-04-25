// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

  public AutoRoutine topBumpDirectionalIntake() {
    final AutoRoutine routine = m_factory.newRoutine("TopBumpDirectionalIntake");
    final AutoTrajectory topBumpDirectionalIntakeAuto =
        routine.trajectory("DepotBumpDirectionalIntake");
    routine
        .active()
        .onTrue(
            topBumpDirectionalIntakeAuto
                .resetOdometry()
                .andThen(topBumpDirectionalIntakeAuto.cmd()));

    topBumpDirectionalIntakeAuto
        .atTime("Intake")
        .onTrue(m_superstructure.setState(StructureState.INTAKE));

    topBumpDirectionalIntakeAuto
        .atTime("Jitter")
        .onTrue(m_superstructure.setState(StructureState.JITTER_AND_SHOOT));

    topBumpDirectionalIntakeAuto
        .atTime("Shoot")
        .onTrue(m_superstructure.setState(StructureState.SHOOT));

    return routine;
  }

  public AutoRoutine getdisrupted() {
    final AutoRoutine routine = m_factory.newRoutine("getdisruptedp1");
    final AutoTrajectory p1 = routine.trajectory("getdisruptedp1");
    final AutoTrajectory p2 = routine.trajectory("getdisruptedp2");

    routine.active().onTrue(p1.resetOdometry().andThen(p1.cmd()));

    p1.atTime("Intake").onTrue(m_superstructure.setState(StructureState.INTAKE));

    p1.doneDelayed(1).onTrue(p2.cmd());

    p2.atTime("StopIntake")
        .onTrue(m_superstructure.setState(StructureState.IDLE))
        .onTrue(m_superstructure.setState(StructureState.REV));

    p2.atTime("Shoot").onTrue(m_superstructure.setState(StructureState.SHOOT));

    return routine;
  }

  public AutoRoutine topBumpDirectionalIntakeSOTM() {
    final AutoRoutine routine = m_factory.newRoutine("TopBumpDirectionalIntakeSOTM");
    final AutoTrajectory topBumpDirectionalintakeSOTMAuto =
        routine.trajectory("DepotBumpDirectionalIntakeSOTM");
    routine
        .active()
        .onTrue(
            topBumpDirectionalintakeSOTMAuto
                .resetOdometry()
                .andThen(topBumpDirectionalintakeSOTMAuto.cmd()));

    topBumpDirectionalintakeSOTMAuto
        .atTime("Intake")
        .onTrue(m_superstructure.setState(StructureState.INTAKE));

    topBumpDirectionalintakeSOTMAuto
        .atTime("Jitter")
        .onTrue(m_superstructure.setState(StructureState.JITTER_AND_SHOOT));

    topBumpDirectionalintakeSOTMAuto
        .atTime("Shoot")
        .onTrue(m_superstructure.setState(StructureState.SHOOT));

    topBumpDirectionalintakeSOTMAuto
        .atTime("StopShooting")
        .onTrue(m_superstructure.setState(StructureState.IDLE));

    return routine;
  }

  public AutoRoutine closetobump() {
    final AutoRoutine routine = m_factory.newRoutine("tweaked");
    final AutoTrajectory topBumpDirectionalintakeSOTMAuto = routine.trajectory("tweaked");
    routine
        .active()
        .onTrue(
            topBumpDirectionalintakeSOTMAuto
                .resetOdometry()
                .andThen(topBumpDirectionalintakeSOTMAuto.cmd()));

    topBumpDirectionalintakeSOTMAuto
        .atTime("Intake")
        .onTrue(m_superstructure.setState(StructureState.INTAKE));

    topBumpDirectionalintakeSOTMAuto
        .atTime("Jitter")
        .onTrue(m_superstructure.setState(StructureState.JITTER_AND_SHOOT));

    topBumpDirectionalintakeSOTMAuto
        .atTime("Shoot")
        .onTrue(m_superstructure.setState(StructureState.SHOOT));

    return routine;
  }

  public AutoRoutine depotBumpSOTM() {
    final AutoRoutine routine = m_factory.newRoutine("DepotBumpSOTM");
    final AutoTrajectory topBumpDirectionalIntakeAuto = routine.trajectory("DepotBumpSOTM");
    routine
        .active()
        .onTrue(
            topBumpDirectionalIntakeAuto
                .resetOdometry()
                .andThen(m_superstructure.setState(StructureState.INTAKE))
                .andThen(topBumpDirectionalIntakeAuto.cmd()));

    topBumpDirectionalIntakeAuto
        .atTime("Intake")
        .onTrue(m_superstructure.setState(StructureState.INTAKE));

    topBumpDirectionalIntakeAuto
        .atTime("StopShooting")
        .onTrue(m_superstructure.setState(StructureState.IDLE));

    topBumpDirectionalIntakeAuto
        .atTime("Shoot2")
        .onTrue(m_superstructure.setState(StructureState.JITTER_AND_SHOOT));
    topBumpDirectionalIntakeAuto
        .atTime("Shoot1")
        .onTrue(m_superstructure.setState(StructureState.SHOOT));

    return routine;
  }

  public AutoRoutine bottomBumpDirectionalIntake() {
    final AutoRoutine routine = m_factory.newRoutine("BottomBumpDirectionalIntake");
    final AutoTrajectory topBumpDirectionalIntakeAuto =
        routine.trajectory("OutpostBumpDirectionalIntake");
    routine
        .active()
        .onTrue(
            topBumpDirectionalIntakeAuto
                .resetOdometry()
                .andThen(topBumpDirectionalIntakeAuto.cmd()));

    topBumpDirectionalIntakeAuto
        .atTime("Intake")
        .onTrue(m_superstructure.setState(StructureState.INTAKE));
    topBumpDirectionalIntakeAuto
        .atTime("StopIntake")
        .onTrue(
            m_superstructure
                .setState(StructureState.IDLE)
                .andThen(m_superstructure.setState(StructureState.REV)));

    topBumpDirectionalIntakeAuto
        .atTime("Shoot")
        .onTrue(
            m_superstructure
                .setState(StructureState.SHOOT)
                .andThen(Commands.waitSeconds(1))
                .andThen(m_superstructure.setState(StructureState.JITTER_AND_SHOOT)));

    return routine;
  }

  public AutoRoutine bottomBumpDirectionalIntakeWait() {
    final AutoRoutine routine = m_factory.newRoutine("BottomBumpDirectionalIntakeWait");
    final AutoTrajectory topBumpDirectionalIntakeAuto =
        routine.trajectory("OutpostBumpDirectionalIntake");
    routine
        .active()
        .onTrue(
            topBumpDirectionalIntakeAuto
                .resetOdometry()
                .andThen(Commands.waitSeconds(2))
                .andThen(topBumpDirectionalIntakeAuto.cmd()));

    topBumpDirectionalIntakeAuto
        .atTime("Intake")
        .onTrue(m_superstructure.setState(StructureState.INTAKE));

    topBumpDirectionalIntakeAuto
        .atTime("Shoot")
        .onTrue(
            m_superstructure
                .setState(StructureState.SHOOT)
                .andThen(Commands.waitSeconds(1))
                .andThen(m_superstructure.setState(StructureState.JITTER_AND_SHOOT)));

    return routine;
  }

  public AutoRoutine preloadAuto() {
    final AutoRoutine routine = m_factory.newRoutine("DepotShootPreload");
    routine
        .active()
        .onTrue(
            m_superstructure
                .setState(StructureState.REV)
                .andThen(Commands.waitSeconds(2))
                .andThen(m_superstructure.setState(StructureState.SHOOT)));
    return routine;
  }

  public AutoRoutine doubleLoop() {
    final AutoRoutine routine = m_factory.newRoutine("doubleLoop");
    final AutoTrajectory doubleLoop = routine.trajectory("DoubleLoop");

    routine.active().onTrue(doubleLoop.resetOdometry().andThen(doubleLoop.cmd()));

    doubleLoop.atTime("Intake").onTrue(m_superstructure.setState(StructureState.INTAKE));

    doubleLoop
        .atTime("StopIntake")
        .onTrue(
            m_superstructure
                .setState(StructureState.IDLE)
                .andThen(m_superstructure.setState(StructureState.REV)));

    doubleLoop
        .atTime("Shoot")
        .onTrue(
            m_superstructure
                .setState(StructureState.SHOOT)
                .andThen(Commands.waitSeconds(.5))
                .andThen(m_superstructure.setState(StructureState.JITTER_AND_SHOOT)));

    return routine;
  }

  public Pose2d getInitialPose(String trajectoryName) {
    var trajectory = Choreo.loadTrajectory(trajectoryName);
    Pose2d initialPose = trajectory.get().getInitialPose(false).get();
    return initialPose;
  }

  private boolean isRedAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red);
  }
}
