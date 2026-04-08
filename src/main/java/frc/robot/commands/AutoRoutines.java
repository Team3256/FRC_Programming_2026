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

  public AutoRoutine depotBumpSOTM() {
    final AutoRoutine routine = m_factory.newRoutine("DepotBumpSOTM");
    final AutoTrajectory topBumpDirectionalIntakeAuto = routine.trajectory("DepotBumpSOTM");
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
        .atTime("StopShooting")
        .onTrue(m_superstructure.setState(StructureState.IDLE));

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

  public AutoRoutine depotShootPreloadAuto() {
    final AutoRoutine routine = m_factory.newRoutine("DepotShootPreload");
    final AutoTrajectory shootPreloadAuto = routine.trajectory("DepotShootPreload");
    routine.active().onTrue(shootPreloadAuto.resetOdometry().andThen(shootPreloadAuto.cmd()));
    shootPreloadAuto
        .atTime("Shoot")
        .onTrue(
            m_superstructure
                .setState(StructureState.REV)
                .andThen(Commands.waitSeconds(2))
                .andThen(m_superstructure.setState(StructureState.SHOOT)));

    shootPreloadAuto.atTime("StopShooting").onTrue(m_superstructure.setState(StructureState.IDLE));

    return routine;
  }

  public AutoRoutine outpostShootPreloadAuto() {
    final AutoRoutine routine = m_factory.newRoutine("OutpostShootPreload");
    final AutoTrajectory shootPreloadAuto = routine.trajectory("OutpostShootPreload");
    routine.active().onTrue(shootPreloadAuto.resetOdometry().andThen(shootPreloadAuto.cmd()));
    shootPreloadAuto
        .atTime("Shoot")
        .onTrue(
            m_superstructure
                .setState(StructureState.REV)
                .andThen(Commands.waitSeconds(2))
                .andThen(m_superstructure.setState(StructureState.SHOOT)));

    shootPreloadAuto.atTime("StopShooting").onTrue(m_superstructure.setState(StructureState.IDLE));

    return routine;
  }

  public AutoRoutine topMidNoClimbDepotAuto() {
    final AutoRoutine routine = m_factory.newRoutine("toTopBumpMidDepotNoCimb");
    final AutoTrajectory topMidNoClimbDepot = routine.trajectory("TopBumpMidDepotNoCimb");

    routine.active().onTrue(topMidNoClimbDepot.resetOdometry().andThen(topMidNoClimbDepot.cmd()));

    topMidNoClimbDepot.atTime("Intake").onTrue(m_superstructure.setState(StructureState.INTAKE));

    topMidNoClimbDepot
        .atTime("StopIntake")
        .onTrue(
            m_superstructure
                .setState(StructureState.IDLE)
                .andThen(m_superstructure.setState(StructureState.REV)));

    topMidNoClimbDepot
        .atTime("Shoot")
        .onTrue(
            Commands.waitSeconds(1.5)
                .andThen(m_superstructure.setState(StructureState.SHOOT))
                .andThen(Commands.waitSeconds(1))
                .andThen(m_superstructure.setState(StructureState.JITTER_AND_SHOOT)));

    return routine;
  }

  public AutoRoutine depotStealAuto() {
    final AutoRoutine routine = m_factory.newRoutine("stealAuto");
    final AutoTrajectory stealAuto = routine.trajectory("Depotsteal");
    final AutoTrajectory stealP2 = routine.trajectory("Depotstealp2");

    routine.active().onTrue(stealAuto.resetOdometry().andThen(stealAuto.cmd()));

    stealAuto.atTime("Intake").onTrue(m_superstructure.setState(StructureState.INTAKE));

    stealAuto.doneDelayed(2).onTrue(stealP2.cmd());

    stealP2
        .atTime("StopIntake")
        .onTrue(
            m_superstructure
                .setState(StructureState.IDLE)
                .andThen(m_superstructure.setState(StructureState.REV)));

    stealP2
        .atTime("Shoot")
        .onTrue(
            Commands.waitSeconds(1.5)
                .andThen(m_superstructure.setState(StructureState.SHOOT))
                .andThen(Commands.waitSeconds(1))
                .andThen(m_superstructure.setState(StructureState.JITTER_AND_SHOOT)));

    return routine;
  }

  public AutoRoutine outpostStealAuto() {
    final AutoRoutine routine = m_factory.newRoutine("outpostStealAuto");
    final AutoTrajectory stealAuto = routine.trajectory("Outpoststeal");
    final AutoTrajectory stealP2 = routine.trajectory("Outpoststealp2");

    routine.active().onTrue(stealAuto.resetOdometry().andThen(stealAuto.cmd()));

    stealAuto.atTime("Intake").onTrue(m_superstructure.setState(StructureState.INTAKE));

    stealAuto.doneDelayed(2).onTrue(stealP2.cmd());

    stealP2
        .atTime("StopIntake")
        .onTrue(
            m_superstructure
                .setState(StructureState.IDLE)
                .andThen(m_superstructure.setState(StructureState.REV)));

    stealP2
        .atTime("Shoot")
        .onTrue(
            Commands.waitSeconds(1.5)
                .andThen(m_superstructure.setState(StructureState.SHOOT))
                .andThen(Commands.waitSeconds(1))
                .andThen(m_superstructure.setState(StructureState.JITTER_AND_SHOOT)));

    return routine;
  }

  public Command outpostRedBumpForwardCrossCmd() {
    return m_factory.trajectoryCmd("OutpostRedBumpForwardCross");
  }

  public Command outpostRedBumpBackCrossCmd() {
    return m_factory.trajectoryCmd("OutpostRedBumpBackCross");
  }

  public Command outpostBlueBumpForwardCrossCmd() {
    return m_factory.trajectoryCmd("OutpostBlueBumpForwardCross");
  }

  public Command outpostBlueBumpBackCrossCmd() {
    return m_factory.trajectoryCmd("OutpostBlueBumpBackCross");
  }

  public Command depotRedBumpForwardCrossCmd() {
    return m_factory.trajectoryCmd("DepotRedBumpForwardCross");
  }

  public Command depotRedBumpBackCrossCmd() {
    return m_factory.trajectoryCmd("DepotRedBumpBackCross");
  }

  public Command depotBlueBumpForwardCrossCmd() {
    return m_factory.trajectoryCmd("DepotBlueBumpForwardCross");
  }

  public Command depotBlueBumpBackCrossCmd() {
    return m_factory.trajectoryCmd("DepotBlueBumpBackCross");
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
