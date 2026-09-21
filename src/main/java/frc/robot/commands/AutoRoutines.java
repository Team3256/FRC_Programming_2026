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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.StructureState;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;

public class AutoRoutines {

  private static final String waitkey = "Auto/WaitKey";

  private static final double waitkeydefault = 0.5;

  private final AutoFactory m_factory;

  // subsystems
  private final Superstructure m_superstructure;
  private final CommandSwerveDrivetrain m_drivetrain;

  public AutoRoutines(
      AutoFactory factory, CommandSwerveDrivetrain drivetrain, Superstructure superstructure) {
    m_factory = factory;
    m_drivetrain = drivetrain; // subsystems
    m_superstructure = superstructure;

    SmartDashboard.setDefaultNumber(waitkey, waitkeydefault);
  }

  private Command tunableWait(String key, double defaultSeconds) {
    return Commands.defer(
        () -> Commands.waitSeconds(SmartDashboard.getNumber(key, defaultSeconds)), Set.of());
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

  public AutoRoutine feed() {
    final AutoRoutine routine = m_factory.newRoutine("feed");
    final AutoTrajectory topBumpDirectionalIntakeAuto = routine.trajectory("feed");
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

    return routine;
  }

  public AutoRoutine stop() {
    final AutoRoutine routine = m_factory.newRoutine("stop");
    final AutoTrajectory topBumpDirectionalIntakeAuto = routine.trajectory("stop");
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

  public AutoRoutine centerdepotOnly() {
    final AutoRoutine routine = m_factory.newRoutine("CenterDepotOnly");
    final AutoTrajectory topBumpDirectionalIntakeAuto = routine.trajectory("CenterDepotOnly");
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
        .atTime("Shoot")
        .onTrue(m_superstructure.setState(StructureState.JITTER_AND_SHOOT));

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

    p2.atTime("Shoot").onTrue(m_superstructure.setState(StructureState.JITTER_AND_SHOOT));

    return routine;
  }

  public AutoRoutine getdisrupteddeep() {
    final AutoRoutine routine = m_factory.newRoutine("getdisruptedp1");

    final AutoTrajectory p1 = routine.trajectory("getdisruptedp1");
    final AutoTrajectory p2 = routine.trajectory("outpostdeep");

    routine.active().onTrue(p1.resetOdometry().andThen(p1.cmd()));

    p1.doneDelayed(1).onTrue(p2.cmd());
    p2.atTime("Intake").onTrue(m_superstructure.setState(StructureState.INTAKE));
    p2.atTime("StopIntake")
        .onTrue(m_superstructure.setState(StructureState.IDLE))
        .onTrue(m_superstructure.setState(StructureState.REV));

    p2.atTime("Shoot").onTrue(m_superstructure.setState(StructureState.JITTER_AND_SHOOT));

    return routine;
  }

  public AutoRoutine depotstealAutoUP() {
    final AutoRoutine routine = m_factory.newRoutine("stealAuto");
    final AutoTrajectory stealAuto = routine.trajectory("steal");
    final AutoTrajectory stealP2 = routine.trajectory("stealp2");

    routine.active().onTrue(stealAuto.resetOdometry().andThen(stealAuto.cmd()));

    stealAuto.atTime("Intake").onTrue(m_superstructure.setState(StructureState.INTAKE));

    stealAuto.done().onTrue(tunableWait(waitkey, waitkeydefault).andThen(stealP2.cmd()));

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

  public AutoRoutine depotstealautoDOWN() {
    final AutoRoutine routine = m_factory.newRoutine("stealAuto");
    final AutoTrajectory stealAuto = routine.trajectory("steal");
    final AutoTrajectory stealP2 = routine.trajectory("stealp2v2");

    routine.active().onTrue(stealAuto.resetOdometry().andThen(stealAuto.cmd()));

    stealAuto.atTime("Intake").onTrue(m_superstructure.setState(StructureState.INTAKE));

    stealAuto.done().onTrue(tunableWait(waitkey, waitkeydefault).andThen(stealP2.cmd()));

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

    stealAuto.done().onTrue(tunableWait(waitkey, waitkeydefault).andThen(stealP2.cmd()));

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

  public AutoRoutine outpostStealAutoDOWN() {
    final AutoRoutine routine = m_factory.newRoutine("outpostStealAuto");
    final AutoTrajectory stealAuto = routine.trajectory("Outpoststeal");
    final AutoTrajectory stealP2 = routine.trajectory("Outpoststealp2v2");

    routine.active().onTrue(stealAuto.resetOdometry().andThen(stealAuto.cmd()));

    stealAuto.atTime("Intake").onTrue(m_superstructure.setState(StructureState.INTAKE));

    stealAuto.done().onTrue(tunableWait(waitkey, waitkeydefault).andThen(stealP2.cmd()));

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

  public AutoRoutine bottomBumpDirectionalIntakeWaitDEEP() {
    final AutoRoutine routine = m_factory.newRoutine("BottomBumpDirectionalIntakeWait");
    final AutoTrajectory topBumpDirectionalIntakeAuto =
        routine.trajectory("OutpostBumpDirectionalIntakeDeep");
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
        .onTrue(m_superstructure.setState(StructureState.IDLE))
        .onTrue(m_superstructure.setState(StructureState.REV));

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

  public void updateField2d(Field2d field2d, List<String> trajectoryNames) {

    ArrayList<Pose2d> poseList = new ArrayList<>();
    for (String t : trajectoryNames) {
      var trajectory = Choreo.loadTrajectory(t);

      if (trajectory.isEmpty()) {
        poseList.add(new Pose2d(0, 0, Rotation2d.kZero));
      } else {
        var poses = trajectory.get().getPoses();

        for (int i = 0; i < poses.length; i += 5) {
          poseList.add(poses[i]);
        }
      }
    }
    field2d.setRobotPose(poseList.get(0));
    System.out.println(poseList.size());

    if (poseList.size() == 1) {
      return;
    }

    field2d.getObject("traj").setPoses(poseList);
  }

  private boolean isRedAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red);
  }
}
