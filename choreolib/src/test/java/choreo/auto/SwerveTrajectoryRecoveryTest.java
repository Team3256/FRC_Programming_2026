// Copyright (c) Choreo contributors

package choreo.auto;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import choreo.trajectory.EventMarker;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import choreo.trajectory.TrajectoryTestHelper;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SchedulerMaker;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;
import org.junit.jupiter.api.Test;

public class SwerveTrajectoryRecoveryTest {
  @Test
  public void recoveryPausesProgressAndTimeTriggers() {
    assert HAL.initialize(500, 0);
    CommandScheduler scheduler = SchedulerMaker.make();
    AtomicReference<Pose2d> robotPose = new AtomicReference<>(Pose2d.kZero);
    AtomicReference<SwerveSample> commandedSample = new AtomicReference<>();
    AtomicBoolean followCommandedPose = new AtomicBoolean(false);
    AtomicInteger markerCount = new AtomicInteger();
    Consumer<SwerveSample> controller =
        sample -> {
          commandedSample.set(sample);
          if (followCommandedPose.get()) {
            robotPose.set(sample.getPose());
          }
        };

    AutoFactory factory =
        new AutoFactory(robotPose::get, robotPose::set, controller, false, new Subsystem() {})
            .withSwerveTrajectoryRecovery(
                new SwerveTrajectoryRecoveryConfig(0.25, Math.PI / 4, 0.05, Math.PI / 36));

    Trajectory<SwerveSample> baseTrajectory =
        TrajectoryTestHelper.linearTrajectory(
            "recovery",
            Pose2d.kZero,
            new Pose2d(1.0, 0.0, Rotation2d.kZero),
            1.0,
            SwerveSample.class);
    Trajectory<SwerveSample> trajectory =
        new Trajectory<>(
            baseTrajectory.name(),
            baseTrajectory.samples(),
            baseTrajectory.splits(),
            List.of(new EventMarker(0.6, "marker")));
    AutoRoutine routine = factory.newRoutine("recovery");
    AutoTrajectory autoTrajectory = factory.trajectory(trajectory, routine, true);
    autoTrajectory.atTime("marker").onTrue(Commands.runOnce(markerCount::incrementAndGet));

    Command routineCommand = routine.cmd();
    Command trajectoryCommand = autoTrajectory.cmd();

    SimHooks.pauseTiming();
    try {
      enableAutonomous();
      scheduler.schedule(routineCommand);
      scheduler.schedule(trajectoryCommand);
      scheduler.run();

      SimHooks.stepTiming(0.4);
      scheduler.run();

      SwerveSample recoveryTarget = commandedSample.get();
      assertTrue(scheduler.isScheduled(trajectoryCommand));
      assertTrue(recoveryTarget.t > 0.25);
      assertEquals(0.0, recoveryTarget.getChassisSpeeds().vxMetersPerSecond, 1e-9);
      assertEquals(0.0, recoveryTarget.getChassisSpeeds().vyMetersPerSecond, 1e-9);
      assertEquals(0.0, recoveryTarget.getChassisSpeeds().omegaRadiansPerSecond, 1e-9);

      SimHooks.stepTiming(1.0);
      scheduler.run();

      assertTrue(scheduler.isScheduled(trajectoryCommand));
      assertEquals(recoveryTarget.t, commandedSample.get().t, 1e-9);
      assertEquals(0, markerCount.get());

      followCommandedPose.set(true);
      robotPose.set(recoveryTarget.getPose());
      scheduler.run();
      SimHooks.stepTiming(0.25);
      scheduler.run();
      scheduler.run();

      assertEquals(1, markerCount.get());
      assertTrue(scheduler.isScheduled(trajectoryCommand));

      SimHooks.stepTiming(0.5);
      scheduler.run();
      scheduler.run();

      assertFalse(scheduler.isScheduled(trajectoryCommand));
      assertEquals(1, markerCount.get());
    } finally {
      scheduler.cancelAll();
      SimHooks.resumeTiming();
    }
  }

  @Test
  public void recoveryCanHoldTheFinalPosePastNominalDuration() {
    assert HAL.initialize(500, 0);
    CommandScheduler scheduler = SchedulerMaker.make();
    AtomicReference<Pose2d> robotPose = new AtomicReference<>(Pose2d.kZero);
    AtomicReference<SwerveSample> commandedSample = new AtomicReference<>();
    Consumer<SwerveSample> controller = commandedSample::set;

    AutoFactory factory =
        new AutoFactory(robotPose::get, robotPose::set, controller, false, new Subsystem() {})
            .withSwerveTrajectoryRecovery(
                new SwerveTrajectoryRecoveryConfig(0.25, Math.PI / 4, 0.05, Math.PI / 36));
    Trajectory<SwerveSample> trajectory =
        TrajectoryTestHelper.linearTrajectory(
            "final-recovery",
            Pose2d.kZero,
            new Pose2d(1.0, 0.0, Rotation2d.kZero),
            1.0,
            SwerveSample.class);
    AutoRoutine routine = factory.newRoutine("final-recovery");
    AutoTrajectory autoTrajectory = factory.trajectory(trajectory, routine, true);
    Command trajectoryCommand = autoTrajectory.cmd();

    SimHooks.pauseTiming();
    try {
      enableAutonomous();
      scheduler.schedule(routine.cmd());
      scheduler.schedule(trajectoryCommand);
      scheduler.run();

      SimHooks.stepTiming(1.1);
      scheduler.run();

      assertTrue(scheduler.isScheduled(trajectoryCommand));
      assertEquals(trajectory.getTotalTime(), commandedSample.get().t, 1e-9);
      assertEquals(0.0, commandedSample.get().getChassisSpeeds().vxMetersPerSecond, 1e-9);

      robotPose.set(commandedSample.get().getPose());
      scheduler.run();
      SimHooks.stepTiming(0.02);
      scheduler.run();

      assertFalse(scheduler.isScheduled(trajectoryCommand));
    } finally {
      scheduler.cancelAll();
      SimHooks.resumeTiming();
    }
  }

  @Test
  public void configRequiresResumeHysteresis() {
    assertThrows(
        IllegalArgumentException.class,
        () -> new SwerveTrajectoryRecoveryConfig(0.1, 0.5, 0.2, 0.1));
    assertThrows(
        IllegalArgumentException.class,
        () -> new SwerveTrajectoryRecoveryConfig(0.1, 0.5, 0.05, 0.6));
  }

  @Test
  public void headingErrorWrapsAtPi() {
    var config = new SwerveTrajectoryRecoveryConfig(1.0, 0.1, 0.5, 0.05);
    Pose2d current = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(179));
    Pose2d target = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(-179));

    assertFalse(config.shouldStartRecovery(current, target));
    assertTrue(config.shouldResume(current, target));
  }

  private static void enableAutonomous() {
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(true);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
    assertTrue(DriverStation.isAutonomousEnabled());
  }
}
