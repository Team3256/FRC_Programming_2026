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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
      // The trajectory's velocity feedforward is kept while recovering so the robot rejoins the
      // path at the speed the trajectory expects, rather than decelerating to a stop.
      assertEquals(1.0, recoveryTarget.getChassisSpeeds().vxMetersPerSecond, 1e-6);
      assertEquals(0.0, recoveryTarget.getChassisSpeeds().vyMetersPerSecond, 1e-6);
      assertEquals(0.0, recoveryTarget.getChassisSpeeds().omegaRadiansPerSecond, 1e-6);

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
  public void recoveryRejoinsAheadOfTheHoldPointAtSpeed() {
    assert HAL.initialize(500, 0);
    CommandScheduler scheduler = SchedulerMaker.make();
    AtomicReference<Pose2d> robotPose = new AtomicReference<>(Pose2d.kZero);
    AtomicReference<SwerveSample> commandedSample = new AtomicReference<>();
    Consumer<SwerveSample> controller = commandedSample::set;

    AutoFactory factory =
        new AutoFactory(robotPose::get, robotPose::set, controller, false, new Subsystem() {})
            .withSwerveTrajectoryRecovery(
                new SwerveTrajectoryRecoveryConfig(0.25, Math.PI / 4, 0.05, Math.PI / 36));
    // 1 m over 1 s, so the trajectory expects 1 m/s everywhere and pose x equals time.
    Trajectory<SwerveSample> trajectory =
        TrajectoryTestHelper.linearTrajectory(
            "rejoin",
            Pose2d.kZero,
            new Pose2d(1.0, 0.0, Rotation2d.kZero),
            1.0,
            SwerveSample.class);
    AutoRoutine routine = factory.newRoutine("rejoin");
    AutoTrajectory autoTrajectory = factory.trajectory(trajectory, routine, true);
    Command trajectoryCommand = autoTrajectory.cmd();

    SimHooks.pauseTiming();
    try {
      enableAutonomous();
      scheduler.schedule(routine.cmd());
      scheduler.schedule(trajectoryCommand);
      scheduler.run();

      // Robot never left the start, so at t=0.4 it is 0.4 m behind and recovery starts.
      SimHooks.stepTiming(0.4);
      scheduler.run();
      assertEquals(0.4, commandedSample.get().t, 1e-6);
      assertEquals(1.0, commandedSample.get().getChassisSpeeds().vxMetersPerSecond, 1e-6);

      // Robot is laterally off the path, ahead of the hold point: the target slides forward to
      // the closest on-path point, still carrying the trajectory's velocity, but does not resume.
      robotPose.set(new Pose2d(0.5, 0.2, Rotation2d.kZero));
      SimHooks.stepTiming(0.1);
      scheduler.run();
      assertEquals(0.5, commandedSample.get().t, 1e-6);
      assertEquals(1.0, commandedSample.get().getChassisSpeeds().vxMetersPerSecond, 1e-6);
      assertEquals(0.5, commandedSample.get().x, 1e-6);
      assertEquals(0.0, commandedSample.get().y, 1e-6);

      // The rejoin point only moves forward, even if the robot falls back behind it.
      robotPose.set(new Pose2d(0.3, 0.2, Rotation2d.kZero));
      SimHooks.stepTiming(0.1);
      scheduler.run();
      assertEquals(0.5, commandedSample.get().t, 1e-6);

      // Robot drives onto the path further ahead: resume from there, not from the hold time.
      robotPose.set(new Pose2d(0.7, 0.02, Rotation2d.kZero));
      SimHooks.stepTiming(0.1);
      scheduler.run();
      assertEquals(0.7, commandedSample.get().t, 1e-6);
      assertEquals(1.0, commandedSample.get().getChassisSpeeds().vxMetersPerSecond, 1e-6);

      // Trajectory time now advances continuously from the rejoin point.
      robotPose.set(new Pose2d(0.8, 0.0, Rotation2d.kZero));
      SimHooks.stepTiming(0.1);
      scheduler.run();
      assertEquals(0.8, commandedSample.get().t, 1e-6);
      assertTrue(scheduler.isScheduled(trajectoryCommand));

      SimHooks.stepTiming(0.3);
      scheduler.run();
      scheduler.run();
      assertFalse(scheduler.isScheduled(trajectoryCommand));
    } finally {
      scheduler.cancelAll();
      SimHooks.resumeTiming();
    }
  }

  @Test
  public void recoveryRejoinSearchIsBoundedByTheWindow() {
    assert HAL.initialize(500, 0);
    CommandScheduler scheduler = SchedulerMaker.make();
    AtomicReference<Pose2d> robotPose = new AtomicReference<>(Pose2d.kZero);
    AtomicReference<SwerveSample> commandedSample = new AtomicReference<>();
    Consumer<SwerveSample> controller = commandedSample::set;

    AutoFactory factory =
        new AutoFactory(robotPose::get, robotPose::set, controller, false, new Subsystem() {})
            .withSwerveTrajectoryRecovery(
                new SwerveTrajectoryRecoveryConfig(0.25, Math.PI / 4, 0.05, Math.PI / 36, 0.2));
    Trajectory<SwerveSample> trajectory =
        TrajectoryTestHelper.linearTrajectory(
            "window",
            Pose2d.kZero,
            new Pose2d(2.0, 0.0, Rotation2d.kZero),
            2.0,
            SwerveSample.class);
    AutoRoutine routine = factory.newRoutine("window");
    AutoTrajectory autoTrajectory = factory.trajectory(trajectory, routine, true);
    Command trajectoryCommand = autoTrajectory.cmd();

    SimHooks.pauseTiming();
    try {
      enableAutonomous();
      scheduler.schedule(routine.cmd());
      scheduler.schedule(trajectoryCommand);
      scheduler.run();

      SimHooks.stepTiming(0.4);
      scheduler.run();
      assertEquals(0.4, commandedSample.get().t, 1e-6);

      // Robot is far ahead; the rejoin point may only advance by the window per cycle.
      robotPose.set(new Pose2d(1.5, 0.3, Rotation2d.kZero));
      SimHooks.stepTiming(0.02);
      scheduler.run();
      assertEquals(0.6, commandedSample.get().t, 1e-6);
      SimHooks.stepTiming(0.02);
      scheduler.run();
      assertEquals(0.8, commandedSample.get().t, 1e-6);
    } finally {
      scheduler.cancelAll();
      SimHooks.resumeTiming();
    }
  }

  @Test
  public void configRejectsNegativeRejoinWindow() {
    assertThrows(
        IllegalArgumentException.class,
        () -> new SwerveTrajectoryRecoveryConfig(0.25, 0.5, 0.05, 0.1, -1.0));
    assertEquals(
        SwerveTrajectoryRecoveryConfig.kDefaultRejoinSearchWindowSeconds,
        new SwerveTrajectoryRecoveryConfig(0.25, 0.5, 0.05, 0.1).rejoinSearchWindowSeconds(),
        1e-9);
  }

  @Test
  public void recoveryStartIsDebounced() {
    assert HAL.initialize(500, 0);
    CommandScheduler scheduler = SchedulerMaker.make();
    AtomicReference<Pose2d> robotPose = new AtomicReference<>(Pose2d.kZero);
    AtomicReference<SwerveSample> commandedSample = new AtomicReference<>();
    Consumer<SwerveSample> controller = commandedSample::set;

    AutoFactory factory =
        new AutoFactory(robotPose::get, robotPose::set, controller, false, new Subsystem() {})
            .withSwerveTrajectoryRecovery(
                new SwerveTrajectoryRecoveryConfig(0.25, Math.PI / 4, 0.05, Math.PI / 36)
                    .withStartDebounce(0.05));
    Trajectory<SwerveSample> trajectory =
        TrajectoryTestHelper.linearTrajectory(
            "debounce",
            Pose2d.kZero,
            new Pose2d(1.0, 0.0, Rotation2d.kZero),
            1.0,
            SwerveSample.class);
    AutoRoutine routine = factory.newRoutine("debounce");
    AutoTrajectory autoTrajectory = factory.trajectory(trajectory, routine, true);
    Command trajectoryCommand = autoTrajectory.cmd();

    SimHooks.pauseTiming();
    try {
      enableAutonomous();
      scheduler.schedule(routine.cmd());
      scheduler.schedule(trajectoryCommand);
      scheduler.run();

      // A single cycle with a large error (e.g. a bad pose estimate) must not pause progress.
      robotPose.set(new Pose2d(0.4, 0.0, Rotation2d.kZero));
      SimHooks.stepTiming(0.4);
      robotPose.set(new Pose2d(0.0, 0.0, Rotation2d.kZero));
      scheduler.run();
      robotPose.set(new Pose2d(0.42, 0.0, Rotation2d.kZero));
      SimHooks.stepTiming(0.02);
      scheduler.run();
      assertEquals(0.42, commandedSample.get().t, 1e-6);

      // Error that persists for the debounce period does start recovery.
      robotPose.set(new Pose2d(0.0, 0.0, Rotation2d.kZero));
      SimHooks.stepTiming(0.02);
      scheduler.run();
      assertEquals(0.44, commandedSample.get().t, 1e-6);
      SimHooks.stepTiming(0.02);
      scheduler.run();
      assertEquals(0.46, commandedSample.get().t, 1e-6);
      SimHooks.stepTiming(0.02);
      scheduler.run();
      assertEquals(0.48, commandedSample.get().t, 1e-6);
      // Debounce elapsed: recovery starts and holds at this cycle's time.
      SimHooks.stepTiming(0.02);
      scheduler.run();
      assertEquals(0.5, commandedSample.get().t, 1e-6);
      SimHooks.stepTiming(0.1);
      scheduler.run();
      assertEquals(0.5, commandedSample.get().t, 1e-6);
    } finally {
      scheduler.cancelAll();
      SimHooks.resumeTiming();
    }
  }

  @Test
  public void resumeRequiresMatchingVelocityWhenSpeedsAreSupplied() {
    assert HAL.initialize(500, 0);
    CommandScheduler scheduler = SchedulerMaker.make();
    AtomicReference<Pose2d> robotPose = new AtomicReference<>(Pose2d.kZero);
    AtomicReference<ChassisSpeeds> robotSpeeds = new AtomicReference<>(new ChassisSpeeds());
    AtomicReference<SwerveSample> commandedSample = new AtomicReference<>();
    Consumer<SwerveSample> controller = commandedSample::set;

    AutoFactory factory =
        new AutoFactory(robotPose::get, robotPose::set, controller, false, new Subsystem() {})
            .withSwerveTrajectoryRecovery(
                new SwerveTrajectoryRecoveryConfig(0.25, Math.PI / 4, 0.05, Math.PI / 36)
                    .withResumeVelocityTolerance(0.3),
                robotSpeeds::get);
    Trajectory<SwerveSample> trajectory =
        TrajectoryTestHelper.linearTrajectory(
            "velocity",
            Pose2d.kZero,
            new Pose2d(1.0, 0.0, Rotation2d.kZero),
            1.0,
            SwerveSample.class);
    AutoRoutine routine = factory.newRoutine("velocity");
    AutoTrajectory autoTrajectory = factory.trajectory(trajectory, routine, true);
    Command trajectoryCommand = autoTrajectory.cmd();

    SimHooks.pauseTiming();
    try {
      enableAutonomous();
      scheduler.schedule(routine.cmd());
      scheduler.schedule(trajectoryCommand);
      scheduler.run();

      SimHooks.stepTiming(0.4);
      scheduler.run();
      assertEquals(0.4, commandedSample.get().t, 1e-6);

      // On the path but stationary: pose is within tolerance, velocity is not, so hold.
      robotPose.set(new Pose2d(0.5, 0.0, Rotation2d.kZero));
      robotSpeeds.set(new ChassisSpeeds());
      SimHooks.stepTiming(0.1);
      scheduler.run();
      assertEquals(0.5, commandedSample.get().t, 1e-6);
      SimHooks.stepTiming(0.1);
      scheduler.run();
      assertEquals(0.5, commandedSample.get().t, 1e-6);

      // Moving at the trajectory's speed: resume, and time advances from the rejoin point.
      robotSpeeds.set(new ChassisSpeeds(0.9, 0.0, 0.0));
      SimHooks.stepTiming(0.02);
      scheduler.run();
      assertEquals(0.5, commandedSample.get().t, 1e-6);
      robotPose.set(new Pose2d(0.6, 0.0, Rotation2d.kZero));
      SimHooks.stepTiming(0.1);
      scheduler.run();
      assertEquals(0.6, commandedSample.get().t, 1e-6);
    } finally {
      scheduler.cancelAll();
      SimHooks.resumeTiming();
    }
  }

  @Test
  public void configRejectsInvalidDebounceAndVelocityTolerance() {
    var base = new SwerveTrajectoryRecoveryConfig(0.25, 0.5, 0.05, 0.1);
    assertThrows(IllegalArgumentException.class, () -> base.withStartDebounce(-0.1));
    assertThrows(IllegalArgumentException.class, () -> base.withResumeVelocityTolerance(-1.0));
    assertEquals(0.0, base.startDebounceSeconds(), 1e-9);
    assertTrue(Double.isInfinite(base.resumeVelocityToleranceMetersPerSecond()));
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
