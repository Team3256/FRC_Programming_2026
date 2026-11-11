// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.sim;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;

/**
 * Constants for the maple-sim robot. Robot geometry comes from the CAD (Real Real Main.step,
 * converted by tools/ascope/step_to_ascope.py); the shot model is fit to our real tuning tables.
 * Values marked "from logs" should be refined once we have match logs to compare against.
 */
public final class SimConstants {
  /**
   * How often the drivetrain and field physics (maple-sim) step. Faster than the robot loop so
   * CTRE's 250 Hz PID behaves, but not so fast that the FUEL bogs it down. With all 408 FUEL out,
   * stepping every 2 ms took a third of a CPU core, and when the robot plowed into a pile the sim
   * fell behind real time and the robot lurched around as it caught up. 5 ms takes about a tenth.
   */
  public static final Time kArenaPeriod = Seconds.of(0.005);

  /** How often the mechanism sims step (see {@link MechanismSimLoop}). They're cheap, so fast. */
  public static final Time kMechanismPeriod = Seconds.of(0.002);

  /**
   * Where the robot starts when the sim boots: the outpost-side auto start (blue). Keep it clear of
   * the HUB, whose face is at x = 4.0 m; starting against it pins the robot so it can't drive.
   */
  public static final Pose2d kStartingPose = new Pose2d(3.57, 2.57, Rotation2d.k180deg);

  // ---------------- Drivetrain ----------------
  public static final Mass kRobotMassWithBumpers = Pounds.of(135);

  /** Bumper to bumper; the CAD bumpers sit 438 mm from center on every side. */
  public static final Distance kBumperLength = Millimeters.of(876);

  public static final DCMotor kDriveMotor = DCMotor.getKrakenX60(1);
  public static final DCMotor kSteerMotor = DCMotor.getKrakenX60(1);

  /** MK5n molded tread on carpet. From logs: compare commanded vs. measured acceleration. */
  public static final double kWheelCOF = 1.2;

  /**
   * Ramps (the BUMPs) as solid walls in maple-sim. Off, because our autos drive over them; {@link
   * BumpSim} makes them ramps the robot climbs over instead.
   */
  public static final boolean kRampColliders = false;

  /**
   * Spawn every FUEL on the field. maple-sim's default "efficiency mode" leaves out two thirds of
   * the neutral zone, which makes sweep autos look much worse than they are.
   */
  public static final boolean kFullFuelField = true;

  // ---------------- Intake ----------------
  /** Intake roller width, across the robot (CAD rollers are about 560 mm). */
  public static final Distance kIntakeWidth = Millimeters.of(560);

  /** How far the deployed intake reaches past the back bumper (CAD: 650 mm vs. 438 mm). */
  public static final Distance kIntakeExtension = Millimeters.of(210);

  /**
   * Intake pivot position (mechanism rotations) below which the intake is down far enough to pick
   * up FUEL. Ground is 0, stowed is 0.3354.
   */
  public static final double kIntakeDeployedMaxPosition = 0.08;

  /** Roller voltage above which the intake is treated as running. */
  public static final double kIntakeRunningVolts = 4.0;

  /** FUEL the hopper holds. From logs/CAD: count a full hopper. */
  public static final int kHopperCapacity = 40;

  /** FUEL preloaded at the start of auto (the game allows up to 8). */
  public static final int kPreloadedFuel = 8;

  // ---------------- Shooter ----------------
  /** Turret axis in the robot frame (CAD turret gear center; matches driveBaseToTurret). */
  public static final Translation2d kRobotToTurret = new Translation2d(0.146, -0.146);

  /**
   * Height FUEL leaves the shooter. In the CAD the flywheel axle is 0.595 m up and FUEL comes off
   * it under the hood, near the top of the shooter plates.
   */
  public static final Distance kShotExitHeight = Meters.of(0.68);

  /**
   * Exit speed per flywheel rotation per second, and the offset, in m/s. Fit to the real tuning
   * tables (ShooterConstants.hubLUT times the 1.04 velocity multiplier,
   * ShooterPivotConstants.hubLUT and the time-of-flight table in ShotCalculator): for each tuned
   * distance, solve for the launch that reaches the top of the HUB in the measured time of flight,
   * under maple-sim's gravity. Residuals are within 0.26 m/s.
   */
  public static final double kExitSpeedPerRps = 0.1848;

  public static final double kExitSpeedOffset = 0.812;

  /**
   * Launch angle above horizontal = kLaunchAngleAtHoodZeroDeg - kLaunchAnglePerHoodDeg * hood
   * angle. From the same fit; residuals within 2.4 degrees.
   */
  public static final double kLaunchAngleAtHoodZeroDeg = 75.06;

  public static final double kLaunchAnglePerHoodDeg = 0.774;

  /** Flywheel speed (rps) below which no shot leaves the robot. */
  public static final double kMinShotRps = 15.0;

  /** Feeder speed (rps) above which FUEL is being fed into the flywheels (feeding runs 80). */
  public static final double kFeedingRps = 40.0;

  /** FUEL per second the feeder pushes into the flywheels. From logs: hopper count over time. */
  public static final double kShotsPerSecond = 8.0;

  private SimConstants() {}
}
