// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.sim;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Arrays;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;

/**
 * The robot's side of the REBUILT field simulation: picks up FUEL when the intake is down and
 * running, keeps count of the hopper, and launches FUEL when the feeder pushes it into the
 * flywheels, in the direction the turret points and at the speed and angle the real shooter would
 * (see {@link SimConstants} for how the shot model was fit). maple-sim takes it from there: the
 * FUEL flies, bounces, scores in the HUB and keeps the match score.
 *
 * <p>Everything it reads is measured subsystem state, so it reacts to the real superstructure logic
 * the same way the robot would.
 */
public class RebuiltFieldSim {
  // maple-sim's RebuiltHub scoring box: 47 in square, from the bottom of the opening 10 in up
  private static final Translation3d kBlueHubOpening = new Translation3d(4.5974, 4.034536, 1.5748);
  private static final Translation3d kRedHubOpening = new Translation3d(11.938, 4.034536, 1.5748);
  private static final double kHubOpeningHalfWidth = Inches.of(47).in(Meters) / 2.0;
  private static final double kHubOpeningDepth = Inches.of(10).in(Meters);

  private final SwerveDriveSimulation drive;
  private final Supplier<Pose2d> estimatedPose;
  private final DoubleSupplier intakeVolts;
  private final DoubleSupplier intakePivotPosition;
  private final DoubleSupplier flywheelRps;
  private final DoubleSupplier hoodPosition;
  private final DoubleSupplier turretPosition;
  private final DoubleSupplier feederRps;
  private final IntakeSimulation intake;

  private double shotProgress = 0.0;
  private int shotsFired = 0;
  private int shotsScored = 0;
  // Filled in by maple-sim on the sim thread (holding the arena lock), logged from periodic()
  private Pose3d[] lastScoredShot = new Pose3d[0];
  private Pose3d[] lastMissedShot = new Pose3d[0];

  /**
   * Sets up the 2026 arena. Call before anything else touches {@link SimulatedArena}, in particular
   * before the drivetrain simulation is added to it.
   */
  public static void createArena() {
    var arena = new Arena2026Rebuilt(SimConstants.kRampColliders);
    arena.setEfficiencyMode(!SimConstants.kFullFuelField);
    SimulatedArena.overrideInstance(arena);
    arena.resetFieldForAuto();
  }

  /**
   * @param intakeVolts intake roller voltage
   * @param intakePivotPosition intake pivot position, mechanism rotations (0 = on the ground)
   * @param flywheelRps flywheel speed, rotations per second
   * @param hoodPosition hood position, mechanism rotations
   * @param turretPosition turret position, mechanism rotations (0 = shooting forward, positive
   *     turns clockwise, see Turret#pointToPose)
   * @param feederRps feeder speed, rotations per second
   */
  public RebuiltFieldSim(
      SwerveDriveSimulation drive,
      Supplier<Pose2d> estimatedPose,
      DoubleSupplier intakeVolts,
      DoubleSupplier intakePivotPosition,
      DoubleSupplier flywheelRps,
      DoubleSupplier hoodPosition,
      DoubleSupplier turretPosition,
      DoubleSupplier feederRps) {
    this.drive = drive;
    this.estimatedPose = estimatedPose;
    this.intakeVolts = intakeVolts;
    this.intakePivotPosition = intakePivotPosition;
    this.flywheelRps = flywheelRps;
    this.hoodPosition = hoodPosition;
    this.turretPosition = turretPosition;
    this.feederRps = feederRps;
    synchronized (SimulatedArena.getInstance()) {
      intake =
          IntakeSimulation.OverTheBumperIntake(
              "Fuel",
              drive,
              SimConstants.kIntakeWidth,
              SimConstants.kIntakeExtension,
              IntakeSimulation.IntakeSide.BACK,
              SimConstants.kHopperCapacity);
      intake.setGamePiecesCount(SimConstants.kPreloadedFuel);
      SimulatedArena.getInstance().addCustomSimulation(new BumpSim(drive));
    }
  }

  /** Puts the FUEL back on the field and preloads the hopper, as at the start of a match. */
  public void resetForAuto() {
    synchronized (SimulatedArena.getInstance()) {
      SimulatedArena.getInstance().resetFieldForAuto();
      intake.setGamePiecesCount(SimConstants.kPreloadedFuel);
      shotProgress = 0.0;
      shotsFired = 0;
      shotsScored = 0;
    }
  }

  /** Call once per robot loop, after the subsystems have read their inputs. */
  public void periodic() {
    var arena = SimulatedArena.getInstance();
    synchronized (arena) {
      boolean intaking =
          DriverStation.isEnabled()
              && intakeVolts.getAsDouble() > SimConstants.kIntakeRunningVolts
              && intakePivotPosition.getAsDouble() < SimConstants.kIntakeDeployedMaxPosition;
      if (intaking) {
        intake.startIntake();
      } else {
        intake.stopIntake();
      }

      boolean feeding =
          DriverStation.isEnabled()
              && feederRps.getAsDouble() > SimConstants.kFeedingRps
              && flywheelRps.getAsDouble() > SimConstants.kMinShotRps;
      if (feeding) {
        shotProgress += SimConstants.kShotsPerSecond * 0.02;
        while (shotProgress >= 1.0 && intake.obtainGamePieceFromIntake()) {
          launch();
          shotProgress -= 1.0;
        }
        // Don't bank shots while the hopper is empty
        shotProgress = Math.min(shotProgress, 1.0);
      } else {
        shotProgress = 0.0;
      }

      Pose2d truePose = drive.getSimulatedDriveTrainPose();
      Logger.recordOutput("FieldSimulation/RobotPose", truePose);
      Logger.recordOutput(
          "FieldSimulation/RobotPose3d",
          BumpSim.robotPose3d(truePose, drive.config.moduleTranslations));
      Logger.recordOutput(
          "FieldSimulation/ModuleStates",
          Arrays.stream(drive.getModules())
              .map(module -> module.getCurrentState())
              .toArray(SwerveModuleState[]::new));
      Logger.recordOutput(
          "FieldSimulation/OdometryError",
          truePose.getTranslation().getDistance(estimatedPose.get().getTranslation()));
      Logger.recordOutput("FieldSimulation/Fuel", arena.getGamePiecesArrayByType("Fuel"));
      Logger.recordOutput("FieldSimulation/HopperFuel", intake.getGamePiecesAmount());
      Logger.recordOutput("FieldSimulation/Intaking", intaking);
      Logger.recordOutput("FieldSimulation/ShotsFired", shotsFired);
      Logger.recordOutput("FieldSimulation/ShotsScored", shotsScored);
      Logger.recordOutput("FieldSimulation/ScoredShot", lastScoredShot);
      Logger.recordOutput("FieldSimulation/MissedShot", lastMissedShot);
      Logger.recordOutput("FieldSimulation/BlueScore", arena.getScore(true));
      Logger.recordOutput("FieldSimulation/RedScore", arena.getScore(false));
    }
  }

  /** Exit speed (m/s) for a flywheel speed, from the fit in {@link SimConstants}. */
  public static double exitSpeed(double flywheelRps) {
    return SimConstants.kExitSpeedPerRps * flywheelRps + SimConstants.kExitSpeedOffset;
  }

  /** Launch angle above horizontal (degrees) for a hood position in mechanism rotations. */
  public static double launchAngleDegrees(double hoodPosition) {
    return SimConstants.kLaunchAngleAtHoodZeroDeg
        - SimConstants.kLaunchAnglePerHoodDeg * hoodPosition * 360.0;
  }

  private void launch() {
    Pose2d pose = drive.getSimulatedDriveTrainPose();
    shotsFired++;
    // Same convention as Turret#pointToPose: position = -(shot direction - robot heading)
    Rotation2d shotDirection =
        pose.getRotation().minus(Rotation2d.fromRotations(turretPosition.getAsDouble()));
    // maple-sim rotates the shooter offset by the shot direction, which only works for a shooter
    // fixed to the chassis. Launch from the turret's field position instead, with the turret's own
    // field velocity (the chassis velocity plus the turret swinging around the robot center).
    Translation2d turretOffset = SimConstants.kRobotToTurret.rotateBy(pose.getRotation());
    ChassisSpeeds chassis = drive.getDriveTrainSimulatedChassisSpeedsFieldRelative();
    var turretSpeeds =
        new ChassisSpeeds(
            chassis.vxMetersPerSecond - chassis.omegaRadiansPerSecond * turretOffset.getY(),
            chassis.vyMetersPerSecond + chassis.omegaRadiansPerSecond * turretOffset.getX(),
            chassis.omegaRadiansPerSecond);
    var fuel =
        new RebuiltFuelOnFly(
            pose.getTranslation().plus(turretOffset),
            Translation2d.kZero,
            turretSpeeds,
            shotDirection,
            SimConstants.kShotExitHeight,
            MetersPerSecond.of(exitSpeed(flywheelRps.getAsDouble())),
            Degrees.of(launchAngleDegrees(hoodPosition.getAsDouble())));
    fuel.withHitTargetCallBack(() -> shotsScored++)
        .withProjectileTrajectoryDisplayCallBack(this::showShot, this::showShot);
    SimulatedArena.getInstance().addGamePieceProjectile(fuel);
  }

  /**
   * Shows a shot's path as scored or missed. maple-sim sorts paths by a target point we don't set
   * (the HUB scores FUEL with its own box check), so it would call every shot a miss; sort them by
   * whether the path passes through that box instead. An empty path means the shot landed.
   */
  private void showShot(List<Pose3d> poses) {
    Pose3d[] path = poses.toArray(Pose3d[]::new);
    if (path.length == 0) {
      lastScoredShot = path;
      lastMissedShot = path;
    } else if (Arrays.stream(path).anyMatch(RebuiltFieldSim::inHubOpening)) {
      lastScoredShot = path;
    } else {
      lastMissedShot = path;
    }
  }

  /** Whether a point is inside either HUB's scoring box (maple-sim's RebuiltHub). */
  private static boolean inHubOpening(Pose3d point) {
    double fromCenterX =
        Math.min(
            Math.abs(point.getX() - kBlueHubOpening.getX()),
            Math.abs(point.getX() - kRedHubOpening.getX()));
    return fromCenterX < kHubOpeningHalfWidth
        && Math.abs(point.getY() - kBlueHubOpening.getY()) < kHubOpeningHalfWidth
        && point.getZ() >= kBlueHubOpening.getZ()
        && point.getZ() <= kBlueHubOpening.getZ() + kHubOpeningDepth;
  }
}
