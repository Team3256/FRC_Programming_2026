// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.sim;

import static edu.wpi.first.units.Units.Kilograms;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.LinesHorizontal;
import frc.robot.FieldConstants.LinesVertical;
import org.dyn4j.geometry.Vector2;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

/**
 * The BUMPs as ramps the robot drives over. maple-sim is 2D, so on its own a BUMP is either a solid
 * wall or not there at all. This adds what crossing one does to the robot: each wheel on a ramp is
 * pulled downhill by its share of the robot's weight, which slows the robot on the way up, speeds
 * it up on the way down and twists it when it hits the ramp at an angle. {@link #robotPose3d} lifts
 * and tilts the robot over the bump for AdvantageScope.
 *
 * <p>The profile matches AdvantageScope's 2026 field model: a ridge 6.513 in tall along the bump's
 * center line with 15 degree ramps either side, so about 49 in deep at the carpet.
 */
public class BumpSim implements SimulatedArena.Simulatable {
  private static final double kHeight = FieldConstants.LeftBump.height;
  private static final double kSlope = Math.toRadians(15.0);
  private static final double kHalfDepth = kHeight / Math.tan(kSlope);

  /**
   * Height of the flat belly pan above the carpet (CAD). Our wheelbase is wider than the ridge, so
   * at the crest the robot rides on its belly rather than its wheels.
   */
  private static final double kBellyClearance = 0.041;

  /** Half the belly pan's length and width (CAD); the bumpers sit higher. */
  private static final double kBellyHalfSize = 0.40;

  /** Each BUMP as {center x, min y, max y}; robots cross them along x. */
  private static final double[][] kBumps = {
    {LinesVertical.hubCenter, LinesHorizontal.rightBumpEnd, LinesHorizontal.rightBumpStart},
    {LinesVertical.hubCenter, LinesHorizontal.leftBumpEnd, LinesHorizontal.leftBumpStart},
    {LinesVertical.oppHubCenter, LinesHorizontal.rightBumpEnd, LinesHorizontal.rightBumpStart},
    {LinesVertical.oppHubCenter, LinesHorizontal.leftBumpEnd, LinesHorizontal.leftBumpStart}
  };

  private final AbstractDriveTrainSimulation drive;
  private final Translation2d[] moduleTranslations;
  private final double downhillForcePerWheel;

  public BumpSim(AbstractDriveTrainSimulation drive) {
    this.drive = drive;
    this.moduleTranslations = drive.config.moduleTranslations;
    // Gravity's pull along the ramp, horizontal part (the normal force takes the rest)
    double weightPerWheel = drive.config.robotMass.in(Kilograms) * 9.81 / moduleTranslations.length;
    this.downhillForcePerWheel = weightPerWheel * Math.sin(kSlope) * Math.cos(kSlope);
  }

  /** Runs on the arena's thread after each physics step; the force acts on the next one. */
  @Override
  public void simulationSubTick(int subTickNum) {
    Pose2d pose = drive.getSimulatedDriveTrainPose();
    for (Translation2d module : moduleTranslations) {
      Translation2d wheel = toField(pose, module);
      double downhill = downhillDirectionX(wheel);
      if (downhill != 0.0) {
        drive.applyForce(
            new Vector2(downhill * downhillForcePerWheel, 0.0),
            new Vector2(wheel.getX(), wheel.getY()));
      }
    }
  }

  /** Height of the field surface at a point: the ramp height on a BUMP, 0 elsewhere. */
  public static double heightAt(Translation2d point) {
    for (double[] bump : kBumps) {
      double fromCenter = Math.abs(point.getX() - bump[0]);
      if (fromCenter < kHalfDepth && point.getY() > bump[1] && point.getY() < bump[2]) {
        return kHeight * (1.0 - fromCenter / kHalfDepth);
      }
    }
    return 0.0;
  }

  /** Which way along x is downhill at a point on a ramp (+1 or -1), or 0 off the BUMPs. */
  private static double downhillDirectionX(Translation2d point) {
    for (double[] bump : kBumps) {
      double fromCenter = point.getX() - bump[0];
      if (Math.abs(fromCenter) < kHalfDepth && point.getY() > bump[1] && point.getY() < bump[2]) {
        return Math.signum(fromCenter);
      }
    }
    return 0.0;
  }

  /**
   * The robot's pose in 3D, raised and tilted to sit on the field surface: on a least-squares plane
   * through the four wheel heights (the modules are symmetric about the robot center, so the fit
   * separates into a pitch and a roll term), lifted further wherever the belly would sink into the
   * crest.
   */
  public static Pose3d robotPose3d(Pose2d pose, Translation2d[] moduleTranslations) {
    double sumHeight = 0.0, sumHx = 0.0, sumXx = 0.0, sumHy = 0.0, sumYy = 0.0;
    for (Translation2d module : moduleTranslations) {
      double h = heightAt(toField(pose, module));
      sumHeight += h;
      sumHx += h * module.getX();
      sumXx += module.getX() * module.getX();
      sumHy += h * module.getY();
      sumYy += module.getY() * module.getY();
    }
    double z = sumHeight / moduleTranslations.length;
    double slopeX = sumHx / sumXx;
    double slopeY = sumHy / sumYy;

    // Check a grid over the belly pan for ground poking above it
    double lift = 0.0;
    for (double x = -kBellyHalfSize; x <= kBellyHalfSize + 1e-9; x += kBellyHalfSize / 4) {
      for (double y = -kBellyHalfSize; y <= kBellyHalfSize + 1e-9; y += kBellyHalfSize) {
        double belly = z + slopeX * x + slopeY * y + kBellyClearance;
        lift = Math.max(lift, heightAt(toField(pose, new Translation2d(x, y))) - belly);
      }
    }

    // Front higher than back is nose up, which is negative pitch; left higher is positive roll
    return new Pose3d(
        pose.getX(),
        pose.getY(),
        z + lift,
        new Rotation3d(Math.atan(slopeY), -Math.atan(slopeX), pose.getRotation().getRadians()));
  }

  private static Translation2d toField(Pose2d pose, Translation2d robotRelative) {
    return pose.getTranslation().plus(robotRelative.rotateBy(pose.getRotation()));
  }
}
