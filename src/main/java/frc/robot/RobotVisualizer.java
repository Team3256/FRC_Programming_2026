// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Logs the poses of the robot's moving parts for the AdvantageScope robot model
 * (ascope_assets/Robot_3256). It uses measured positions, so it works the same on real robot logs
 * as in simulation.
 *
 * <p>In AdvantageScope, drag "RobotVisualizer/Components" onto the robot in a 3D field tab.
 */
public class RobotVisualizer {
  /** Turret axis in the robot frame (CAD turret gear center); model_0 is centered on it. */
  private static final Translation3d kRobotToTurret = new Translation3d(0.14605, -0.14605, 0.0);

  /**
   * Which way the shooter faces in the CAD, robot frame. The turret model is turned by the
   * difference between this and where the turret position says the shooter faces.
   */
  private static final double kCadShooterFacingRad = Math.toRadians(-16.04);

  /**
   * Hood pivot (the flywheel axle) relative to the turret axis, with the turret as in the CAD;
   * model_1 is centered on it.
   */
  private static final Translation3d kTurretToHoodPivot =
      new Translation3d(0.0771, -0.0222, 0.5953);

  /** The flywheel axle direction with the turret as in the CAD; the hood swings about it. */
  private static final double[] kHoodAxis = {0.276, 0.961, 0.0};

  /**
   * Hood rotation per unit of hood position (mechanism rotations). Positive tips the hood over the
   * flywheel, which flattens the shot. Flip the sign if the hood swings the wrong way.
   */
  private static final double kHoodRadiansPerRotation = 2 * Math.PI;

  /** Intake pivot axle in the robot frame (CAD 0500_Pivot_Axle); model_2 is centered on it. */
  private static final Translation3d kRobotToIntakePivot = new Translation3d(-0.2745, 0.0, 0.216);

  /**
   * Intake rotation per unit of pivot position, about the robot's +Y axis. The CAD has the intake
   * on the ground (position 0); positive lifts it up and over into the robot (stow is 0.3354).
   */
  private static final double kIntakeRadiansPerRotation = 2 * Math.PI;

  private final DoubleSupplier turretPosition;
  private final DoubleSupplier hoodPosition;
  private final DoubleSupplier intakePivotPosition;

  /**
   * @param turretPosition measured turret position, in mechanism rotations
   * @param hoodPosition measured hood position, in mechanism rotations
   * @param intakePivotPosition measured intake pivot position, in mechanism rotations
   */
  public RobotVisualizer(
      DoubleSupplier turretPosition,
      DoubleSupplier hoodPosition,
      DoubleSupplier intakePivotPosition) {
    this.turretPosition = turretPosition;
    this.hoodPosition = hoodPosition;
    this.intakePivotPosition = intakePivotPosition;
  }

  public void periodic() {
    // Same convention as Turret#pointToPose: the shooter faces -position (rotations) off the
    // robot's heading
    double shooterFacing = -turretPosition.getAsDouble() * 2 * Math.PI;
    var turret =
        new Pose3d(kRobotToTurret, new Rotation3d(0.0, 0.0, shooterFacing - kCadShooterFacingRad));

    var hood =
        turret.transformBy(
            new Transform3d(
                kTurretToHoodPivot,
                new Rotation3d(
                    VecBuilder.fill(kHoodAxis[0], kHoodAxis[1], kHoodAxis[2]),
                    hoodPosition.getAsDouble() * kHoodRadiansPerRotation)));

    var intake =
        new Pose3d(
            kRobotToIntakePivot,
            new Rotation3d(
                0.0, intakePivotPosition.getAsDouble() * kIntakeRadiansPerRotation, 0.0));

    // Order matches the model_N files: 0 = turret, 1 = hood, 2 = intake
    Logger.recordOutput("RobotVisualizer/Components", turret, hood, intake);
  }
}
