// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.sim;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public final class SimMechs {

  public static final Mechanism2d mech = new Mechanism2d(5, 5);

  private static final MechanismRoot2d ampevator = mech.getRoot("ampevator", 1, 0);

  private static final MechanismLigament2d m_ampevator =
      ampevator.append(new MechanismLigament2d("ampevator", 2, 90));
  private static final MechanismLigament2d m_ampevatorRollers =
      m_ampevator.append(
          new MechanismLigament2d("rollers", .35, 0, 5.0, new Color8Bit(Color.kPurple)));

  private static final MechanismRoot2d pivotShooterRoot = mech.getRoot("Pivot Shooter", 3.5, 0.2);
  private static final MechanismLigament2d pivotShooterViz =
      pivotShooterRoot.append(
          new MechanismLigament2d("Pivot Shooter", 1, 0.0, 5.0, new Color8Bit(Color.kGreen)));
  private static final MechanismLigament2d rightShooterViz =
      pivotShooterViz.append(
          new MechanismLigament2d(
              "Right Shooter Flywheel", 0.35, 90, 2.5, new Color8Bit(Color.kRed)));
  private static final MechanismLigament2d leftShooterViz =
      pivotShooterViz.append(
          new MechanismLigament2d(
              "Left Shooter Flywheel", .25, 0.0, 2.5, new Color8Bit(Color.kYellow)));

  public static void updateAmpevator(double position) {
    m_ampevator.setLength(position);
  }

  public static void addToShooterFlywheelAngle(double leftAngle, double rightAngle) {
    leftShooterViz.setAngle(leftShooterViz.getAngle() + leftAngle);
    rightShooterViz.setAngle(rightShooterViz.getAngle() + rightAngle);
  }

  public static void updatePivotShooter(Rotation2d angle) {
    pivotShooterViz.setAngle(angle.getDegrees());
  }

  public static void init() {
    SmartDashboard.putData("Ampevator", mech);
  }
}
