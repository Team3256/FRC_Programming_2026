// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.sim;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import frc.robot.subsystems.intakepivot.IntakePivotConstants;

public final class SimMechs {

  public final Mechanism2d mech =
      new Mechanism2d(Constants.SimulationConstants.kDrivebaseWidth.in(Meters), 1.0);

  private final MechanismRoot2d groundIntakeRoot =
      mech.getRoot(
          "Ground Intake",
          Constants.SimulationConstants.kDrivebaseWidth.in(Meters) / 2 - 0.15,
          Inches.of(2).in(Meters));

  private final MechanismLigament2d intakePivotViz =
      groundIntakeRoot.append(
          new MechanismLigament2d(
              "Intake Pivot",
              IntakePivotConstants.PivotSim.intakePivotLength.in(Meters) / 4,
              0.0,
              7,
              new Color8Bit(Color.kBlue)));

  private final MechanismLigament2d groundIntakeRollerViz =
      intakePivotViz.append(
          new MechanismLigament2d(
              "Intake Roller", Inches.of(2).in(Meters), 0.0, 3, new Color8Bit(Color.kRed)));

  private final MechanismRoot2d shooterRoot =
      mech.getRoot(
          "Shooter",
          Constants.SimulationConstants.kDrivebaseWidth.in(Meters) / 2 - 0.05,
          Inches.of(14).in(Meters));

  private final MechanismLigament2d shooterPivotViz =
      shooterRoot.append(
          new MechanismLigament2d(
              "Shooter Pivot", Inches.of(10).in(Meters), 0.0, 7, new Color8Bit(Color.kGreen)));

  private final MechanismLigament2d shooterWheelViz =
      shooterPivotViz.append(
          new MechanismLigament2d(
              "Shooter Wheel", Inches.of(4).in(Meters), 0.0, 4, new Color8Bit(Color.kOrange)));

  public final Mechanism2d turretMech =
      new Mechanism2d(Constants.SimulationConstants.kDrivebaseWidth.in(Meters), 1.0);
  private static SimMechs instance = null;

  private final MechanismRoot2d turretRoot =
      mech.getRoot(
          "Turret",
          Constants.SimulationConstants.kDrivebaseWidth.in(Meters) / 2 - 0.15,
          Inches.of(7).in(Meters));

  private final MechanismLigament2d turretViz =
      turretRoot.append(
          new MechanismLigament2d(
              "Turret", Inches.of(3).in(Meters), 0.0, 5, new Color8Bit(Color.kPink)));

  private SimMechs() {}

  public static SimMechs getInstance() {
    if (instance == null) {
      instance = new SimMechs();
    }
    return instance;
  }

  public void updatePivot(Angle angle) {
    intakePivotViz.setAngle(angle.in(Degrees));
  }

  public void updateRollers(Angle x) {
    groundIntakeRollerViz.setAngle(groundIntakeRollerViz.getAngle() + x.in(Degrees));
  }

  public void updateTurret(Angle x) {
    turretViz.setAngle(turretViz.getAngle() + x.in(Degrees));
  }

  public void updateShooterPivot(Angle angle) {
    shooterPivotViz.setAngle(angle.in(Degrees)); // flips direction
  }

  public void updateShooterWheel(Angle delta) {
    shooterWheelViz.setAngle(shooterWheelViz.getAngle() + delta.in(Degrees));
  }

  public void publishToNT() {
    SmartDashboard.putData("RobotSim", mech);
  }

  public void updateClimb(Angle of) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'updateClimb'");
  }
}
