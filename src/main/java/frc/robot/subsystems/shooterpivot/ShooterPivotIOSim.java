// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooterpivot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.littletonrobotics.junction.LoggedRobot;

public class ShooterPivotIOSim extends ShooterPivotIOTalonFX {

  private final SingleJointedArmSim pivotSimModel =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(1),
          ShooterPivotConstants.PivotSim.pivotSimGearing,
          ShooterPivotConstants.PivotSim.jkGMetersSquared,
          ShooterPivotConstants.PivotSim.shooterPivotLength.in(Meters),
          ShooterPivotConstants.PivotSim.minAngle.getRadians(),
          ShooterPivotConstants.PivotSim.maxAngle.getRadians(),
          true,
          ShooterPivotConstants.PivotSim.startingAngle.getRadians());

  private TalonFXSimState pivotSimState;

  public ShooterPivotIOSim() {
    super();
    pivotSimState = super.getMotor().getSimState();
    pivotSimState.Orientation = ChassisReference.Clockwise_Positive;
  }

  @Override
  public void updateInputs(ShooterPivotIOInputs inputs) {

    pivotSimState = super.getMotor().getSimState();
    pivotSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    pivotSimModel.setInputVoltage(pivotSimState.getMotorVoltage());
    pivotSimModel.update(LoggedRobot.defaultPeriodSecs);
    pivotSimState.setRawRotorPosition(
        Units.radiansToRotations(pivotSimModel.getAngleRads())
            * ShooterPivotConstants.PivotSim.pivotSimGearing);
    pivotSimState.setRotorVelocity(
        Units.radiansToRotations(pivotSimModel.getVelocityRadPerSec())
            * ShooterPivotConstants.PivotSim.pivotSimGearing);
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(pivotSimModel.getCurrentDrawAmps()));

    super.updateInputs(inputs);
    //SimMechs.getInstance().updatePivot(Radians.of(pivotSimModel.getAngleRads()));
  }
}