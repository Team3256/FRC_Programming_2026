// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intakepivot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.sim.SimMechs;

public class IntakePivotIOSim extends IntakePivotIOTalonFX {

  private final TalonFXSimState pivotSimState;

  private final SingleJointedArmSim pivotSimModel =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(1),
          IntakePivotConstants.PivotSim.pivotSimGearing,
          IntakePivotConstants.PivotSim.jkGMetersSquared,
          IntakePivotConstants.PivotSim.intakePivotLength.in(Meters),
          IntakePivotConstants.PivotSim.minAngle.getRadians(),
          IntakePivotConstants.PivotSim.maxAngle.getRadians(),
          true,
          IntakePivotConstants.PivotSim.startingAngle.getRadians());

  public IntakePivotIOSim() {
    super();
    pivotSimState = super.getMotor().getSimState();
    pivotSimState.Orientation = ChassisReference.Clockwise_Positive;
  }

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {

    pivotSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    pivotSimModel.setInputVoltage(pivotSimState.getMotorVoltage());
    pivotSimModel.update(TimedRobot.kDefaultPeriod);

    double rotorPosRotations =
        (pivotSimModel.getAngleRads() / (2 * Math.PI))
            * IntakePivotConstants.PivotSim.pivotSimGearing;
    double rotorVelRps =
        (pivotSimModel.getVelocityRadPerSec() / (2 * Math.PI))
            * IntakePivotConstants.PivotSim.pivotSimGearing;

    pivotSimState.setRawRotorPosition(rotorPosRotations);
    pivotSimState.setRotorVelocity(rotorVelRps);

    super.updateInputs(inputs);
    SimMechs.getInstance().updatePivot(Radians.of(pivotSimModel.getAngleRads()));
  }
}
