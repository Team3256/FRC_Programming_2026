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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.sim.MechanismSimLoop;
import frc.robot.sim.SimMechs;

public class IntakePivotIOSim extends IntakePivotIOTalonFX {

  private final TalonFXSimState pivotSimState;

  private final SingleJointedArmSim pivotSimModel =
      new SingleJointedArmSim(
          DCMotor.getKrakenX44(1),
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
    // The pivot runs a Kraken X44 (CAD); see ShooterPivotIOSim
    pivotSimState.setMotorType(TalonFXSimState.MotorType.KrakenX44);
    MechanismSimLoop.register(this::step);
  }

  /** Steps the physics, faster than the robot loop so the TalonFX's position loop stays stable. */
  private void step(double dtSeconds) {
    pivotSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    pivotSimModel.setInputVoltage(pivotSimState.getMotorVoltage());
    pivotSimModel.update(dtSeconds);
    pivotSimState.setRawRotorPosition(
        Units.radiansToRotations(pivotSimModel.getAngleRads())
            * IntakePivotConstants.PivotSim.pivotSimGearing);
    pivotSimState.setRotorVelocity(
        Units.radiansToRotations(pivotSimModel.getVelocityRadPerSec())
            * IntakePivotConstants.PivotSim.pivotSimGearing);
  }

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {
    super.updateInputs(inputs);
    SimMechs.getInstance().updatePivot(Radians.of(pivotSimModel.getAngleRads()));
  }
}
