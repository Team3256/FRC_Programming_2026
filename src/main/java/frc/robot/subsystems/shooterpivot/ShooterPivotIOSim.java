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
import frc.robot.sim.MechanismSimLoop;
import frc.robot.sim.SimMechs;

public class ShooterPivotIOSim extends ShooterPivotIOTalonFX {

  private final SingleJointedArmSim pivotSimModel =
      new SingleJointedArmSim(
          DCMotor.getKrakenX44(1),
          ShooterPivotConstants.PivotSim.pivotSimGearing,
          ShooterPivotConstants.PivotSim.jkGMetersSquared,
          ShooterPivotConstants.PivotSim.shooterPivotLength.in(Meters),
          ShooterPivotConstants.PivotSim.minAngle.getRadians(),
          ShooterPivotConstants.PivotSim.maxAngle.getRadians(),
          false,
          ShooterPivotConstants.PivotSim.startingAngle.getRadians());

  private final TalonFXSimState pivotSimState;

  public ShooterPivotIOSim() {
    super();
    pivotSimState = super.getMotor().getSimState();
    // Must match the motor's inversion, or the sim runs the hood into its lower limit
    pivotSimState.Orientation = ChassisReference.CounterClockwise_Positive;
    // The hood runs a Kraken X44 (CAD). The TalonFX sim assumes an X60 otherwise, misjudges the
    // back-EMF, and its current limit then keeps it from braking, so the hood slams end to end
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
            * ShooterPivotConstants.PivotSim.pivotSimGearing);
    pivotSimState.setRotorVelocity(
        Units.radiansToRotations(pivotSimModel.getVelocityRadPerSec())
            * ShooterPivotConstants.PivotSim.pivotSimGearing);
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(pivotSimModel.getCurrentDrawAmps()));
  }

  @Override
  public void updateInputs(ShooterPivotIOInputs inputs) {
    super.updateInputs(inputs);
    SimMechs.getInstance().updateShooterPivot(Radians.of(pivotSimModel.getAngleRads()));
  }
}
