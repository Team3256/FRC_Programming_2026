// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utils.PhoenixUtil;

public class TurretIOTalonFX {
  private final TalonFX indexMotor = new TalonFX(TurretConstants.kTurretMotorID);
  final VelocityVoltage velReq = new VelocityVoltage(0).withSlot(0);

  private final StatusSignal<Voltage> TurretMotorVoltage = indexMotor.getMotorVoltage();
  private final StatusSignal<AngularVelocity> TurretMotorVelocity = indexMotor.getVelocity();
  private final StatusSignal<Current> TurretMotorStatorCurrent = indexMotor.getStatorCurrent();
  private final StatusSignal<Current> TurretMotorSupplyCurrent = indexMotor.getSupplyCurrent();
  private final StatusSignal<Temperature> TurretMotorTemperature = indexMotor.getDeviceTemp();

  public TurretIOTalonFX() {
    PhoenixUtil.applyMotorConfigs(
        indexMotor, TurretConstants.motorConfigs, TurretConstants.flashConfigRetries);

    BaseStatusSignal.setUpdateFrequencyForAll(
        TurretConstants.updateFrequency,
        TurretMotorVoltage,
        TurretMotorVelocity,
        TurretMotorStatorCurrent,
        TurretMotorSupplyCurrent,
        TurretMotorTemperature);
    PhoenixUtil.registerSignals(
        true,
        TurretMotorVoltage,
        TurretMotorVelocity,
        TurretMotorStatorCurrent,
        TurretMotorSupplyCurrent,
        TurretMotorTemperature);
    indexMotor.optimizeBusUtilization();
  }

  public void updateInputs(TurretIO.TurretIOInputs inputs) {
    inputs.turretMotorVoltage = TurretMotorVoltage.getValueAsDouble();
    inputs.turretMotorVelocity = TurretMotorVelocity.getValueAsDouble();
    inputs.turretMotorStatorCurrent = TurretMotorStatorCurrent.getValueAsDouble();
    inputs.turretMotorSupplyCurrent = TurretMotorSupplyCurrent.getValueAsDouble();
    inputs.turretMotorTemperature = TurretMotorTemperature.getValueAsDouble();
  }

  public void setVoltage(double voltage) {
    indexMotor.setVoltage(voltage);
  }

  public void setVelocity(double velocity) {
    indexMotor.setControl(velReq.withVelocity(velocity));
  }

  public void off() {
    indexMotor.setControl(new NeutralOut());
  }

  public TalonFX getTurretMotor() {
    return indexMotor;
  }
}
