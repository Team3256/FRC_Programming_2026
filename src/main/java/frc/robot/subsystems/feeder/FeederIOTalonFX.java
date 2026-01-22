// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.feeder;

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

public class FeederIOTalonFX implements FeederIO {
  private final TalonFX feederMotor = new TalonFX(FeederConstants.kFeederMotorID);
  final VelocityVoltage velReq = new VelocityVoltage(0).withSlot(0);

  private final StatusSignal<Voltage> feederMotorVoltage = feederMotor.getMotorVoltage();
  private final StatusSignal<AngularVelocity> feederMotorVelocity = feederMotor.getVelocity();
  private final StatusSignal<Current> feederMotorStatorCurrent = feederMotor.getStatorCurrent();
  private final StatusSignal<Current> feederMotorSupplyCurrent = feederMotor.getSupplyCurrent();
  private final StatusSignal<Temperature> feederMotorTemperature = feederMotor.getDeviceTemp();

  public FeederIOTalonFX() {
    PhoenixUtil.applyMotorConfigs(
        feederMotor, FeederConstants.motorConfigs, FeederConstants.flashConfigRetries);

    BaseStatusSignal.setUpdateFrequencyForAll(
        FeederConstants.updateFrequency,
        feederMotorVoltage,
        feederMotorVelocity,
        feederMotorStatorCurrent,
        feederMotorSupplyCurrent,
        feederMotorTemperature);
    PhoenixUtil.registerSignals(
        true,
        feederMotorVoltage,
        feederMotorVelocity,
        feederMotorSupplyCurrent,
        feederMotorTemperature);
    feederMotor.optimizeBusUtilization();
  }

  public void updateInputs(FeederIOInputs inputs) {

    inputs.feederMotorVoltage = feederMotorVoltage.getValueAsDouble();
    inputs.feederMotorVelocity = feederMotorVelocity.getValueAsDouble();
    inputs.feederMotorStatorCurrent = feederMotorStatorCurrent.getValueAsDouble();
    inputs.feederMotorSupplyCurrent = feederMotorSupplyCurrent.getValueAsDouble();
    inputs.feederMotorTemperature = feederMotorTemperature.getValueAsDouble();
  }

  public void setVoltage(double voltage) {
    feederMotor.setVoltage(voltage);
  }

  public void setVelocity(double velocity) {
    feederMotor.setControl(velReq.withVelocity(velocity));
  }

  public void off() {
    feederMotor.setControl(new NeutralOut());
  }

  public TalonFX getFeederMotor() {
    return feederMotor;
  }
}
