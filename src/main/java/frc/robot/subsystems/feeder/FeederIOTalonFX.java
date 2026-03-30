// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utils.PhoenixUtil;

public class FeederIOTalonFX implements FeederIO {
  private final TalonFX feederMotor1 = new TalonFX(FeederConstants.kFeederMotor1ID);
  final VelocityVoltage velReq = new VelocityVoltage(0).withSlot(0);

  private final StatusSignal<Voltage> feederMotor1Voltage = feederMotor1.getMotorVoltage();
  private final StatusSignal<AngularVelocity> feederMotor1Velocity = feederMotor1.getVelocity();
  private final StatusSignal<Current> feederMotor1StatorCurrent = feederMotor1.getStatorCurrent();
  private final StatusSignal<Current> feederMotor1SupplyCurrent = feederMotor1.getSupplyCurrent();
  private final StatusSignal<Temperature> feederMotor1Temperature = feederMotor1.getDeviceTemp();

  private final TalonFX feederMotor2 = new TalonFX(FeederConstants.kFeederMotor2ID);
  final Follower followReq =
      new Follower(FeederConstants.kFeederMotor1ID, MotorAlignmentValue.Aligned);

  private final StatusSignal<Voltage> feederMotor2Voltage = feederMotor2.getMotorVoltage();
  private final StatusSignal<AngularVelocity> feederMotor2Velocity = feederMotor2.getVelocity();
  private final StatusSignal<Current> feederMotor2StatorCurrent = feederMotor2.getStatorCurrent();
  private final StatusSignal<Current> feederMotor2SupplyCurrent = feederMotor2.getSupplyCurrent();
  private final StatusSignal<Temperature> feederMotor2Temperature = feederMotor2.getDeviceTemp();

  public FeederIOTalonFX() {
    PhoenixUtil.applyMotorConfigs(
        feederMotor1, FeederConstants.motorConfigs, FeederConstants.flashConfigRetries);

    PhoenixUtil.applyMotorConfigs(
        feederMotor2, FeederConstants.motorConfigs, FeederConstants.flashConfigRetries);

    BaseStatusSignal.setUpdateFrequencyForAll(
        FeederConstants.updateFrequency,
        feederMotor1Voltage,
        feederMotor1Velocity,
        feederMotor1StatorCurrent,
        feederMotor1SupplyCurrent,
        feederMotor1Temperature,
        feederMotor2Voltage,
        feederMotor2Velocity,
        feederMotor2StatorCurrent,
        feederMotor2SupplyCurrent,
        feederMotor2Temperature);
    PhoenixUtil.registerSignals(
        false,
        feederMotor1Voltage,
        feederMotor1Velocity,
        feederMotor1StatorCurrent,
        feederMotor1SupplyCurrent,
        feederMotor1Temperature,
        feederMotor2Voltage,
        feederMotor2Velocity,
        feederMotor2StatorCurrent,
        feederMotor2SupplyCurrent,
        feederMotor2Temperature);
    feederMotor1.optimizeBusUtilization();
    feederMotor2.optimizeBusUtilization();

    feederMotor2.setControl(followReq);
  }

  public void updateInputs(FeederIOInputs inputs) {

    inputs.feederMotor1Voltage = feederMotor1Voltage.getValueAsDouble();
    inputs.feederMotor1Velocity = feederMotor1Velocity.getValueAsDouble();
    inputs.feederMotor1StatorCurrent = feederMotor1StatorCurrent.getValueAsDouble();
    inputs.feederMotor1SupplyCurrent = feederMotor1SupplyCurrent.getValueAsDouble();
    inputs.feederMotor1Temperature = feederMotor1Temperature.getValueAsDouble();

    inputs.feederMotor2Voltage = feederMotor2Voltage.getValueAsDouble();
    inputs.feederMotor2Velocity = feederMotor2Velocity.getValueAsDouble();
    inputs.feederMotor2StatorCurrent = feederMotor2StatorCurrent.getValueAsDouble();
    inputs.feederMotor2SupplyCurrent = feederMotor2SupplyCurrent.getValueAsDouble();
    inputs.feederMotor2Temperature = feederMotor2Temperature.getValueAsDouble();
  }

  public void setVoltage(double voltage) {
    feederMotor1.setVoltage(voltage);
  }

  public void setVelocity(double velocity) {
    feederMotor1.setControl(velReq.withVelocity(velocity));
  }

  public void off() {
    feederMotor1.setControl(new NeutralOut());
  }

  public TalonFX getFeederMotor1() {
    return feederMotor1;
  }
}
