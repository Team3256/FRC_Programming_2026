// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utils.PhoenixUtil;

public class IndexerIOTalonFX implements IndexerIO {
  private final TalonFX indexMotor =
      new TalonFX(IndexerConstants.kIndexerMotorID, new CANBus("Gordito"));
  final VelocityVoltage velReq = new VelocityVoltage(0).withSlot(0);

  private final StatusSignal<Voltage> indexerMotorVoltage = indexMotor.getMotorVoltage();
  private final StatusSignal<AngularVelocity> indexerMotorVelocity = indexMotor.getVelocity();
  private final StatusSignal<Current> indexerMotorStatorCurrent = indexMotor.getStatorCurrent();
  private final StatusSignal<Current> indexerMotorSupplyCurrent = indexMotor.getSupplyCurrent();
  private final StatusSignal<Temperature> indexerMotorTemperature = indexMotor.getDeviceTemp();

  public IndexerIOTalonFX() {
    PhoenixUtil.applyMotorConfigs(
        indexMotor, IndexerConstants.motorConfigs, IndexerConstants.flashConfigRetries);

    BaseStatusSignal.setUpdateFrequencyForAll(
        IndexerConstants.updateFrequency,
        indexerMotorVoltage,
        indexerMotorVelocity,
        indexerMotorStatorCurrent,
        indexerMotorSupplyCurrent,
        indexerMotorTemperature);
    PhoenixUtil.registerSignals(
        true,
        indexerMotorVoltage,
        indexerMotorVelocity,
        indexerMotorStatorCurrent,
        indexerMotorSupplyCurrent,
        indexerMotorTemperature);
    indexMotor.optimizeBusUtilization();
  }

  public void updateInputs(IndexerIOInputs inputs) {

    inputs.indexerMotorVoltage = indexerMotorVoltage.getValueAsDouble();
    inputs.indexerMotorVelocity = indexerMotorVelocity.getValueAsDouble();
    inputs.indexerMotorStatorCurrent = indexerMotorStatorCurrent.getValueAsDouble();
    inputs.indexerMotorSupplyCurrent = indexerMotorSupplyCurrent.getValueAsDouble();
    inputs.indexerMotorTemperature = indexerMotorTemperature.getValueAsDouble();
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

  public TalonFX getIndexerMotor() {
    return indexMotor;
  }
}
