// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.indexer;

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

public class IndexerIOTalonFX implements IndexerIO {
  private final TalonFX indexMotor1 = new TalonFX(IndexerConstants.kIndexerMotor1ID);
  final VelocityVoltage velReq = new VelocityVoltage(0).withSlot(0);

  private final StatusSignal<Voltage> indexerMotor1Voltage = indexMotor1.getMotorVoltage();
  private final StatusSignal<AngularVelocity> indexerMotor1Velocity = indexMotor1.getVelocity();
  private final StatusSignal<Current> indexerMotor1StatorCurrent = indexMotor1.getStatorCurrent();
  private final StatusSignal<Current> indexerMotor1SupplyCurrent = indexMotor1.getSupplyCurrent();
  private final StatusSignal<Temperature> indexerMotor1Temperature = indexMotor1.getDeviceTemp();


  private final TalonFX indexMotor2 = new TalonFX(IndexerConstants.kIndexerMotor2ID);
  final Follower followReq = new Follower(IndexerConstants.kIndexerMotor1ID, MotorAlignmentValue.Aligned);

  private final StatusSignal<Voltage> indexerMotor2Voltage = indexMotor2.getMotorVoltage();
  private final StatusSignal<AngularVelocity> indexerMotor2Velocity = indexMotor2.getVelocity();
  private final StatusSignal<Current> indexerMotor2StatorCurrent = indexMotor2.getStatorCurrent();
  private final StatusSignal<Current> indexerMotor2SupplyCurrent = indexMotor2.getSupplyCurrent();
  private final StatusSignal<Temperature> indexerMotor2Temperature = indexMotor2.getDeviceTemp();


  public IndexerIOTalonFX() {
    PhoenixUtil.applyMotorConfigs(
            indexMotor1, IndexerConstants.motorConfigs, IndexerConstants.flashConfigRetries);

    PhoenixUtil.applyMotorConfigs(indexMotor2, IndexerConstants.motorConfigs, IndexerConstants.flashConfigRetries);

    BaseStatusSignal.setUpdateFrequencyForAll(
        IndexerConstants.updateFrequency,
            indexerMotor1Voltage,
            indexerMotor1Velocity,
            indexerMotor1StatorCurrent,
            indexerMotor1SupplyCurrent,
            indexerMotor1Temperature,
            indexerMotor2Voltage,
            indexerMotor2Velocity,
            indexerMotor2StatorCurrent,
            indexerMotor2SupplyCurrent,
            indexerMotor2Temperature
            );
    PhoenixUtil.registerSignals(
        false,
            indexerMotor1Voltage,
            indexerMotor1Velocity,
            indexerMotor1StatorCurrent,
            indexerMotor1SupplyCurrent,
            indexerMotor1Temperature,
            indexerMotor2Voltage,
            indexerMotor2Velocity,
            indexerMotor2StatorCurrent,
            indexerMotor2SupplyCurrent,
            indexerMotor2Temperature);
    indexMotor1.optimizeBusUtilization();
    indexMotor2.optimizeBusUtilization();

    indexMotor2.setControl(followReq);
  }

  public void updateInputs(IndexerIOInputs inputs) {

    inputs.indexerMotor1Voltage = indexerMotor1Voltage.getValueAsDouble();
    inputs.indexerMotor1Velocity = indexerMotor1Velocity.getValueAsDouble();
    inputs.indexerMotor1StatorCurrent = indexerMotor1StatorCurrent.getValueAsDouble();
    inputs.indexerMotor1SupplyCurrent = indexerMotor1SupplyCurrent.getValueAsDouble();
    inputs.indexerMotor1Temperature = indexerMotor1Temperature.getValueAsDouble();


    inputs.indexerMotor2Voltage = indexerMotor2Voltage.getValueAsDouble();
    inputs.indexerMotor2Velocity = indexerMotor2Velocity.getValueAsDouble();
    inputs.indexerMotor2StatorCurrent = indexerMotor2StatorCurrent.getValueAsDouble();
    inputs.indexerMotor2SupplyCurrent = indexerMotor2SupplyCurrent.getValueAsDouble();
    inputs.indexerMotor2Temperature = indexerMotor2Temperature.getValueAsDouble();
  }

  public void setVoltage(double voltage) {
    indexMotor1.setVoltage(voltage);
  }

  public void setVelocity(double velocity) {
    indexMotor1.setControl(velReq.withVelocity(velocity));
  }

  public void off() {
    indexMotor1.setControl(new NeutralOut());
  }

  public TalonFX getIndexerMotor() {
    return indexMotor1;
  }
}
