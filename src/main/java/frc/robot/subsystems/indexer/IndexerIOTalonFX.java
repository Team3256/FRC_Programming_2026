// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.indexer;

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

public class IndexerIOTalonFX implements IndexerIO {
    private final TalonFX intakeMotor =
            new TalonFX(IndexerConstants.kIndexerMotorID, "bruh"); // TODO: remove deprecated
    final VelocityVoltage intakeRequest = new VelocityVoltage(0).withSlot(0);

    private final StatusSignal<Voltage> IndexerMotorVoltage = intakeMotor.getMotorVoltage();
    private final StatusSignal<AngularVelocity> IndexerMotorVelocity = intakeMotor.getVelocity();
    private final StatusSignal<Current> IndexerMotorStatorCurrent = intakeMotor.getStatorCurrent();
    private final StatusSignal<Current> IndexerMotorSupplyCurrent = intakeMotor.getSupplyCurrent();
    private final StatusSignal<Temperature> IndexerMotorTemperature = intakeMotor.getDeviceTemp();

    public IndexerIOTalonFX() {
        PhoenixUtil.applyMotorConfigs(
                intakeMotor, IndexerConstants.motorConfigs, IndexerConstants.flashConfigRetries);

        BaseStatusSignal.setUpdateFrequencyForAll(
                IndexerConstants.updateFrequency,
                IndexerMotorVoltage,
                IndexerMotorVelocity,
                IndexerMotorStatorCurrent,
                IndexerMotorSupplyCurrent,
                IndexerMotorTemperature);
        PhoenixUtil.registerSignals(
                true,
                IndexerMotorVoltage,
                IndexerMotorVelocity,
                IndexerMotorStatorCurrent,
                IndexerMotorSupplyCurrent,
                IndexerMotorTemperature);
        intakeMotor.optimizeBusUtilization();
    }

    public void updateInputs(IndexerIOInputs inputs) {

        inputs.indexerMotorVoltage = IndexerMotorVoltage.getValueAsDouble();
        inputs.indexerMotorVelocity = IndexerMotorVelocity.getValueAsDouble();
        inputs.indexerMotorStatorCurrent = IndexerMotorStatorCurrent.getValueAsDouble();
        inputs.indexerMotorSupplyCurrent = IndexerMotorSupplyCurrent.getValueAsDouble();
        inputs.indexerMotorTemperature = IndexerMotorTemperature.getValueAsDouble();
    }

    public void setVoltage(double voltage) {
        intakeMotor.setVoltage(voltage);
    }

    public void setVelocity(double velocity) {
        intakeMotor.setControl(intakeRequest.withVelocity(velocity));
    }

    public void off() {
        intakeMotor.setControl(new NeutralOut());
    }

    public TalonFX getIndexerMotor() {
        return intakeMotor;
    }
}