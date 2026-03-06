// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intakerollers;

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
import frc.robot.subsystems.intakepivot.IntakePivotConstants;
import frc.robot.utils.PhoenixUtil;

public class IntakeRollersIOTalonFX implements IntakeRollersIO {
  private final TalonFX intakeMotorLeft = new TalonFX(IntakeRollersConstants.kIntakeRollerMotorIDLeft);
  final VelocityVoltage intakeRequest = new VelocityVoltage(0).withSlot(0);

  private final StatusSignal<Voltage> rollerVoltageLeft = intakeMotorLeft.getMotorVoltage();
  private final StatusSignal<AngularVelocity> rollerVelocityLeft = intakeMotorLeft.getVelocity();
  private final StatusSignal<Current> rollerStatorCurrentLeft =
      intakeMotorLeft.getStatorCurrent();
  private final StatusSignal<Current> rollerSupplyCurrentLeft =
      intakeMotorLeft.getSupplyCurrent();
  private final StatusSignal<Temperature> rollerTemperatureLeft =
      intakeMotorLeft.getDeviceTemp();


  private final TalonFX intakeMotorRight = new TalonFX(IntakeRollersConstants.kIntakeRollerMotorIDRight);
  final Follower followReq = new Follower(IntakeRollersConstants.kIntakeRollerMotorIDLeft, MotorAlignmentValue.Opposed);

  private final StatusSignal<Voltage> rollerVoltageRight = intakeMotorRight.getMotorVoltage();
  private final StatusSignal<AngularVelocity> rollerVelocityRight = intakeMotorRight.getVelocity();
  private final StatusSignal<Current> rollerStatorCurrentRight =
      intakeMotorRight.getStatorCurrent();
  private final StatusSignal<Current> rollerSupplyCurrentRight =
      intakeMotorRight.getSupplyCurrent();
  private final StatusSignal<Temperature> rollerTemperatureRight =
      intakeMotorRight.getDeviceTemp();

  public IntakeRollersIOTalonFX() {
    PhoenixUtil.applyMotorConfigs(
            intakeMotorLeft,
        IntakeRollersConstants.motorConfigs,
        IntakeRollersConstants.flashConfigRetries);
    PhoenixUtil.applyMotorConfigs(
            intakeMotorRight,
            IntakePivotConstants.motorConfigs,
            IntakePivotConstants.flashConfigRetries
    );





    BaseStatusSignal.setUpdateFrequencyForAll(
        IntakeRollersConstants.updateFrequency,
            rollerVoltageLeft,
            rollerVelocityLeft,
            rollerStatorCurrentLeft,
            rollerSupplyCurrentLeft,
            rollerTemperatureLeft,
            rollerVoltageRight,
            rollerVelocityRight,
            rollerSupplyCurrentRight,
            rollerStatorCurrentRight,
            rollerTemperatureRight);
    PhoenixUtil.registerSignals(
        false,
            rollerVoltageLeft,
            rollerVelocityLeft,
            rollerStatorCurrentLeft,
            rollerSupplyCurrentLeft,
            rollerTemperatureLeft,
            rollerVoltageRight,
            rollerVelocityRight,
            rollerSupplyCurrentRight,
            rollerStatorCurrentRight,
            rollerTemperatureRight);
    intakeMotorLeft.optimizeBusUtilization();
    intakeMotorRight.optimizeBusUtilization();


    intakeMotorRight.setControl(followReq);
  }

  @Override
  public void updateInputs(IntakeRollersIOInputs inputs) {

    inputs.rollerVoltageLeft = rollerVoltageLeft.getValueAsDouble();
    inputs.rollerVelocityLeft = rollerVelocityLeft.getValueAsDouble();
    inputs.rollerStatorCurrentLeft = rollerStatorCurrentLeft.getValueAsDouble();
    inputs.rollerSupplyCurrentLeft = rollerSupplyCurrentLeft.getValueAsDouble();
    inputs.rollerTemperatureLeft = rollerTemperatureLeft.getValueAsDouble();

    inputs.rollerVoltageRight = rollerVoltageRight.getValueAsDouble();
    inputs.rollerVelocityRight = rollerVelocityRight.getValueAsDouble();
    inputs.rollerSupplyCurrentRight = rollerSupplyCurrentRight.getValueAsDouble();
    inputs.rollerStatorCurrentRight = rollerStatorCurrentRight.getValueAsDouble();
    inputs.rollerTemperatureRight = rollerTemperatureRight.getValueAsDouble();
  }

  @Override
  public void setVoltage(double voltage) {
    intakeMotorLeft.setVoltage(voltage);
    intakeMotorRight.setControl(followReq);
  }

  @Override
  public void setVelocity(double velocity) {

    intakeMotorLeft.setControl(intakeRequest.withVelocity(velocity));
    intakeMotorRight.setControl(followReq);
  }

  @Override
  public void off() {
    intakeMotorLeft.setControl(new NeutralOut());
  }

  @Override
  public TalonFX getIntakeRollerMotor() {
    return intakeMotorLeft;
  }
}
