// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter.shooterpivot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utils.PhoenixUtil;

public class ShooterPivotIOTalonFX implements ShooterPivotIO {

  private final TalonFX shooterPivotMotor = new TalonFX(ShooterPivotConstants.pivotMotorId);
  private final MotionMagicVoltage motionMagicRequest =
      new MotionMagicVoltage(0).withSlot(0).withEnableFOC(ShooterPivotConstants.kUseFOC);

  private final StatusSignal<Voltage> shooterPivotMotorVoltage =
      shooterPivotMotor.getMotorVoltage();
  private final StatusSignal<AngularVelocity> shooterPivotMotorVelocity =
      shooterPivotMotor.getVelocity();
  private final StatusSignal<Angle> shooterPivotMotorPosition = shooterPivotMotor.getPosition();
  private final StatusSignal<Current> shooterPivotMotorStatorCurrent =
      shooterPivotMotor.getStatorCurrent();
  private final StatusSignal<Current> shooterPivotMotorSupplyCurrent =
      shooterPivotMotor.getSupplyCurrent();

  public ShooterPivotIOTalonFX() {

    PhoenixUtil.applyMotorConfigs(
        shooterPivotMotor,
        ShooterPivotConstants.motorConfigs,
        ShooterPivotConstants.flashConfigRetries);

    BaseStatusSignal.setUpdateFrequencyForAll(
        ShooterPivotConstants.updateFrequency,
        shooterPivotMotorVoltage,
        shooterPivotMotorVelocity,
        shooterPivotMotorPosition,
        shooterPivotMotorSupplyCurrent,
        shooterPivotMotorStatorCurrent);

    PhoenixUtil.registerSignals(
        true,
        shooterPivotMotorVoltage,
        shooterPivotMotorVelocity,
        shooterPivotMotorPosition,
        shooterPivotMotorStatorCurrent,
        shooterPivotMotorSupplyCurrent);
  }

  @Override
  public void updateInputs(ShooterPivotIOInputs inputs) {
    inputs.shooterPivotMotorVoltage = shooterPivotMotorVoltage.getValue().in(Volt);
    inputs.shooterPivotMotorVelocity = shooterPivotMotorVelocity.getValue().in(RotationsPerSecond);
    inputs.shooterPivotMotorPosition = shooterPivotMotorPosition.getValue().in(Rotations);
    inputs.shooterPivotMotorStatorCurrent = shooterPivotMotorStatorCurrent.getValue().in(Amps);
    inputs.shooterPivotMotorSupplyCurrent = shooterPivotMotorSupplyCurrent.getValue().in(Amps);
  }

  @Override
  public void setPosition(double position) {
    shooterPivotMotor.setControl(motionMagicRequest.withPosition(position));
  }

  @Override
  public void setVoltage(double voltage) {
    shooterPivotMotor.setVoltage(voltage);
  }

  @Override
  public void off() {
    shooterPivotMotor.setControl(new NeutralOut());
  }

  @Override
  public TalonFX getMotor() {
    return shooterPivotMotor;
  }

  @Override
  public void resetPosition(Angle angle) {
    shooterPivotMotor.setPosition(angle);
  }
}
