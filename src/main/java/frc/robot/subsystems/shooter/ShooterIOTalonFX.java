// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utils.PhoenixUtil;

public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX shooterMotor =
      new TalonFX(ShooterConstants.shooterMain, new CANBus("Gordito"));
  final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  final MotionMagicVelocityVoltage motionMagicRequest =
      new MotionMagicVelocityVoltage(0).withSlot(0);
  final VelocityTorqueCurrentFOC regenRequest = new VelocityTorqueCurrentFOC(0).withSlot(1);
  private final VoltageOut voltageReq = new VoltageOut(0);

  private final StatusSignal<Voltage> shooterMotorVoltage = shooterMotor.getMotorVoltage();
  private final StatusSignal<AngularVelocity> shooterMotorVelocity = shooterMotor.getVelocity();
  private final StatusSignal<Current> shooterMotorStatorCurrent = shooterMotor.getStatorCurrent();
  private final StatusSignal<Current> shooterMotorSupplyCurrent = shooterMotor.getSupplyCurrent();
  private final StatusSignal<Temperature> shooterMotorTemperature = shooterMotor.getDeviceTemp();

  private final TalonFX shooterMotorFollower = new TalonFX(ShooterConstants.shooterFollower);
  private final Follower followReq =
      new Follower(ShooterConstants.shooterMain, MotorAlignmentValue.Aligned);

  private final StatusSignal<Voltage> shooterMotorFollowerVoltage =
      shooterMotorFollower.getMotorVoltage();
  private final StatusSignal<AngularVelocity> shooterMotorFollowerVelocity =
      shooterMotorFollower.getVelocity();
  private final StatusSignal<Current> shooterMotorFollowerStatorCurrent =
      shooterMotorFollower.getStatorCurrent();
  private final StatusSignal<Current> shooterMotorFollowerSupplyCurrent =
      shooterMotorFollower.getSupplyCurrent();
  private final StatusSignal<Temperature> shooterMotorFollowerTemperature =
      shooterMotorFollower.getDeviceTemp();

  public ShooterIOTalonFX() {
    PhoenixUtil.applyMotorConfigs(
        shooterMotor, ShooterConstants.motorConfigs, ShooterConstants.flashConfigRetries);

    PhoenixUtil.applyMotorConfigs(
        shooterMotorFollower,
        ShooterConstants.followerMotorConfigs,
        ShooterConstants.flashConfigRetries);

    BaseStatusSignal.setUpdateFrequencyForAll(
        ShooterConstants.updateFrequency,
        shooterMotorVoltage,
        shooterMotorVelocity,
        shooterMotorStatorCurrent,
        shooterMotorSupplyCurrent,
        shooterMotorTemperature,
        shooterMotorFollowerVoltage,
        shooterMotorFollowerVelocity,
        shooterMotorFollowerStatorCurrent,
        shooterMotorFollowerSupplyCurrent,
        shooterMotorFollowerTemperature);
    PhoenixUtil.registerSignals(
        false,
        shooterMotorVoltage,
        shooterMotorVelocity,
        shooterMotorStatorCurrent,
        shooterMotorSupplyCurrent,
        shooterMotorTemperature,
        shooterMotorFollowerVoltage,
        shooterMotorFollowerVelocity,
        shooterMotorFollowerStatorCurrent,
        shooterMotorFollowerSupplyCurrent,
        shooterMotorFollowerTemperature);
    shooterMotor.optimizeBusUtilization();
    shooterMotorFollower.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {

    inputs.shooterMotorVoltage = shooterMotorVoltage.getValueAsDouble();
    inputs.shooterMotorVelocity = shooterMotorVelocity.getValueAsDouble();
    inputs.shooterMotorStatorCurrent = shooterMotorStatorCurrent.getValueAsDouble();
    inputs.shooterMotorSupplyCurrent = shooterMotorSupplyCurrent.getValueAsDouble();
    inputs.shooterMotorTemperature = shooterMotorTemperature.getValueAsDouble();

    inputs.shooterMotorFollowerVoltage = shooterMotorFollowerVoltage.getValueAsDouble();
    inputs.shooterMotorFollowerVelocity = shooterMotorFollowerVelocity.getValueAsDouble();
    inputs.shooterMotorFollowerStatorCurrent = shooterMotorFollowerStatorCurrent.getValueAsDouble();
    inputs.shooterMotorFollowerSupplyCurrent = shooterMotorFollowerSupplyCurrent.getValueAsDouble();
    inputs.shooterMotorFollowerTemperature = shooterMotorFollowerTemperature.getValueAsDouble();
  }

  @Override
  public void setShooterVoltage(double voltage) {
    shooterMotor.setVoltage(voltage);
    shooterMotorFollower.setControl(followReq);
  }

  @Override
  public void setShooterVelocity(double velocity) {
    if (ShooterConstants.kUseMotionMagic) {
      shooterMotor.setControl(motionMagicRequest.withVelocity(velocity));
    } else {
      shooterMotor.setControl(velocityRequest.withVelocity(velocity));
    }
    shooterMotor.setControl(followReq);
  }

  @Override
  public void off() {
    if (ShooterConstants.kUseShooterRegenBraking) {
      shooterMotor.setControl(regenRequest);
    } else {
      shooterMotor.setControl(new NeutralOut());
    }
  }

  @Override
  public TalonFX getMotor() {
    return shooterMotor;
  }

  @Override
  public TalonFX getFollowerMotor() {
    return shooterMotorFollower;
  }
}
