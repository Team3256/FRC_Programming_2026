// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utils.PhoenixUtil;

public class TurretIOTalonFX implements TurretIO {

  private final TalonFX turretMotor = new TalonFX(TurretConstants.turretMotorId);
  private final MotionMagicVoltage motionMagicRequest =
      new MotionMagicVoltage(0).withSlot(0).withEnableFOC(TurretConstants.kUseFOC);

  private final StatusSignal<Voltage> turretMotorVoltage = turretMotor.getMotorVoltage();
  private final StatusSignal<AngularVelocity> turretMotorVelocity = turretMotor.getVelocity();
  private final StatusSignal<Angle> turretMotorPosition = turretMotor.getPosition();
  private final StatusSignal<Current> turretMotorStatorCurrent = turretMotor.getStatorCurrent();
  private final StatusSignal<Current> turretMotorSupplyCurrent = turretMotor.getSupplyCurrent();

  private final CANcoder cancoder1 = new CANcoder(TurretConstants.turretEncoder1);
  private final CANcoder cancoder2 = new CANcoder(TurretConstants.turretEncoder2);

  private final StatusSignal<Angle> cancoder1AbsolutePosition = cancoder1.getAbsolutePosition();
  private final StatusSignal<Angle> cancoder1Position = cancoder1.getPosition();
  private final StatusSignal<AngularVelocity> cancoder1Velocity = cancoder1.getVelocity();

  private final StatusSignal<Angle> cancoder2AbsolutePosition = cancoder2.getAbsolutePosition();
  private final StatusSignal<Angle> cancoder2Position = cancoder2.getPosition();
  private final StatusSignal<AngularVelocity> cancoder2Velocity = cancoder2.getVelocity();

  public TurretIOTalonFX() {

    PhoenixUtil.applyMotorConfigs(
        turretMotor, TurretConstants.motorConfigs, TurretConstants.flashConfigRetries);

    PhoenixUtil.applyCancoderConfig(
        cancoder1, TurretConstants.cancoderConfiguration1, TurretConstants.flashConfigRetries);

    PhoenixUtil.applyCancoderConfig(
        cancoder2, TurretConstants.cancoderConfiguration2, TurretConstants.flashConfigRetries);

    BaseStatusSignal.setUpdateFrequencyForAll(
        TurretConstants.updateFrequency,
        turretMotorVoltage,
        turretMotorVelocity,
        turretMotorPosition,
        turretMotorSupplyCurrent,
        turretMotorStatorCurrent);

    PhoenixUtil.registerSignals(
        false,
        turretMotorVoltage,
        turretMotorVelocity,
        turretMotorPosition,
        turretMotorStatorCurrent,
        turretMotorSupplyCurrent);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.turretMotorVoltage = turretMotorVoltage.getValue().in(Volt);
    inputs.turretMotorVelocity = turretMotorVelocity.getValue().in(RotationsPerSecond);
    inputs.turretMotorPosition = turretMotorPosition.getValue().in(Rotations);
    inputs.turretMotorStatorCurrent = turretMotorStatorCurrent.getValue().in(Amps);
    inputs.turretMotorSupplyCurrent = turretMotorSupplyCurrent.getValue().in(Amps);
    inputs.turretEncoder1Position = cancoder1Position.getValue().in(Rotations);
    inputs.turretEncoder1Velocity = cancoder1Velocity.getValue().in(RotationsPerSecond);
    inputs.turretEncoder1Position = cancoder1AbsolutePosition.getValue().in(Rotations);

    inputs.turretEncoder2Position = cancoder2Position.getValue().in(Rotations);
    inputs.turretEncoder2Velocity = cancoder2Velocity.getValue().in(RotationsPerSecond);
    inputs.turretEncoder2Position = cancoder2AbsolutePosition.getValue().in(Rotations);
  }

  @Override
  public void setPosition(double position) {
    turretMotor.setControl(motionMagicRequest.withPosition(position));
  }

  @Override
  public void setVoltage(double voltage) {
    turretMotor.setVoltage(voltage);
  }

  @Override
  public void off() {
    turretMotor.setControl(new NeutralOut());
  }

  @Override
  public TalonFX getMotor() {
    return turretMotor;
  }

  @Override
  public void resetPosition(Angle angle) {
    turretMotor.setPosition(angle);
  }
}
