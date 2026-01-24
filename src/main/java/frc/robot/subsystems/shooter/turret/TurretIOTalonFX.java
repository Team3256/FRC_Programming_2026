// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter.turret;

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

public class TurretIOTalonFX implements TurretIO {

  private final TalonFX TurretMotor = new TalonFX(TurretConstants.turretMotorId);
  private final MotionMagicVoltage motionMagicRequest =
      new MotionMagicVoltage(0).withSlot(0).withEnableFOC(TurretConstants.kUseFOC);

  private final StatusSignal<Voltage> TurretMotorVoltage = TurretMotor.getMotorVoltage();
  private final StatusSignal<AngularVelocity> TurretMotorVelocity = TurretMotor.getVelocity();
  private final StatusSignal<Angle> TurretMotorPosition = TurretMotor.getPosition();
  private final StatusSignal<Current> TurretMotorStatorCurrent = TurretMotor.getStatorCurrent();
  private final StatusSignal<Current> TurretMotorSupplyCurrent = TurretMotor.getSupplyCurrent();

  public TurretIOTalonFX() {

    PhoenixUtil.applyMotorConfigs(
        TurretMotor, TurretConstants.motorConfigs, TurretConstants.flashConfigRetries);

    BaseStatusSignal.setUpdateFrequencyForAll(
        TurretConstants.updateFrequency,
        TurretMotorVoltage,
        TurretMotorVelocity,
        TurretMotorPosition,
        TurretMotorSupplyCurrent,
        TurretMotorStatorCurrent);

    PhoenixUtil.registerSignals(
        true,
        TurretMotorVoltage,
        TurretMotorVelocity,
        TurretMotorPosition,
        TurretMotorStatorCurrent,
        TurretMotorSupplyCurrent);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.turretMotorVoltage = TurretMotorVoltage.getValue().in(Volt);
    inputs.turretMotorVelocity = TurretMotorVelocity.getValue().in(RotationsPerSecond);
    inputs.turretMotorPosition = TurretMotorPosition.getValue().in(Rotations);
    inputs.turretMotorStatorCurrent = TurretMotorStatorCurrent.getValue().in(Amps);
    inputs.turretMotorSupplyCurrent = TurretMotorSupplyCurrent.getValue().in(Amps);
  }

  @Override
  public void setPosition(double position) {
    TurretMotor.setControl(motionMagicRequest.withPosition(position));
  }

  @Override
  public void setVoltage(double voltage) {
    TurretMotor.setVoltage(voltage);
  }

  @Override
  public void off() {
    TurretMotor.setControl(new NeutralOut());
  }

  @Override
  public TalonFX getMotor() {
    return TurretMotor;
  }

  @Override
  public void resetPosition(Angle angle) {
    TurretMotor.setPosition(angle);
  }
}
