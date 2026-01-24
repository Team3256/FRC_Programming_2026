// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.*;
import frc.robot.utils.PhoenixUtil;

public class ClimbIOTalonFX implements ClimbIO {
  private final TalonFX climbMotor = new TalonFX(ClimbConstants.kClimbMotorID);
  private final MotionMagicVoltage motionMagicRequest =
      new MotionMagicVoltage(0).withSlot(0).withEnableFOC(ClimbConstants.kUseFOC);
  private final StatusSignal<Voltage> climbMotorVoltage = climbMotor.getMotorVoltage();
  private final StatusSignal<AngularVelocity> climbMotorVelocity = climbMotor.getVelocity();
  private final StatusSignal<Angle> climbMotorPosition = climbMotor.getPosition();
  private final StatusSignal<Current> climbMotorSupplyCurrent = climbMotor.getSupplyCurrent();
  private final StatusSignal<Current> climbMotorStatorCurrent = climbMotor.getStatorCurrent();

  public ClimbIOTalonFX() {
    PhoenixUtil.applyMotorConfigs(climbMotor, ClimbConstants.motorConfigs, 5);
    BaseStatusSignal.setUpdateFrequencyForAll(
        ClimbConstants.kUpdateFrequency,
        climbMotorVoltage,
        climbMotorVelocity,
        climbMotorPosition,
        climbMotorSupplyCurrent,
        climbMotorStatorCurrent);

    PhoenixUtil.registerSignals(
        false,
        climbMotorVoltage,
        climbMotorVelocity,
        climbMotorPosition,
        climbMotorSupplyCurrent,
        climbMotorStatorCurrent);
    climbMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        climbMotorVoltage,
        climbMotorVelocity,
        climbMotorPosition,
        climbMotorSupplyCurrent,
        climbMotorStatorCurrent);

    inputs.climbMotorVoltage = climbMotorVoltage.getValueAsDouble();
    inputs.climbMotorVelocity = climbMotorVelocity.getValue().in(RotationsPerSecond);
    inputs.climbMotorPosition = climbMotorPosition.getValue().in(Rotations);
    inputs.climbMotorStatorCurrent = climbMotorStatorCurrent.getValueAsDouble();
    inputs.climbMotorSupplyCurrent = climbMotorSupplyCurrent.getValueAsDouble();
  }

  @Override
  public void setPosition(Angle position) {
    climbMotor.setControl(motionMagicRequest.withPosition(position));
  }

  @Override
  public void setVoltage(double voltage) {
    climbMotor.setVoltage(voltage);
  }

  @Override
  public void off() {
    climbMotor.setControl(new NeutralOut());
  }

  @Override
  public void zero() {
    climbMotor.setPosition(0);
  }

  @Override
  public TalonFX getMotor() {
    return climbMotor;
  }
}
