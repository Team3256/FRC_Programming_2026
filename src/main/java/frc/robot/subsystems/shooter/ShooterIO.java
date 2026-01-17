// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double shooterMotorVoltage = 0.0;
    public double shooterMotorVelocity = 0.0;
    public double shooterMotorStatorCurrent = 0.0;
    public double shooterMotorSupplyCurrent = 0.0;
    public double shooterMotorTemperature = 0.0;

    public double shooterMotorFollowerVoltage = 0.0;
    public double shooterMotorFollowerVelocity = 0.0;
    public double shooterMotorFollowerStatorCurrent = 0.0;
    public double shooterMotorFollowerSupplyCurrent = 0.0;
    public double shooterMotorFollowerTemperature = 0.0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setShooterVoltage(double voltage) {}

  public default void setShooterVelocity(double velocity) {}


  public default TalonFX getMotor() {
    return new TalonFX(0);
  }

  public default TalonFX getFollowerMotor() {
    return new TalonFX(0);
  }

  public default void off() {}
}
