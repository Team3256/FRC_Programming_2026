// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.hardware.TalonFX;
import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
  @AutoLog
  public static class FeederIOInputs {
    public double feederMotorVoltage = 0.0;
    public double feederMotorVelocity = 0.0;
    public double feederMotorStatorCurrent = 0.0;
    public double feederMotorSupplyCurrent = 0.0;
    public double feederMotorTemperature = 0.0;
  }

  public default void updateInputs(FeederIOInputs inputs) {}

  public default void setVoltage(double voltage) {}

  public default void setVelocity(double velocity) {}

  public default TalonFX getFeederMotor() {
    return new TalonFX(0);
  }

  public default void off() {}
}
