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
    public double feederMotor1Voltage = 0.0;
    public double feederMotor1Velocity = 0.0;
    public double feederMotor1StatorCurrent = 0.0;
    public double feederMotor1SupplyCurrent = 0.0;
    public double feederMotor1Temperature = 0.0;

    public double feederMotor2Voltage = 0.0;
    public double feederMotor2Velocity = 0.0;
    public double feederMotor2StatorCurrent = 0.0;
    public double feederMotor2SupplyCurrent = 0.0;
    public double feederMotor2Temperature = 0.0;
  }

  public default void updateInputs(FeederIOInputs inputs) {}

  public default void setVoltage(double voltage) {}

  public default void setVelocity(double velocity) {}

  public default TalonFX getFeederMotor1() {
    return new TalonFX(0);
  }

  public default void off() {}
}
