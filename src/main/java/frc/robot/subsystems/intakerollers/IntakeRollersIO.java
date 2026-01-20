// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intakerollers;

import com.ctre.phoenix6.hardware.TalonFX;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeRollersIO {
  @AutoLog
  public static class IntakeRollersIOInputs {
    public double intakeRollerMotorVoltage = 0.0;
    public double intakeRollerMotorVelocity = 0.0;
    public double intakeRollerMotorStatorCurrent = 0.0;
    public double intakeRollerMotorSupplyCurrent = 0.0;
    public double intakeRollerMotorTemperature = 0.0;
  }

  public default void updateInputs(IntakeRollersIOInputs inputs) {}

  public default void setVoltage(double voltage) {}

  public default void setVelocity(double velocity) {}

  public default TalonFX getIntakeRollerMotor() {
    return new TalonFX(0);
  }

  public default void off() {}
}
