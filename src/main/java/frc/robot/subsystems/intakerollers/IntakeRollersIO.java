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
    public double rollerVoltageLeft = 0.0;
    public double rollerVelocityLeft = 0.0;
    public double rollerStatorCurrentLeft = 0.0;
    public double rollerSupplyCurrentLeft = 0.0;
    public double rollerTemperatureLeft = 0.0;

    public double rollerVoltageRight = 0.0;
    public double rollerVelocityRight = 0.0;
    public double rollerSupplyCurrentRight = 0.0;
    public double rollerStatorCurrentRight = 0.0;
    public double rollerTemperatureRight = 0.0;
  }

  public default void updateInputs(IntakeRollersIOInputs inputs) {}

  public default void setVoltage(double voltage) {}

  public default void setVelocity(double velocity) {}

  public default TalonFX getIntakeRollerMotor() {
    return new TalonFX(0);
  }

  public default void off() {}
}
