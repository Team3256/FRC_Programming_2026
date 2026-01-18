// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  public static class TurretIOInputs {
    double turretMotorVoltage = 0.0;
    double turretMotorVelocity = 0.0;
    double turretMotorStatorCurrent = 0.0;
    double turretMotorSupplyCurrent = 0.0;
    double turretMotorTemperature = 0.0;

  }

  public static enum TurretIOOutputMode {
    BRAKE,
    COAST,
    CLOSED_LOOP
  }

  public static class TurretIOOutputs {
    public TurretIOOutputMode mode = TurretIOOutputMode.BRAKE;

    // closed loop control TODO: Fill
    public double position = 0.0;
    public double velocity = 0.0;
    public double kP = 0.0;
    public double kD = 0.0;
  }

  public default void updateInputs(TurretIOInputs inputs) {}

  public default void applyOutputs(TurretIOOutputs outputs) {}
}
