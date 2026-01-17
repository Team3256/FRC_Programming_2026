// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.hardware.TalonFX;
import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public static class IndexerIOInputs {
    public double indexerMotorVoltage = 0.0;
    public double indexerMotorVelocity = 0.0;
    public double indexerMotorStatorCurrent = 0.0;
    public double indexerMotorSupplyCurrent = 0.0;
    public double indexerMotorTemperature = 0.0;
  }

  public static void updateInputs(IndexerIOInputs inputs) {}

  public static void setVoltage(double voltage) {}

  public static void setVelocity(double velocity) {}

  public default TalonFX getIndexerMotor() {
    return new TalonFX(0);
  }

  public static void off() {}
}
