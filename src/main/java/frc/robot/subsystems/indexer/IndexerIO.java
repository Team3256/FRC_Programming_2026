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
    public double indexerMotor1Voltage = 0.0;
    public double indexerMotor1Velocity = 0.0;
    public double indexerMotor1StatorCurrent = 0.0;
    public double indexerMotor1SupplyCurrent = 0.0;
    public double indexerMotor1Temperature = 0.0;

    public double indexerMotor2Voltage = 0.0;
    public double indexerMotor2Velocity = 0.0;
    public double indexerMotor2StatorCurrent = 0.0;
    public double indexerMotor2SupplyCurrent = 0.0;
    public double indexerMotor2Temperature = 0.0;
  }

  public default void updateInputs(IndexerIOInputs inputs) {}

  public default void setVoltage(double voltage) {}

  public default void setVelocity(double velocity) {}

  public default TalonFX getIndexerMotor() {
    return new TalonFX(0);
  }

  public default void off() {}
}
