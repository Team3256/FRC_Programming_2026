// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooterpivot;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterPivotIO {
  @AutoLog
  public class ShooterPivotIOInputs {

    public double shooterPivotMotorVoltage = 0.0;
    public double shooterPivotMotorVelocity = 0.0;
    public double shooterPivotMotorPosition = 0.0;
    public double shooterPivotMotorStatorCurrent = 0.0;
    public double shooterPivotMotorSupplyCurrent = 0.0;
  }

  public default void updateInputs(ShooterPivotIOInputs inputs) {}

  public default void setPosition(Angle position) {}

  public default void setPosition(double position) {}

  public default void setVoltage(double voltage) {}

  public default TalonFX getMotor() {
    return new TalonFX(0);
  }

  public default void resetPosition(Angle angle) {}

  public default void off() {}

  public default void zero() {}
}
