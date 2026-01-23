// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter.shooterpivot;

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

  default void updateInputs(ShooterPivotIOInputs inputs) {}

  default void setPosition(double position) {}

  default void setVoltage(double voltage) {}

  default TalonFX getMotor() {
    return new TalonFX(0);
  }

  default void zero() {}

  default void resetPosition(Angle angle) {}

  default void off() {}
}
