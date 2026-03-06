// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.turret;

import com.ctre.phoenix6.hardware.TalonFX;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  class TurretIOInputs {

    public double turretMotorVoltage = 0.0;
    public double turretMotorVelocity = 0.0;
    public double turretMotorPosition = 0.0;
    public double turretMotorStatorCurrent = 0.0;
    public double turretMotorSupplyCurrent = 0.0;

    public double turretEncoder1AbsolutePosition = 0.0;
    public double turretEncoder1Position = 0.0;
    public double turretEncoder1Velocity = 0.0;

    public double turretEncoder2AbsolutePosition = 0.0;
    public double turretEncoder2Position = 0.0;
    public double turretEncoder2Velocity = 0.0;
  }

  default void updateInputs(TurretIOInputs inputs) {}

  default void setPosition(double position) {}

  default void setVoltage(double voltage) {}

  default TalonFX getMotor() {
    return new TalonFX(0);
  }

  default void zero() {}

  default void resetPosition(double angle) {}

  default void resetPosition() {}

  default void off() {}

  default double blockingGetCC1() {
    return 0;
  }

  default double blockingGetCC2() {
    return 0;
  }
}
