package frc.robot.subsystems.intakepivot;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.hardware.TalonFX;

public interface IntakePivotIO {
  @AutoLog
  public static class IntakePivotIOInputs {
    public double pivotMotorVoltage = 0.0;
    public double pivotMotorVelocity = 0.0;
    public double pivotMotorPosition = 0.0;
    public double pivotMotorStatorCurrent = 0.0;
    public double pivotMotorSupplyCurrent = 0.0;
  }

  public default TalonFX getMotor() {
    return new TalonFX(0);
  };

  default void updateInputs(IntakePivotIOInputs inputs) {}

  default void setPosition(double position) {}

  default void setVoltage(double voltage) {}

  default void stop() {}

  default void resetPosition(double angle) {}

  public default void off() {}

  public default void zero() {}

}