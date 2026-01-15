package frc.robot.subsystems.intakerollers;

import com.ctre.phoenix6.hardware.CANrange;
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
    public double canRangeDistance = 0.0;
  }

  public default void updateInputs(IntakeRollersIOInputs inputs) {}

  public default void setVoltage(double voltage) {}

  public default void setVelocity(double velocity) {}

  public default TalonFX getIntakeRollerMotor() {
    return new TalonFX(0);
  }

  public default CANrange getCanRange() {
    return new CANrange(0);
  }

  public default void off() {}
}
