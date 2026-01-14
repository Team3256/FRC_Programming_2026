package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
    public double shooterMotorVoltage = 0.0;
    public double shooterMotorVelocity = 0.0;
    public double shooterMotorStatorCurrent = 0.0;
    public double shooterMotorSupplyCurrent = 0.0;
    public double shooterMotorTemperature = 0.0;
    public double shooterMotorReferenceSlope = 0.0;

    public double shooterMotorFollowerVoltage = 0.0;
    public double shooterMotorFollowerVelocity = 0.0;
    public double shooterMotorFollowerStatorCurrent = 0.0;
    public double shooterMotorFollowerSupplyCurrent = 0.0;
    public double shooterMotorFollowerTemperature = 0.0;
    public double shooterMotorFollowerReferenceSlope = 0.0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setShooterVoltage(double voltage) {}

  public default void setShooterVelocity(double velocity) {}

  public default void setShooterFollowerVoltage(double voltage) {}

  public default void setShooterFollowerVelocity(double velocity) {}

  public default TalonFX getMotor() {
    return new TalonFX(0);
  }

  public default TalonFX getFollowerMotor() {
    return new TalonFX(0);
  }

  public default VoltageOut getVoltageRequest() {
    return new VoltageOut(0);
  }

  public default void off() {}
}