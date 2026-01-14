package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utils.PhoenixUtil;

public class ShooterIOTalonFX implements ShooterIO {
      private final TalonFX shooterMotor = new TalonFX(ShooterConstants.kShooterMotorRightID);
  final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  final MotionMagicVelocityVoltage motionMagicRequest =
      new MotionMagicVelocityVoltage(0).withSlot(0);
  final VelocityTorqueCurrentFOC regenRequest = new VelocityTorqueCurrentFOC(0).withSlot(1);
  private final VoltageOut voltageReq = new VoltageOut(0);

  private final StatusSignal<Voltage> shooterMotorVoltage = shooterMotor.getMotorVoltage();
  private final StatusSignal<AngularVelocity> shooterMotorVelocity = shooterMotor.getVelocity();
  private final StatusSignal<Current> shooterMotorStatorCurrent = shooterMotor.getStatorCurrent();
  private final StatusSignal<Current> shooterMotorSupplyCurrent = shooterMotor.getSupplyCurrent();
  private final StatusSignal<Temperature> shooterMotorTemperature = shooterMotor.getDeviceTemp();
  private final StatusSignal<Double> shooterMotorReferenceSlope =
      shooterMotor.getClosedLoopReferenceSlope();

  private final TalonFX shooterMotorFollower = new TalonFX(ShooterConstants.kShooterMotorLeftID);
  final VelocityVoltage velocityRequestFollower = new VelocityVoltage(0).withSlot(0);
  final MotionMagicVelocityVoltage motionMagicRequestFollower =
      new MotionMagicVelocityVoltage(0).withSlot(0);

  private final StatusSignal<Voltage> shooterMotorFollowerVoltage =
      shooterMotorFollower.getMotorVoltage();
  private final StatusSignal<AngularVelocity> shooterMotorFollowerVelocity =
      shooterMotorFollower.getVelocity();
  private final StatusSignal<Current> shooterMotorFollowerStatorCurrent =
      shooterMotorFollower.getStatorCurrent();
  private final StatusSignal<Current> shooterMotorFollowerSupplyCurrent =
      shooterMotorFollower.getSupplyCurrent();
  private final StatusSignal<Temperature> shooterMotorFollowerTemperature =
      shooterMotorFollower.getDeviceTemp();
  private final StatusSignal<Double> shooterMotorFollowerReferenceSlope =
      shooterMotorFollower.getClosedLoopReferenceSlope();

  public ShooterIOTalonFX() {
    PhoenixUtil.applyMotorConfigs(
        shooterMotor, ShooterConstants.motorConfigs, ShooterConstants.flashConfigRetries);

    PhoenixUtil.applyMotorConfigs(
        shooterMotorFollower,
        ShooterConstants.followerMotorConfigs,
        ShooterConstants.flashConfigRetries);

    BaseStatusSignal.setUpdateFrequencyForAll(
        ShooterConstants.updateFrequency,
        shooterMotorVoltage,
        shooterMotorVelocity,
        shooterMotorStatorCurrent,
        shooterMotorSupplyCurrent,
        shooterMotorTemperature,
        shooterMotorReferenceSlope,
        shooterMotorFollowerVoltage,
        shooterMotorFollowerVelocity,
        shooterMotorFollowerStatorCurrent,
        shooterMotorFollowerSupplyCurrent,
        shooterMotorFollowerTemperature,
        shooterMotorFollowerReferenceSlope);
    shooterMotor.optimizeBusUtilization();
    shooterMotorFollower.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        shooterMotorVoltage,
        shooterMotorVelocity,
        shooterMotorStatorCurrent,
        shooterMotorSupplyCurrent,
        shooterMotorTemperature,
        shooterMotorReferenceSlope,
        shooterMotorFollowerVoltage,
        shooterMotorFollowerVelocity,
        shooterMotorFollowerStatorCurrent,
        shooterMotorFollowerSupplyCurrent,
        shooterMotorFollowerTemperature,
        shooterMotorFollowerReferenceSlope);
    inputs.shooterMotorVoltage = shooterMotorVoltage.getValueAsDouble();
    inputs.shooterMotorVelocity = shooterMotorVelocity.getValueAsDouble();
    inputs.shooterMotorStatorCurrent = shooterMotorStatorCurrent.getValueAsDouble();
    inputs.shooterMotorSupplyCurrent = shooterMotorSupplyCurrent.getValueAsDouble();
    inputs.shooterMotorTemperature = shooterMotorTemperature.getValueAsDouble();
    inputs.shooterMotorReferenceSlope = shooterMotorReferenceSlope.getValueAsDouble();

    inputs.shooterMotorFollowerVoltage = shooterMotorFollowerVoltage.getValueAsDouble();
    inputs.shooterMotorFollowerVelocity = shooterMotorFollowerVelocity.getValueAsDouble();
    inputs.shooterMotorFollowerStatorCurrent = shooterMotorFollowerStatorCurrent.getValueAsDouble();
    inputs.shooterMotorFollowerSupplyCurrent = shooterMotorFollowerSupplyCurrent.getValueAsDouble();
    inputs.shooterMotorFollowerTemperature = shooterMotorFollowerTemperature.getValueAsDouble();
    inputs.shooterMotorFollowerReferenceSlope =
        shooterMotorFollowerReferenceSlope.getValueAsDouble();
  }

  @Override
  public void setShooterVoltage(double voltage) {
    shooterMotor.setVoltage(voltage);
  }

  @Override
  public void setShooterVelocity(double velocity) {
    if (ShooterConstants.kUseMotionMagic) {
      shooterMotor.setControl(motionMagicRequest.withVelocity(velocity));
    } else {
      shooterMotor.setControl(velocityRequest.withVelocity(velocity));
    }
  }

  @Override
  public void setShooterFollowerVoltage(double voltage) {
    shooterMotorFollower.setVoltage(voltage);
  }

  @Override
  public void setShooterFollowerVelocity(double velocity) {
    if (ShooterConstants.kUseMotionMagic) {
      shooterMotorFollower.setControl(motionMagicRequestFollower.withVelocity(velocity));
    } else {
      shooterMotorFollower.setControl(velocityRequestFollower.withVelocity(velocity));
    }
  }

  @Override
  public void off() {
    if (ShooterConstants.kUseShooterRegenBraking) {
      shooterMotor.setControl(regenRequest);
      shooterMotorFollower.setControl(regenRequest);
    } else {
      shooterMotor.setControl(new NeutralOut());
      shooterMotorFollower.setControl(new NeutralOut());
    }
  }

  @Override
  public TalonFX getMotor() {
    return shooterMotor;
  }

  @Override
  public TalonFX getFollowerMotor() {
    return shooterMotorFollower;
  }

  @Override
  public VoltageOut getVoltageRequest() {
    return voltageReq;
  }
}
