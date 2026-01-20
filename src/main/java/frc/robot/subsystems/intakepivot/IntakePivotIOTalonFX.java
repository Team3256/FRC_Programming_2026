package frc.robot.subsystems.intakepivot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utils.PhoenixUtil;

public class IntakePivotIOTalonFX implements IntakePivotIO {

  private final TalonFX pivotMotor =
      new TalonFX(IntakePivotConstants.pivotMotorId);

  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0)
      .withSlot(0)
      .withEnableFOC(IntakePivotConstants.kUseFOC);

  private final StatusSignal<Voltage> motorVoltage = pivotMotor.getMotorVoltage();
  private final StatusSignal<AngularVelocity> velocity = pivotMotor.getVelocity();
  private final StatusSignal<Angle> position = pivotMotor.getPosition();
  private final StatusSignal<Current> statorCurrent = pivotMotor.getStatorCurrent();
  private final StatusSignal<Current> supplyCurrent = pivotMotor.getSupplyCurrent();

  public IntakePivotIOTalonFX() {
    PhoenixUtil.applyMotorConfigs(
        pivotMotor,
        IntakePivotConstants.motorConfigs,
        IntakePivotConstants.flashConfigRetries);

    BaseStatusSignal.setUpdateFrequencyForAll(
        IntakePivotConstants.updateFrequency,
        motorVoltage,
        velocity,
        position,
        statorCurrent,
        supplyCurrent);

    PhoenixUtil.registerSignals(
        true,
        motorVoltage,
        velocity,
        position,
        statorCurrent,
        supplyCurrent);
  }

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {

    inputs.pivotMotorVoltage = motorVoltage.getValue().in(Volts);
    inputs.pivotMotorVelocity = velocity.getValue().in(RotationsPerSecond);
    
    inputs.pivotMotorPosition = position.getValue().in(Rotations);

    inputs.pivotMotorStatorCurrent = statorCurrent.getValue().in(Amps);
    inputs.pivotMotorSupplyCurrent = supplyCurrent.getValue().in(Amps);
  }

  @Override
  public void setPosition(double target) {
    if (IntakePivotConstants.kUseMotionMagic) {
      pivotMotor.setControl(motionMagicRequest.withPosition(target));
    }
  }

  @Override
  public TalonFX getMotor() {
    return pivotMotor;
  }

  @Override
  public void setVoltage(double volts) {
    pivotMotor.setVoltage(volts);
  }

  @Override
  public void stop() {
    pivotMotor.setControl(new NeutralOut());
  }

  @Override
  public void resetPosition(double angle) {
    pivotMotor.setPosition(angle);
  }
}