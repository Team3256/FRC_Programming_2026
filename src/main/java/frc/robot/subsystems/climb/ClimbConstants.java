// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;

public class ClimbConstants {
  public static final int kClimbMotorID = 49;

  public static final double kUpdateFrequency = 0;
  public static final boolean kUseMotionMagic = false;

  public static final double kUpVoltage = 6;
  public static final double kDownVoltage = -6;

  public static final boolean kUseFOC = false;

  public static final TalonFXConfiguration motorConfigs =
      new TalonFXConfiguration()
          .withSlot0(new Slot0Configs().withKS(0).withKV(0).withKP(0.5).withKI(0).withKD(0))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Brake)
                  .withInverted(InvertedValue.Clockwise_Positive))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicAcceleration(4) // TODO tune
                  .withMotionMagicCruiseVelocity(1)) // TODO tune
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(80) // TODO tune
                  .withSupplyCurrentLimitEnable(false))
          .withFeedback(
              new FeedbackConfigs()
                  .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                  .withSensorToMechanismRatio(400) // this is actually the ratio lol
                  .withRotorToSensorRatio(1));

  // Presets (rotations) // make this angle at some point
  public final double stowPosition = -95; // decreased to allow pullback
  public final double pullPosition = -100; // pulls in
  public final double readyPosition = -28; // move out to hook

  public static final class sim {
    public static final double simGearing = 10;
    public static final Distance climbLength = Inches.of(3);
    public static final double jkGMetersSquared = 1.2922967095;

    public static final Rotation2d minAngle = Rotation2d.fromDegrees(0);
    public static final Rotation2d maxAngle = Rotation2d.fromDegrees(360);
    public static final Rotation2d startingAngle = Rotation2d.fromDegrees(90);
  }
}