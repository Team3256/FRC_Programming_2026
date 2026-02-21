// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intakepivot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

public final class IntakePivotConstants {
  public static final int pivotMotorId = 37;

  public static final boolean kUseFOC = false;
  public static final boolean kUseMotionMagic = true;
  public static final int flashConfigRetries = 5;
  public static final double stowPosition = .342;

  public static final double jitterPosition = .15;
  public static final double jitterIntermediate = 0.05;
  public static final double groundIntakePosition = 0;

  public static final TalonFXConfiguration motorConfigs =
      new TalonFXConfiguration()
          .withSlot0(
              new Slot0Configs()
                  .withKS(.169)
                  .withKV(.09 * 43.5)
                  .withKP(300)
                  .withKI(0)
                  .withKD(15)
                  .withKA(0)
                  .withKG(.469)
                  .withGravityType(GravityTypeValue.Arm_Cosine))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Brake)
                  .withInverted(InvertedValue.Clockwise_Positive))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicAcceleration(20)
                  .withMotionMagicCruiseVelocity(5))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(80))
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(43.5));

  public static final class PivotSim {
    // not sure about what gearing means and not sure if its equal to gear ratio
    public static final double pivotSimGearing = 40;

    public static final Distance intakePivotLength = Inches.of(24);
    public static final Mass intakePivotMass = Kilograms.of(1);
    public static final double jkGMetersSquared = 1;

    public static final Rotation2d minAngle = Rotation2d.fromDegrees(0);
    public static final Rotation2d maxAngle = Rotation2d.fromDegrees(180);
    public static final Rotation2d startingAngle = Rotation2d.fromDegrees(90);
  }

  public static double updateFrequency = 50;
}
