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
  public static final double stowPosition = .3354;

  public static final double jitterPosition = .15;
  public static final double jitterIntermediate = 0.05;
  public static final double groundIntakePosition = 0;

  public static final TalonFXConfiguration motorConfigs =
      new TalonFXConfiguration()
          .withSlot0(
              new Slot0Configs()
                  .withKS(.169)
                  .withKV(.09 * 85.46)
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
                  .withStatorCurrentLimit(40)
                  .withSupplyCurrentLimit(50)
                  .withSupplyCurrentLimitEnable(true)
                  .withSupplyCurrentLowerTime(.1)
                  .withSupplyCurrentLowerLimit(20))
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(85.46));

  public static final class PivotSim {
    /**
     * Motor rotations per arm rotation. Must equal the SensorToMechanismRatio, or the sim arm ends
     * up somewhere other than where the code sent it.
     */
    public static final double pivotSimGearing = 85.46;

    /** Pivot axle to the front roller, from the CAD. */
    public static final Distance intakePivotLength = Millimeters.of(340);

    public static final Mass intakePivotMass = Kilograms.of(4);
    public static final double jkGMetersSquared = 0.4;

    /** Angle 0 is the intake on the ground (arm level), as is mechanism position 0. */
    public static final Rotation2d minAngle = Rotation2d.fromDegrees(0);

    public static final Rotation2d maxAngle = Rotation2d.fromRotations(stowPosition + 0.01);
    public static final Rotation2d startingAngle = Rotation2d.fromRotations(stowPosition);
  }

  public static double updateFrequency = 50;
}
