// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class ShooterConstants {
  /* Misc */
  public static final boolean kUseFOC = true;
  public static final boolean kUseShooterRegenBraking = true;
  /* CAN */
  public static int kShooterMotorRightID = 43;
  public static int kShooterMotorLeftID = 48;
  /* PID */
  // Shooter
  public static MotorOutputConfigs motorOutputConfigs =
      new MotorOutputConfigs()
          .withNeutralMode(NeutralModeValue.Brake)
          .withInverted(InvertedValue.CounterClockwise_Positive);
  public static TalonFXConfiguration motorConfigs =
      new TalonFXConfiguration()
          .withSlot0(
              new Slot0Configs()
                  .withKS(0)
                  .withKV(0.15) // Original 0.145
                  // .withKA(1.48)// Original 0 only for feedforward, might not
                  // use
                  .withKP(8)
                  .withKI(0)
                  .withKD(0.1))
          // For regenerative braking
          // we need to make sure that the backcurrent is below the breaker limit
          // P = 2 gives us like 102 amps so that's good enough
          .withSlot1(new Slot1Configs().withKS(0).withKV(0).withKP(2).withKI(0).withKD(0))
          .withMotorOutput(motorOutputConfigs)
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicAcceleration(1600)
                  .withMotionMagicCruiseVelocity(0))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(60))
          .withTorqueCurrent(
              new TorqueCurrentConfigs()
                  .withPeakForwardTorqueCurrent(80)
                  .withPeakReverseTorqueCurrent(80));
  public static TalonFXConfiguration followerMotorConfigs =
      motorConfigs.withMotorOutput(
          motorOutputConfigs.withInverted(InvertedValue.Clockwise_Positive));

  public static double kShooterSpeakerRPS =
      -40; // i got tired of trying to reverse the shooter wheels
  public static double kShooterFollowerSpeakerRPS = 40; // really 80

  public static double kShooterFeederRPS = -35;
  public static double kShooterFollowerFeederRPS = 35;

  public static final class SimulationConstants {
    public static double kLeftGearingRatio = 1.0; // TODO: Update this value
    public static double kLeftMomentOfInertia = 0.0001; // TODO: Update this value
    public static double kRightGearingRatio = 1.0; // TODO: Update this value
    public static double kRightMomentOfInertia = 0.0001; // TODO: Update this value
    // Scale down the angular velocity so we can actually see what is happening
    public static double kAngularVelocityScalar = 0.05;
  }

  /* Misc */
  // before: 1800/6
  public static double updateFrequency = 50.0;
  public static boolean kUseMotionMagic = false;

  public static int flashConfigRetries = 5;
}
