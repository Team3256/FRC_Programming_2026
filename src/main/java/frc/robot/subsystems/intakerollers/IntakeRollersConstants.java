

// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intakerollers;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.UpdateModeValue;

public class IntakeRollersConstants {

  public static final boolean kUseFOC = false;

  // Constants used in CANrange construction
  public static final int kCANrangeId = 1;

  // Configure the CANrange for basic use
  public static final CANrangeConfiguration canRangeConfigs =
      new CANrangeConfiguration()
          .withFovParams(
              new FovParamsConfigs()
                  .withFOVRangeX(7)
                  .withFOVRangeY(7)
                  .withFOVCenterY(.7)
                  .withFOVCenterX(.7))
          .withToFParams(
              new ToFParamsConfigs()
                  .withUpdateMode(UpdateModeValue.ShortRangeUserFreq)
                  .withUpdateFrequency(50));

  public static final int kIntakeRollerMotorID = 45;
  public static final double intakeVoltage = -6;

  public static final double handoffVoltage = 4;
  public static double l1OuttakeVoltage = 8;

  public static double updateFrequency = 50;

  public static final TalonFXConfiguration motorConfigs =
      new TalonFXConfiguration()
          .withSlot0(new Slot0Configs().withKS(0).withKV(0.12).withKP(1).withKI(0).withKD(0))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Brake)
                  .withInverted(InvertedValue.Clockwise_Positive))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(60));
  public static int flashConfigRetries = 5;

  public static final int motorCoralStall = 40;
  public static final double canRangeInThreshold = .4;

  public static final class SimulationConstants {
    public static double rollerGearingRatio = 1.0;
    public static double rollerMomentOfInertia = 1;

    // Scale down the angular velocity so we can actually see what is happening
    public static double kAngularVelocityScalar = 5;
  }
}
