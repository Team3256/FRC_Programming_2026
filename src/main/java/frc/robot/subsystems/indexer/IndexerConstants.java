// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.indexer;

// TODO: FILL

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IndexerConstants {

  public static final boolean kUseFOC = false;

  public static final int kIndexerMotor1ID = 49;
  public static final int kIndexerMotor2ID = 50;

  public static final double indexVolt = 9;

  public static double updateFrequency = 50;

  public static final double autoUnjamCurrentThreshold = 50.0; // amps
  public static final double autoUnjamCurrentDuration =
      1.0; // seconds over threshold before unjam triggers
  public static final double autoUnjamVoltage = -4.0; // volts reversed during unjam
  public static final double autoUnjamDuration = 0.5; // seconds to run in reverse

  public static final TalonFXConfiguration motorConfigs =
      new TalonFXConfiguration()
          .withSlot0(new Slot0Configs().withKS(0.37).withKV(0.1).withKP(0.05).withKI(0).withKD(0))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Brake)
                  .withInverted(InvertedValue.Clockwise_Positive))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(60)
                  .withSupplyCurrentLimit(50)
                  .withSupplyCurrentLimitEnable(true)
                  .withSupplyCurrentLowerLimit(35)
                  .withSupplyCurrentLowerTime(0.1));

  public static int flashConfigRetries = 5;

  public static final class SimulationConstants {
    public static double rollerGearingRatio = 0.0;
    public static double rollerMomentOfInertia = 0.0;

    // Scale down the angular velocity so we can actually see what is happening
    public static double kAngularVelocityScalar = 0.0;
  }
}
