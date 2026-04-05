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

public class IntakeRollersConstants {

  public static final boolean kUseFOC = false;

  public static final int kIntakeRollerMotorIDLeft = 46;
  public static final int kIntakeRollerMotorIDRight = 47;

  public static double updateFrequency = 50;

  public static final TalonFXConfiguration motorConfigs =
      new TalonFXConfiguration()
          .withSlot0(new Slot0Configs().withKS(0).withKV(0.12).withKP(1).withKI(0).withKD(0))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Coast)
                  .withInverted(InvertedValue.CounterClockwise_Positive))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(60)
                      .withSupplyCurrentLimit(50)
                      .withSupplyCurrentLimitEnable(true)
                      .withSupplyCurrentLowerLimit(30)
                      .withSupplyCurrentLowerTime(0.1));
  public static int flashConfigRetries = 5;

  public static final class SimulationConstants {
    public static double rollerGearingRatio = 1.0;
    public static double rollerMomentOfInertia = 1;

    // Scale down the angular velocity so we can actually see what is happening
    public static double kAngularVelocityScalar = 5;
  }
}
