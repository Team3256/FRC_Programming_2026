// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.utils;

import com.ctre.phoenix6.configs.*;

public class PhoenixConfigEquality {
  // Stolen from 254, as it's a dependency of PhoenixUtil.
  // Should probably make this configured like how we normally configure code

  public static final boolean ENABLE_LOGGING_INEQ = true;
  public static final double TALON_CONFIG_EPSILON = 0.05;
  public static final double CANCODER_CONFIG_EPSILON = 0.05;

  public static final double CANDI_CONFIG_EPSILON = 0.05;
  public static final double CANRANGE_CONFIG_EPSILON = .05;
  private static final double kEpsilon = 1e-12;

  private static boolean epsilonEquals(double a, double b, double epsilon) {
    return (a - epsilon <= b) && (a + epsilon >= b);
  }

  private static boolean epsilonEquals(double a, double b) {
    return epsilonEquals(a, b, kEpsilon);
  }

  // private static boolean epsilonEquals(int a, int b, int epsilon) {
  // return (a - epsilon <= b) && (a + epsilon >= b);
  // }

  public static boolean isEqual(TalonFXConfiguration a, TalonFXConfiguration b) {
    return isEqual(a.Slot0, b.Slot0)
            && isEqual(a.Slot1, b.Slot1)
            && isEqual(a.Slot2, b.Slot2)
            && isEqual(a.MotorOutput, b.MotorOutput)
            && isEqual(a.CurrentLimits, b.CurrentLimits)
            && isEqual(a.Voltage, b.Voltage)
            && isEqual(a.TorqueCurrent, b.TorqueCurrent)
            && isEqual(a.Feedback, b.Feedback)
            && isEqual(a.OpenLoopRamps, b.OpenLoopRamps)
            && isEqual(a.ClosedLoopRamps, b.ClosedLoopRamps)
            && isEqual(a.HardwareLimitSwitch, b.HardwareLimitSwitch)
            && isEqual(a.Audio, b.Audio)
            && isEqual(a.SoftwareLimitSwitch, b.SoftwareLimitSwitch)
            && isEqual(a.MotionMagic, b.MotionMagic);
  }

  public static boolean isEqual(CANrangeConfiguration a, CANrangeConfiguration b) {
    return isEqual(a.FovParams, b.FovParams)
            && a.FutureProofConfigs == b.FutureProofConfigs
            && isEqual(a.ProximityParams, b.ProximityParams)
            && isEqual(a.ToFParams, b.ToFParams);
  }

  public static boolean isEqual(CANdiConfiguration a, CANdiConfiguration b) {
    return isEqual(a.DigitalInputs, b.DigitalInputs)
            && isEqual(a.PWM1, b.PWM1)
            && isEqual(a.PWM2, b.PWM2)
            && isEqual(a.Quadrature, b.Quadrature);
  }

  public static boolean isEqual(FovParamsConfigs a, FovParamsConfigs b) { // done
    return epsilonEquals(a.FOVCenterX, b.FOVCenterX, CANRANGE_CONFIG_EPSILON)
            && epsilonEquals(a.FOVCenterY, b.FOVCenterY, CANRANGE_CONFIG_EPSILON)
            && epsilonEquals(a.FOVRangeX, b.FOVRangeX, CANRANGE_CONFIG_EPSILON)
            && epsilonEquals(a.FOVRangeY, b.FOVRangeY, CANRANGE_CONFIG_EPSILON);
  }

  public static boolean isEqual(PWM1Configs a, PWM1Configs b) {
    return a.SensorDirection == b.SensorDirection
            && epsilonEquals(a.AbsoluteSensorOffset, b.AbsoluteSensorOffset, CANDI_CONFIG_EPSILON)
            && epsilonEquals(
            a.AbsoluteSensorDiscontinuityPoint,
            b.AbsoluteSensorDiscontinuityPoint,
            CANDI_CONFIG_EPSILON);
  }

  public static boolean isEqual(ProximityParamsConfigs a, ProximityParamsConfigs b) {
    return epsilonEquals(
            a.MinSignalStrengthForValidMeasurement,
            b.MinSignalStrengthForValidMeasurement,
            CANRANGE_CONFIG_EPSILON)
            && epsilonEquals(a.ProximityHysteresis, b.ProximityHysteresis, CANRANGE_CONFIG_EPSILON)
            && epsilonEquals(
            a.ProximityThreshold, b.ProximityThreshold, CANRANGE_CONFIG_EPSILON); // all epsilon
  }

  public static boolean isEqual(ToFParamsConfigs a, ToFParamsConfigs b) {
    return a.UpdateMode == b.UpdateMode
            && epsilonEquals(a.UpdateFrequency, b.UpdateFrequency, CANRANGE_CONFIG_EPSILON); // done
  }

  public static boolean isEqual(PWM2Configs a, PWM2Configs b) {
    return a.SensorDirection == b.SensorDirection
            && epsilonEquals(a.AbsoluteSensorOffset, b.AbsoluteSensorOffset, CANDI_CONFIG_EPSILON)
            && epsilonEquals(
            a.AbsoluteSensorDiscontinuityPoint,
            b.AbsoluteSensorDiscontinuityPoint,
            CANDI_CONFIG_EPSILON);
  }

  public static boolean isEqual(QuadratureConfigs a, QuadratureConfigs b) {
    return a.SensorDirection == b.SensorDirection
            && a.QuadratureEdgesPerRotation == b.QuadratureEdgesPerRotation;
  }

  public static boolean isEqual(DigitalInputsConfigs a, DigitalInputsConfigs b) {
    return a.S1CloseState == b.S1CloseState && a.S2CloseState == b.S2CloseState;
  }

  public static boolean isEqual(CANcoderConfiguration a, CANcoderConfiguration b) {
    return a.FutureProofConfigs == b.FutureProofConfigs && isEqual(a.MagnetSensor, b.MagnetSensor);
  }

  public static boolean isEqual(MagnetSensorConfigs a, MagnetSensorConfigs b) {
    return a.SensorDirection == b.SensorDirection
            && epsilonEquals(a.MagnetOffset, b.MagnetOffset, CANCODER_CONFIG_EPSILON)
            && epsilonEquals(
            a.AbsoluteSensorDiscontinuityPoint,
            b.AbsoluteSensorDiscontinuityPoint,
            CANCODER_CONFIG_EPSILON);
  }

  public static boolean isEqual(Slot0Configs a, Slot0Configs b) {
    boolean val =
            epsilonEquals(a.kP, b.kP, TALON_CONFIG_EPSILON)
                    && epsilonEquals(a.kI, b.kI, TALON_CONFIG_EPSILON)
                    && epsilonEquals(a.kD, b.kD, TALON_CONFIG_EPSILON)
                    && epsilonEquals(a.kV, b.kV, TALON_CONFIG_EPSILON)
                    && epsilonEquals(a.kS, b.kS, TALON_CONFIG_EPSILON);
    if (ENABLE_LOGGING_INEQ && !val) {
      System.out.println("Slot0Configs not equal");
    }
    return val;
  }

  public static boolean isEqual(Slot1Configs a, Slot1Configs b) {
    boolean val =
            epsilonEquals(a.kP, b.kP, TALON_CONFIG_EPSILON)
                    && epsilonEquals(a.kI, b.kI, TALON_CONFIG_EPSILON)
                    && epsilonEquals(a.kD, b.kD, TALON_CONFIG_EPSILON)
                    && epsilonEquals(a.kV, b.kV, TALON_CONFIG_EPSILON)
                    && epsilonEquals(a.kS, b.kS, TALON_CONFIG_EPSILON);
    if (ENABLE_LOGGING_INEQ && !val) {
      System.out.println("Slot1Configs not equal");
    }
    return val;
  }

  public static boolean isEqual(Slot2Configs a, Slot2Configs b) {
    boolean val =
            epsilonEquals(a.kP, b.kP, TALON_CONFIG_EPSILON)
                    && epsilonEquals(a.kI, b.kI, TALON_CONFIG_EPSILON)
                    && epsilonEquals(a.kD, b.kD, TALON_CONFIG_EPSILON)
                    && epsilonEquals(a.kV, b.kV, TALON_CONFIG_EPSILON)
                    && epsilonEquals(a.kS, b.kS, TALON_CONFIG_EPSILON);
    if (ENABLE_LOGGING_INEQ && !val) {
      System.out.println("Slot2Configs not equal");
    }
    return val;
  }

  public static boolean isEqual(MotorOutputConfigs a, MotorOutputConfigs b) {
    boolean val =
            a.Inverted.value == b.Inverted.value
                    && a.NeutralMode.value == b.NeutralMode.value
                    && epsilonEquals(
                    a.DutyCycleNeutralDeadband, b.DutyCycleNeutralDeadband, TALON_CONFIG_EPSILON)
                    && epsilonEquals(a.PeakForwardDutyCycle, b.PeakForwardDutyCycle, TALON_CONFIG_EPSILON)
                    && epsilonEquals(a.PeakReverseDutyCycle, b.PeakReverseDutyCycle, TALON_CONFIG_EPSILON);
    if (ENABLE_LOGGING_INEQ && !val) {
      System.out.println("MotorOutputConfigs not equal");
    }
    return val;
  }

  public static boolean isEqual(CurrentLimitsConfigs a, CurrentLimitsConfigs b) {
    boolean val =
            epsilonEquals(a.StatorCurrentLimit, b.StatorCurrentLimit, TALON_CONFIG_EPSILON)
                    && epsilonEquals(a.SupplyCurrentLimit, b.SupplyCurrentLimit, TALON_CONFIG_EPSILON)
                    && a.StatorCurrentLimitEnable == b.StatorCurrentLimitEnable
                    && a.SupplyCurrentLimitEnable == b.SupplyCurrentLimitEnable;
    if (ENABLE_LOGGING_INEQ && !val) {
      System.out.println("CurrentLimitsConfigs not equal");
    }
    return val;
  }

  public static boolean isEqual(VoltageConfigs a, VoltageConfigs b) {
    boolean val =
            epsilonEquals(
                    a.SupplyVoltageTimeConstant, b.SupplyVoltageTimeConstant, TALON_CONFIG_EPSILON)
                    && epsilonEquals(a.PeakForwardVoltage, b.PeakForwardVoltage)
                    && epsilonEquals(a.PeakReverseVoltage, b.PeakReverseVoltage);
    if (ENABLE_LOGGING_INEQ && !val) {
      System.out.println("VoltageConfigs not equal");
    }
    return val;
  }

  public static boolean isEqual(TorqueCurrentConfigs a, TorqueCurrentConfigs b) {
    boolean val =
            epsilonEquals(a.PeakForwardTorqueCurrent, b.PeakForwardTorqueCurrent, TALON_CONFIG_EPSILON)
                    && epsilonEquals(
                    a.PeakReverseTorqueCurrent, b.PeakReverseTorqueCurrent, TALON_CONFIG_EPSILON)
                    && epsilonEquals(
                    a.TorqueNeutralDeadband, b.TorqueNeutralDeadband, TALON_CONFIG_EPSILON);
    if (ENABLE_LOGGING_INEQ && !val) {
      System.out.println("TorqueCurrentConfigs not equal");
    }
    return val;
  }

  public static boolean isEqual(FeedbackConfigs a, FeedbackConfigs b) {
    boolean val =
            epsilonEquals(a.FeedbackRotorOffset, b.FeedbackRotorOffset, TALON_CONFIG_EPSILON)
                    && epsilonEquals(
                    a.SensorToMechanismRatio, b.SensorToMechanismRatio, TALON_CONFIG_EPSILON)
                    && epsilonEquals(a.RotorToSensorRatio, b.RotorToSensorRatio, TALON_CONFIG_EPSILON)
                    && a.FeedbackSensorSource.value == b.FeedbackSensorSource.value
                    && a.FeedbackRemoteSensorID == b.FeedbackRemoteSensorID;
    if (ENABLE_LOGGING_INEQ && !val) {
      System.out.println("FeedbackConfigs not equal");
      System.out.printf(
              "A-FeedbackRotorOffset: %f, B-FeedbackRotorOffset: %f\n",
              a.FeedbackRotorOffset, b.FeedbackRotorOffset);
      System.out.printf(
              "A-SensorToMechanismRatio: %f, B-SensorToMechanismRatio: %f\n",
              a.SensorToMechanismRatio, b.SensorToMechanismRatio);
      System.out.printf(
              "A-RotorToSensorRatio: %f, B-RotorToSensorRatio: %f\n",
              a.RotorToSensorRatio, b.RotorToSensorRatio);
      System.out.printf(
              "A-FeedbackSensorSource: %d, B-FeedbackSensorSource: %d\n",
              a.FeedbackSensorSource.value, b.FeedbackSensorSource.value);
      System.out.printf(
              "A-FeedbackRemoteSensorID: %d, B-FeedbackRemoteSensorID: %d\n",
              a.FeedbackRemoteSensorID, b.FeedbackRemoteSensorID);
    }
    return val;
  }

  public static boolean isEqual(OpenLoopRampsConfigs a, OpenLoopRampsConfigs b) {
    boolean val =
            epsilonEquals(
                    a.DutyCycleOpenLoopRampPeriod, b.DutyCycleOpenLoopRampPeriod, TALON_CONFIG_EPSILON)
                    && epsilonEquals(
                    a.VoltageOpenLoopRampPeriod, b.VoltageOpenLoopRampPeriod, TALON_CONFIG_EPSILON)
                    && epsilonEquals(
                    a.TorqueOpenLoopRampPeriod, b.TorqueOpenLoopRampPeriod, TALON_CONFIG_EPSILON);
    if (ENABLE_LOGGING_INEQ && !val) {
      System.out.println("OpenLoopRampsConfigs not equal");
    }
    return val;
  }

  public static boolean isEqual(ClosedLoopRampsConfigs a, ClosedLoopRampsConfigs b) {
    boolean val =
            epsilonEquals(
                    a.DutyCycleClosedLoopRampPeriod,
                    b.DutyCycleClosedLoopRampPeriod,
                    TALON_CONFIG_EPSILON)
                    && epsilonEquals(
                    a.VoltageClosedLoopRampPeriod, b.VoltageClosedLoopRampPeriod, TALON_CONFIG_EPSILON)
                    && epsilonEquals(
                    a.TorqueClosedLoopRampPeriod, b.TorqueClosedLoopRampPeriod, TALON_CONFIG_EPSILON);
    if (ENABLE_LOGGING_INEQ && !val) {
      System.out.println("ClosedLoopRampsConfigs not equal");
    }
    return val;
  }

  public static boolean isEqual(HardwareLimitSwitchConfigs a, HardwareLimitSwitchConfigs b) {
    boolean val =
            a.ForwardLimitAutosetPositionEnable == b.ForwardLimitAutosetPositionEnable
                    && a.ForwardLimitEnable == b.ForwardLimitEnable
                    && a.ReverseLimitAutosetPositionEnable == b.ReverseLimitAutosetPositionEnable
                    && a.ReverseLimitEnable == b.ReverseLimitEnable
                    && epsilonEquals(
                    a.ForwardLimitAutosetPositionValue,
                    b.ForwardLimitAutosetPositionValue,
                    TALON_CONFIG_EPSILON)
                    && epsilonEquals(
                    a.ReverseLimitAutosetPositionValue,
                    b.ReverseLimitAutosetPositionValue,
                    TALON_CONFIG_EPSILON)
                    && a.ForwardLimitRemoteSensorID == b.ForwardLimitRemoteSensorID
                    && a.ReverseLimitRemoteSensorID == b.ReverseLimitRemoteSensorID
                    && a.ForwardLimitSource.value == b.ForwardLimitSource.value
                    && a.ReverseLimitSource.value == b.ReverseLimitSource.value
                    && a.ForwardLimitType.value == b.ForwardLimitType.value
                    && a.ReverseLimitType.value == b.ReverseLimitType.value;
    if (ENABLE_LOGGING_INEQ && !val) {
      System.out.println("HardwareLimitSwitchConfigs not equal");
    }
    return val;
  }

  public static boolean isEqual(AudioConfigs a, AudioConfigs b) {
    boolean val = a.BeepOnBoot == b.BeepOnBoot;
    if (ENABLE_LOGGING_INEQ && !val) {
      System.out.println("AudioConfigs not equal");
    }
    return val;
  }

  public static boolean isEqual(SoftwareLimitSwitchConfigs a, SoftwareLimitSwitchConfigs b) {
    boolean val =
            epsilonEquals(
                    a.ForwardSoftLimitThreshold, b.ForwardSoftLimitThreshold, TALON_CONFIG_EPSILON)
                    && epsilonEquals(
                    a.ReverseSoftLimitThreshold, b.ReverseSoftLimitThreshold, TALON_CONFIG_EPSILON)
                    && a.ReverseSoftLimitEnable == b.ReverseSoftLimitEnable
                    && a.ForwardSoftLimitEnable == b.ForwardSoftLimitEnable;
    if (ENABLE_LOGGING_INEQ && !val) {
      System.out.println("SoftwareLimitSwitchConfigs not equal");
    }
    return val;
  }

  public static boolean isEqual(MotionMagicConfigs a, MotionMagicConfigs b) {
    boolean val =
            epsilonEquals(a.MotionMagicAcceleration, b.MotionMagicAcceleration, TALON_CONFIG_EPSILON)
                    && epsilonEquals(
                    a.MotionMagicCruiseVelocity, b.MotionMagicCruiseVelocity, TALON_CONFIG_EPSILON)
                    && epsilonEquals(a.MotionMagicJerk, b.MotionMagicJerk, TALON_CONFIG_EPSILON);
    if (ENABLE_LOGGING_INEQ && !val) {
      System.out.println("MotionMagicConfigs not equal");
    }
    return val;
  }
}