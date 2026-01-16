// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.utils;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.function.Supplier;

// The only method you should really use is applyMotorConfigs
public class PhoenixUtil {
  // Spams fetching StatusCode until it works or we run out of attempts
  public static boolean spamGetStatusCode(Supplier<StatusCode> function, int numTries) {
    StatusCode code = function.get();
    int tries = 0;
    while (code != StatusCode.OK && tries < numTries) {
      DriverStation.reportWarning("Retrying CTRE Device Config " + code.getName(), false);
      code = function.get();
      tries++;
    }
    if (code != StatusCode.OK) {
      DriverStation.reportError(
              "Failed to execute phoenix pro api call after " + numTries + " attempts", false);
      return false;
    }
    return true;
  }

  public static boolean spamGetStatusCode(Supplier<StatusCode> function) {
    // Default that 254 uses
    return spamGetStatusCode(function, 5);
  }

  public static boolean readAndVerifyConfiguration(TalonFX talon, TalonFXConfiguration config) {
    TalonFXConfiguration readConfig = new TalonFXConfiguration();
    if (!spamGetStatusCode(() -> talon.getConfigurator().refresh(readConfig))) {
      // could not get config!
      DriverStation.reportWarning(
              "Failed to read config for talon [" + talon.getDescription() + "]", false);
      return false;
    } else if (!PhoenixConfigEquality.isEqual(config, readConfig)) {
      // configs did not match
      DriverStation.reportWarning(
              "Configuration verification failed for talon [" + talon.getDescription() + "]", false);
      return false;
    } else {
      // configs read and match, Talon OK
      return true;
    }
  }

  public static boolean readAndVerifyConfiguration(
          CANcoder cancoder, CANcoderConfiguration config) {
    CANcoderConfiguration readConfig = new CANcoderConfiguration();
    if (!spamGetStatusCode(() -> cancoder.getConfigurator().refresh(readConfig))) {
      // could not get config!
      DriverStation.reportWarning(
              "Failed to read config for CANCoder [" + cancoder.getDeviceID() + "]", false);
      return false;
    } else if (!PhoenixConfigEquality.isEqual(config, readConfig)) {
      // configs did not match
      DriverStation.reportWarning(
              "Configuration verification failed for cancoder [" + cancoder.getDeviceID() + "]", false);
      return false;
    } else {
      // configs read and match, Talon OK
      return true;
    }
  }

  public static boolean readAndVerifyConfiguration(CANdi canDi, CANdiConfiguration config) {
    CANdiConfiguration readConfig = new CANdiConfiguration();
    if (!spamGetStatusCode(() -> canDi.getConfigurator().refresh(readConfig))) {
      // could not get config!
      DriverStation.reportWarning(
              "Failed to read config for CANdi [" + canDi.getDeviceID() + "]", false);
      return false;
    } else if (!PhoenixConfigEquality.isEqual(config, readConfig)) {
      // configs did not match
      DriverStation.reportWarning(
              "Configuration verification failed for CANdi [" + canDi.getDeviceID() + "]", false);
      return false;
    } else {
      // configs read and match, Talon OK
      return true;
    }
  }

  public static boolean readAndVerifyConfiguration(
          CANrange CANrange, CANrangeConfiguration canRangeConfig) {
    CANrangeConfiguration readConfig = new CANrangeConfiguration();
    if (!spamGetStatusCode(() -> CANrange.getConfigurator().refresh(readConfig))) {
      // could not get config!
      DriverStation.reportWarning(
              "Failed to read config for talon [" + CANrange.getDeviceID() + "]", false);
      return false;
    } else if (!PhoenixConfigEquality.isEqual(canRangeConfig, readConfig)) {
      // configs did not match
      DriverStation.reportWarning(
              "Configuration verification failed for talon [" + CANrange.getDeviceID() + "]", false);
      return false;
    } else {
      // configs read and match, Talon OK
      return true;
    }
  }

  // The main function you should use for most purposes
  public static boolean applyMotorConfigs(
          TalonFX motor, TalonFXConfiguration motorConfig, int numTries) {
    for (int i = 0; i < numTries; i++) {
      if (RobotBase.isSimulation()) {
        return motor.getConfigurator().apply(motorConfig).isOK();
      }
      if (spamGetStatusCode(() -> motor.getConfigurator().apply(motorConfig))) {
        // API says we applied config, lets make sure it's right
        if (readAndVerifyConfiguration(motor, motorConfig)) {
          return true;
        } else {
          DriverStation.reportWarning(
                  "Failed to verify config for talon ["
                          + motor.getDescription()
                          + "] (attempt "
                          + (i + 1)
                          + " of "
                          + numTries
                          + ")",
                  false);
        }
      } else {
        DriverStation.reportWarning(
                "Failed to apply config for talon ["
                        + motor.getDescription()
                        + "] (attempt "
                        + (i + 1)
                        + " of "
                        + numTries
                        + ")",
                false);
      }
    }
    DriverStation.reportError(
            "Failed to apply config for talon after " + numTries + " attempts", false);
    return false;
  }

  public static boolean applyCancoderConfig(
          CANcoder cancoder, CANcoderConfiguration cancoderConfig, int numTries) {
    for (int i = 0; i < numTries; i++) {
      if (RobotBase.isSimulation()) {
        return cancoder.getConfigurator().apply(cancoderConfig).isOK();
      }
      if (spamGetStatusCode(() -> cancoder.getConfigurator().apply(cancoderConfig))) {
        // API says we applied config, lets make sure it's right
        if (readAndVerifyConfiguration(cancoder, cancoderConfig)) {
          return true;
        } else {
          DriverStation.reportWarning(
                  "Failed to verify config for cancoder ["
                          + cancoder.getDeviceID()
                          + "] (attempt "
                          + (i + 1)
                          + " of "
                          + numTries
                          + ")",
                  false);
        }
      } else {
        DriverStation.reportWarning(
                "Failed to apply config for cancoder ["
                        + cancoder.getDeviceID()
                        + "] (attempt "
                        + (i + 1)
                        + " of "
                        + numTries
                        + ")",
                false);
      }
    }
    DriverStation.reportError(
            "Failed to apply config for cancoder after " + numTries + " attempts", false);
    return false;
  }

  public static boolean applyCANdiConfigs(
          CANdi candi, CANdiConfiguration candiConfig, int numTries) {
    for (int i = 0; i < numTries; i++) {
      if (RobotBase.isSimulation()) {
        return candi.getConfigurator().apply(candiConfig).isOK();
      }
      if (spamGetStatusCode(() -> candi.getConfigurator().apply(candiConfig))) {
        // API says we applied config, lets make sure it's right
        if (readAndVerifyConfiguration(candi, candiConfig)) {
          return true;
        } else {
          DriverStation.reportWarning(
                  "Failed to verify config for candi ["
                          + candi.getDeviceID()
                          + "] (attempt "
                          + (i + 1)
                          + " of "
                          + numTries
                          + ")",
                  false);
        }
      } else {
        DriverStation.reportWarning(
                "Failed to apply config for candi ["
                        + candi.getDeviceID()
                        + "] (attempt "
                        + (i + 1)
                        + " of "
                        + numTries
                        + ")",
                false);
      }
    }
    DriverStation.reportError(
            "Failed to apply config for candi after " + numTries + " attempts", false);
    return false;
  }

  public static boolean applyCanRangeConfigs(
          CANrange CANrange, CANrangeConfiguration canRangeConfig, int numTries) {
    for (int i = 0; i < numTries; i++) {
      if (RobotBase.isSimulation()) {
        return CANrange.getConfigurator().apply(canRangeConfig).isOK();
      }
      if (spamGetStatusCode(() -> CANrange.getConfigurator().apply(canRangeConfig))) {
        // API says we applied config, lets make sure it's right
        if (readAndVerifyConfiguration(CANrange, canRangeConfig)) {
          return true;
        } else {
          DriverStation.reportWarning(
                  "Failed to verify config for candi ["
                          + CANrange.getDeviceID()
                          + "] (attempt "
                          + (i + 1)
                          + " of "
                          + numTries
                          + ")",
                  false);
        }
      } else {
        DriverStation.reportWarning(
                "Failed to apply config for candi ["
                        + CANrange.getDeviceID()
                        + "] (attempt "
                        + (i + 1)
                        + " of "
                        + numTries
                        + ")",
                false);
      }
    }
    DriverStation.reportError(
            "Failed to apply config for candi after " + numTries + " attempts", false);
    return false;
  }

  private static BaseStatusSignal[] canivoreSignals = new BaseStatusSignal[0];

  private static BaseStatusSignal[] rioSignals = new BaseStatusSignal[0];

  /** Registers a set of signals for synchronized refresh. */
  public static void registerSignals(boolean canivore, BaseStatusSignal... signals) {
    if (canivore) {
      BaseStatusSignal[] newSignals = new BaseStatusSignal[canivoreSignals.length + signals.length];
      System.arraycopy(canivoreSignals, 0, newSignals, 0, canivoreSignals.length);
      System.arraycopy(signals, 0, newSignals, canivoreSignals.length, signals.length);
      canivoreSignals = newSignals;
    } else {
      BaseStatusSignal[] newSignals = new BaseStatusSignal[rioSignals.length + signals.length];
      System.arraycopy(rioSignals, 0, newSignals, 0, rioSignals.length);
      System.arraycopy(signals, 0, newSignals, rioSignals.length, signals.length);
      rioSignals = newSignals;
    }
  }

  public static void refreshAll() {
    if (canivoreSignals.length > 0) {
      BaseStatusSignal.refreshAll(canivoreSignals);
    }
    if (rioSignals.length > 0) {
      BaseStatusSignal.refreshAll(rioSignals);
    }
  }
}