// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class ControllerConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    public static class DriverConstants {
      public static final double kStickDeadband = 0.1;
      public static final double kRotationalDeadband = 0.12;
    }
  }

  // Naming scheme for FeatureFlags:
  // k___Enabled = enables/disables a subsystem
  // kUse___ = enables/disables a specific feature
  // but all of the kUse constants should
  // be in their specific SubsystemConstants file
  public static class FeatureFlags {
    // AdvantageKit is a logging library we use that
    // can provide logging replay and it's how we can
    // do comprehensive logging. Logging to NetworkTables
    // is disabled on field (FMS connected).
    public static final boolean kAdvKitEnabled = true;
    // Monologue is another logging library that we use for
    // sending values to our dashboard because we don't want
    // AdvantageKit (which logs EVERYTHING) to dump to NetworkTables
    // during competition. So we use Monologue to log the things
    // for our dashboard (it will ALWAYS log to NetworkTables;
    // AdvantageKit can be configured to log to a file when connected to FMS)
    public static final boolean kMonologueEnabled = true;
    // If true, the LoggedTunableNumber will work and do TunableNumber things
    public static final boolean kTuningModeEnabled = false;
    // Toggle whether or not the controller map should dump
    public static final boolean kControllerMapEnabled = true;

    public static final boolean kSwerveEnabled = true;
    public static final boolean kVisionEnabled = false;
    public static final boolean kClimbEnabled = true;
    public static final boolean kIntakeEnabled = true;
    public static final boolean kShooterEnabled = true;
    public static final boolean kPivotShooterEnabled = true;
    public static final boolean kTurretEnabled = true;
    public static final boolean kAmpevatorEnabled = true;
    public static final boolean kAmpevatorRollersEnabled = true;
    public static final boolean kSpindexEnabled = true;
  }

  public static class Logging {
    public static final boolean kLogToUSB = true;
    public static final boolean kAdvkitUseReplayLogs = false;
    // Defaults from Monologue docs
    public static final boolean kMonologueFileOnly = false;
    public static final boolean kMonologueLazyLogging = false;
  }
}