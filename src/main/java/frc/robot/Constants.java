// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Distance;

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
      public static final double kStickDeadband = 0.05;
      public static final double kRotationalDeadband = 0.1;
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
    // We don't want AdvantageKit (which logs EVERYTHING)
    // to dump to NetworkTables during competition.
    // So we use Monologue to log the things we deem necessary
    // for our dashboard (it will ALWAYS log to NetworkTables;
    // AdvantageKit will be configured to log to a file when connected to FMS)
    public static final boolean kEpilogueEnabled = true;
    // If true, the LoggedTunableNumber will work and do TunableNumber things
    public static final boolean kTuningModeEnabled = false;

    public static final boolean kPhotonEnabled = false;
  }

  public static class Logging {
    public static final boolean kLogToUSB = false;
    public static final boolean kAdvkitUseReplayLogs = false;
    // DEBUG: Everything (which will be useful for debugging)
    // INFO: Everything except DEBUG
    public static final Logged.Importance kEpilogueImportance = Logged.Importance.DEBUG;
  }

  public static class SimulationConstants {
    public static final Distance kDrivebaseWidth = Inches.of(27);
  }

  public enum RobotType {
    COMPETITION("TODO: TUne"),
    PRACTICE("TODO: Tune"),
    SIMULATION("no label");

    public final String serialNumber;

    RobotType(String serialNumber) {
      this.serialNumber = serialNumber;
    }
  }

  public static final FieldConstants.BranchHeights branchHeights =
      FieldConstants.BranchHeights.PRACTICE_FIELD;

  /*
   * Robot Constants.
   */

  public static class RobotConstants {
    public static final double bumperLength = 0.1; // should be right?
    public static final double bumperWidth = 22;
  }
}