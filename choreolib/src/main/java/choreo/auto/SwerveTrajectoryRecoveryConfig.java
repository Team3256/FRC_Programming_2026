// Copyright (c) Choreo contributors

package choreo.auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;

/**
 * Configuration for pausing a swerve trajectory's progress when tracking error becomes too large.
 *
 * <p>Recovery starts when either error threshold is exceeded. Trajectory time remains paused until
 * both errors are within their resume tolerances. Resume tolerances must be less than or equal to
 * their corresponding error thresholds to provide hysteresis and avoid rapidly entering and leaving
 * recovery.
 *
 * @param translationErrorThresholdMeters translation error that starts recovery, in meters
 * @param headingErrorThresholdRadians absolute heading error that starts recovery, in radians
 * @param translationToleranceMeters translation error required to resume, in meters
 * @param headingToleranceRadians absolute heading error required to resume, in radians
 */
public record SwerveTrajectoryRecoveryConfig(
    double translationErrorThresholdMeters,
    double headingErrorThresholdRadians,
    double translationToleranceMeters,
    double headingToleranceRadians) {
  /**
   * Validates a swerve trajectory recovery configuration.
   *
   * @param translationErrorThresholdMeters translation error that starts recovery, in meters
   * @param headingErrorThresholdRadians absolute heading error that starts recovery, in radians
   * @param translationToleranceMeters translation error required to resume, in meters
   * @param headingToleranceRadians absolute heading error required to resume, in radians
   */
  public SwerveTrajectoryRecoveryConfig {
    requireFiniteNonnegative(translationErrorThresholdMeters, "translationErrorThresholdMeters");
    requireHeadingError(headingErrorThresholdRadians, "headingErrorThresholdRadians");
    requireFiniteNonnegative(translationToleranceMeters, "translationToleranceMeters");
    requireHeadingError(headingToleranceRadians, "headingToleranceRadians");

    if (translationToleranceMeters > translationErrorThresholdMeters) {
      throw new IllegalArgumentException(
          "translationToleranceMeters must be less than or equal to "
              + "translationErrorThresholdMeters");
    }
    if (headingToleranceRadians > headingErrorThresholdRadians) {
      throw new IllegalArgumentException(
          "headingToleranceRadians must be less than or equal to "
              + "headingErrorThresholdRadians");
    }
  }

  private static void requireFiniteNonnegative(double value, String name) {
    if (!Double.isFinite(value) || value < 0.0) {
      throw new IllegalArgumentException(name + " must be finite and nonnegative");
    }
  }

  private static void requireHeadingError(double value, String name) {
    requireFiniteNonnegative(value, name);
    if (value > Math.PI) {
      throw new IllegalArgumentException(name + " must be less than or equal to pi");
    }
  }

  boolean shouldStartRecovery(Pose2d currentPose, Pose2d targetPose) {
    return translationError(currentPose, targetPose) > translationErrorThresholdMeters
        || headingError(currentPose, targetPose) > headingErrorThresholdRadians;
  }

  boolean shouldResume(Pose2d currentPose, Pose2d targetPose) {
    return translationError(currentPose, targetPose) <= translationToleranceMeters
        && headingError(currentPose, targetPose) <= headingToleranceRadians;
  }

  private static double translationError(Pose2d currentPose, Pose2d targetPose) {
    return currentPose.getTranslation().getDistance(targetPose.getTranslation());
  }

  private static double headingError(Pose2d currentPose, Pose2d targetPose) {
    return Math.abs(
        MathUtil.angleModulus(
            currentPose.getRotation().getRadians() - targetPose.getRotation().getRadians()));
  }
}
