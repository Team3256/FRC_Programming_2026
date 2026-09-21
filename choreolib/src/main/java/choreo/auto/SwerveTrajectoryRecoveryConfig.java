// Copyright (c) Choreo contributors

package choreo.auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Configuration for pausing a swerve trajectory's progress when tracking error becomes too large.
 *
 * <p>Recovery starts when either error threshold is exceeded. While recovering, trajectory time is
 * paused and the robot is steered back onto the path: each cycle the point on the trajectory
 * closest to the robot (searching forward from the paused time, up to {@code
 * rejoinSearchWindowSeconds} ahead) becomes the target, and that sample's velocity feedforward is
 * kept so the robot rejoins the path already moving at the speed the trajectory expects there.
 * Trajectory time resumes from the rejoin point once both errors are within their resume
 * tolerances. Resume tolerances must be less than or equal to their corresponding error thresholds
 * to provide hysteresis and avoid rapidly entering and leaving recovery.
 *
 * <p>Recovery only starts once an error threshold has been exceeded continuously for {@code
 * startDebounceSeconds}, so a single bad pose estimate does not pause the trajectory. If the
 * factory is given a chassis speeds supplier, resuming additionally requires the robot's
 * field-relative translational velocity to be within {@code resumeVelocityToleranceMetersPerSecond}
 * of the rejoin sample's, so the trajectory does not resume while the robot is crossing the path at
 * the wrong speed and immediately fall off again.
 *
 * @param translationErrorThresholdMeters translation error that starts recovery, in meters
 * @param headingErrorThresholdRadians absolute heading error that starts recovery, in radians
 * @param translationToleranceMeters translation error required to resume, in meters
 * @param headingToleranceRadians absolute heading error required to resume, in radians
 * @param rejoinSearchWindowSeconds how far ahead of the paused trajectory time the rejoin point may
 *     advance during a single recovery, in seconds
 * @param startDebounceSeconds how long an error threshold must be continuously exceeded before
 *     recovery starts, in seconds
 * @param resumeVelocityToleranceMetersPerSecond translational velocity error required to resume, in
 *     meters per second; only applied when the factory has a chassis speeds supplier
 */
public record SwerveTrajectoryRecoveryConfig(
    double translationErrorThresholdMeters,
    double headingErrorThresholdRadians,
    double translationToleranceMeters,
    double headingToleranceRadians,
    double rejoinSearchWindowSeconds,
    double startDebounceSeconds,
    double resumeVelocityToleranceMetersPerSecond) {
  /** Default distance ahead of the paused time that the rejoin point may advance, in seconds. */
  public static final double kDefaultRejoinSearchWindowSeconds = 1.0;

  /** Default start debounce, in seconds. Zero starts recovery on the first cycle over threshold. */
  public static final double kDefaultStartDebounceSeconds = 0.0;

  /** Default resume velocity tolerance, in meters per second. Infinite disables the check. */
  public static final double kDefaultResumeVelocityToleranceMetersPerSecond =
      Double.POSITIVE_INFINITY;

  /**
   * Validates a swerve trajectory recovery configuration.
   *
   * @param translationErrorThresholdMeters translation error that starts recovery, in meters
   * @param headingErrorThresholdRadians absolute heading error that starts recovery, in radians
   * @param translationToleranceMeters translation error required to resume, in meters
   * @param headingToleranceRadians absolute heading error required to resume, in radians
   * @param rejoinSearchWindowSeconds how far ahead of the paused trajectory time the rejoin point
   *     may advance during a single recovery, in seconds
   * @param startDebounceSeconds how long an error threshold must be continuously exceeded before
   *     recovery starts, in seconds
   * @param resumeVelocityToleranceMetersPerSecond translational velocity error required to resume,
   *     in meters per second; may be infinite to disable the check
   */
  public SwerveTrajectoryRecoveryConfig {
    requireFiniteNonnegative(translationErrorThresholdMeters, "translationErrorThresholdMeters");
    requireHeadingError(headingErrorThresholdRadians, "headingErrorThresholdRadians");
    requireFiniteNonnegative(translationToleranceMeters, "translationToleranceMeters");
    requireHeadingError(headingToleranceRadians, "headingToleranceRadians");
    requireFiniteNonnegative(rejoinSearchWindowSeconds, "rejoinSearchWindowSeconds");
    requireFiniteNonnegative(startDebounceSeconds, "startDebounceSeconds");
    if (Double.isNaN(resumeVelocityToleranceMetersPerSecond)
        || resumeVelocityToleranceMetersPerSecond < 0.0) {
      throw new IllegalArgumentException(
          "resumeVelocityToleranceMetersPerSecond must be nonnegative");
    }

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

  /**
   * Creates a swerve trajectory recovery configuration using the default rejoin search window of
   * {@link #kDefaultRejoinSearchWindowSeconds}.
   *
   * @param translationErrorThresholdMeters translation error that starts recovery, in meters
   * @param headingErrorThresholdRadians absolute heading error that starts recovery, in radians
   * @param translationToleranceMeters translation error required to resume, in meters
   * @param headingToleranceRadians absolute heading error required to resume, in radians
   */
  public SwerveTrajectoryRecoveryConfig(
      double translationErrorThresholdMeters,
      double headingErrorThresholdRadians,
      double translationToleranceMeters,
      double headingToleranceRadians) {
    this(
        translationErrorThresholdMeters,
        headingErrorThresholdRadians,
        translationToleranceMeters,
        headingToleranceRadians,
        kDefaultRejoinSearchWindowSeconds);
  }

  /**
   * Creates a swerve trajectory recovery configuration with no start debounce and no resume
   * velocity check.
   *
   * @param translationErrorThresholdMeters translation error that starts recovery, in meters
   * @param headingErrorThresholdRadians absolute heading error that starts recovery, in radians
   * @param translationToleranceMeters translation error required to resume, in meters
   * @param headingToleranceRadians absolute heading error required to resume, in radians
   * @param rejoinSearchWindowSeconds how far ahead of the paused trajectory time the rejoin point
   *     may advance during a single recovery, in seconds
   */
  public SwerveTrajectoryRecoveryConfig(
      double translationErrorThresholdMeters,
      double headingErrorThresholdRadians,
      double translationToleranceMeters,
      double headingToleranceRadians,
      double rejoinSearchWindowSeconds) {
    this(
        translationErrorThresholdMeters,
        headingErrorThresholdRadians,
        translationToleranceMeters,
        headingToleranceRadians,
        rejoinSearchWindowSeconds,
        kDefaultStartDebounceSeconds,
        kDefaultResumeVelocityToleranceMetersPerSecond);
  }

  /**
   * Returns a copy of this configuration with the given start debounce.
   *
   * @param seconds how long an error threshold must be continuously exceeded before recovery
   *     starts, in seconds
   * @return the updated configuration
   */
  public SwerveTrajectoryRecoveryConfig withStartDebounce(double seconds) {
    return new SwerveTrajectoryRecoveryConfig(
        translationErrorThresholdMeters,
        headingErrorThresholdRadians,
        translationToleranceMeters,
        headingToleranceRadians,
        rejoinSearchWindowSeconds,
        seconds,
        resumeVelocityToleranceMetersPerSecond);
  }

  /**
   * Returns a copy of this configuration with the given resume velocity tolerance.
   *
   * @param metersPerSecond translational velocity error required to resume, in meters per second
   * @return the updated configuration
   */
  public SwerveTrajectoryRecoveryConfig withResumeVelocityTolerance(double metersPerSecond) {
    return new SwerveTrajectoryRecoveryConfig(
        translationErrorThresholdMeters,
        headingErrorThresholdRadians,
        translationToleranceMeters,
        headingToleranceRadians,
        rejoinSearchWindowSeconds,
        startDebounceSeconds,
        metersPerSecond);
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

  /**
   * Whether the robot's field-relative translational velocity is close enough to the target's to
   * resume.
   *
   * @param currentSpeeds the robot's current field-relative chassis speeds
   * @param targetSpeeds the rejoin sample's field-relative chassis speeds
   * @return true when the translational velocity error is within tolerance
   */
  boolean isVelocityWithinTolerance(ChassisSpeeds currentSpeeds, ChassisSpeeds targetSpeeds) {
    double error =
        Math.hypot(
            currentSpeeds.vxMetersPerSecond - targetSpeeds.vxMetersPerSecond,
            currentSpeeds.vyMetersPerSecond - targetSpeeds.vyMetersPerSecond);
    return error <= resumeVelocityToleranceMetersPerSecond;
  }

  static double translationError(Pose2d currentPose, Pose2d targetPose) {
    return currentPose.getTranslation().getDistance(targetPose.getTranslation());
  }

  private static double headingError(Pose2d currentPose, Pose2d targetPose) {
    return Math.abs(
        MathUtil.angleModulus(
            currentPose.getRotation().getRadians() - targetPose.getRotation().getRadians()));
  }
}
