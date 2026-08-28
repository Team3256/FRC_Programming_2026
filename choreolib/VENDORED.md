# Vendored ChoreoLib

This directory contains the ChoreoLib source used by this robot project. Gradle includes it as a
composite build, so a normal clone of `FRC_Programming_2026` contains everything needed to compile
the custom trajectory-recovery API. No separate Choreo checkout or locally published JAR is
required.

The source is based on upstream Choreo commit `764ff38` with the team's opt-in swerve trajectory
recovery changes on top. The custom implementation is intentionally limited to:

- `src/main/java/choreo/auto/AutoFactory.java`
- `src/main/java/choreo/auto/AutoTrajectory.java`
- `src/main/java/choreo/auto/SwerveTrajectoryRecoveryConfig.java`
- `src/test/java/choreo/auto/SwerveTrajectoryRecoveryTest.java`

## Updating from upstream

1. Merge or rebase the latest authors' release into the separate Choreo fork.
2. Run the ChoreoLib test and Javadoc suite in that fork.
3. Copy the fork's `choreolib` directory into this directory, excluding `.gradle` and `build`.
4. Copy Choreo's root `LICENSE.txt` to this directory.
5. Run `./gradlew spotlessCheck test` from the robot-project root.

Keeping the recovery change confined to the files above minimizes conflicts when refreshing the
vendored copy.
