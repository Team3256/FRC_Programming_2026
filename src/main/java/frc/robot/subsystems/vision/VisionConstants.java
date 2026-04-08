// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;

import java.io.IOException;
import java.nio.file.Path;

import static frc.robot.FieldConstants.fieldType;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout;

    static {
        try {
            aprilTagLayout = new AprilTagFieldLayout(Path.of(
                    Filesystem.getDeployDirectory().toString(),
                    "apriltags",
                    "welded",
                    "2026-practice" + ".json"));
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    // Camera names, must match names configured on coprocessor
  public static String frontRightCam = "frontRight";
  public static String backRightCam = "backRight";
  public static String frontLeftCam = "frontLeft";
  public static String backLeftCam = "backLeft";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static Transform3d robotToFrontRightCam =
      new Transform3d(
          Units.inchesToMeters(12.647),
          Units.inchesToMeters(-9.703),
          Units.inchesToMeters(13.518),
          new Rotation3d(0.0, Units.degreesToRadians(-20), Units.degreesToRadians(-45)));
  public static Transform3d robotToBackRightCam =
      new Transform3d(
          Units.inchesToMeters(11.916),
          Units.inchesToMeters(11.506),
          Units.inchesToMeters(28.702),
          new Rotation3d(0.0, Units.degreesToRadians(-10), Units.degreesToRadians(-135)));
  public static Transform3d robotToFrontLeftCam =
      new Transform3d(
          Units.inchesToMeters(13.267),
          Units.inchesToMeters(10.650),
          Units.inchesToMeters(26.339),
          new Rotation3d(0.0, Units.degreesToRadians(-15), Units.degreesToRadians(0)));

  public static Transform3d robotToBackLeftCam =
      new Transform3d(
          Units.inchesToMeters(10.082),
          Units.inchesToMeters(12.583),
          Units.inchesToMeters(26.839),
          new Rotation3d(0.0, Units.degreesToRadians(-15), Units.degreesToRadians(108)));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.99; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1, // Camera 0
        1, // Camera 1
        1,
        1 // Camera 3
      };
}
