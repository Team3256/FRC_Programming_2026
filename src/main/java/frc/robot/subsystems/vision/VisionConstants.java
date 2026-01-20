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
import java.util.List;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  // Camera names, must match names configured on coprocessor
  public static String leftCam = "left";
  public static String rightCam = "right";
  public static String frontCam = "front";
  public static String backCam = "back";

  public static List<Short> nonHubIds =
      List.of(
          (short) 6,
          (short) 7,
          (short) 16,
          (short) 15,
          (short) 14,
          (short) 13,
          (short) 12,
          (short) 1,
          (short) 22,
          (short) 23,
          (short) 32,
          (short) 31,
          (short) 30,
          (short) 29,
          (short) 28,
          (short) 17);

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static Transform3d robotToLeftCam =
      new Transform3d(
          Units.inchesToMeters(3),
          Units.inchesToMeters(6.565),
          Units.inchesToMeters(6.123),
          new Rotation3d(0.0, Units.degreesToRadians(-25), Units.degreesToRadians(90)));
  public static Transform3d robotToRightCam =
      new Transform3d(
          Units.inchesToMeters(-3),
          Units.inchesToMeters(-6.53),
          Units.inchesToMeters(6.109),
          new Rotation3d(0.0, Units.degreesToRadians(-25), Units.degreesToRadians(-90)));

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
        1.0, // Camera 0
        1.7, // Camera 1
        1.5,
        1.5 // Camera 3
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
