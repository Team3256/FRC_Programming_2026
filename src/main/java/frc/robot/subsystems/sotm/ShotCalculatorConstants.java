package frc.robot.subsystems.sotm;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;


public final class ShotCalculatorConstants {

  public static final double kGravity = 9.81; // m/s^2


  public static final int kMaxIterations = 10;
  public static final double kConvergenceThreshold = 0.01; // meters


  public static final Transform3d ROBOT_TO_SHOOTER = new Transform3d(
      new Translation3d(
          0.0,
          0.0,
          0.3
      ),
      new Rotation3d()
  );

  //distance to shooter speed
  //meters to RPS
  public static final InterpolatingDoubleTreeMap DISTANCE_TO_SHOOTER_SPEED =
      new InterpolatingDoubleTreeMap();


  //distance to pivot angle
  //meters to radians
  public static final InterpolatingDoubleTreeMap DISTANCE_TO_PIVOT_ANGLE =
      new InterpolatingDoubleTreeMap();

  static {

  //distance to shooter speed
  //meters to RPS
    DISTANCE_TO_SHOOTER_SPEED.put(1.0, 40.0);

   //distance to pivot angle
   //meters to radians
    DISTANCE_TO_PIVOT_ANGLE.put(1.0, Math.toRadians(45));
  }

  public static final double DEFAULT_PROJECTILE_VELOCITY = 10.0; // m/s

  public static final double MAX_SHOT_DISTANCE = 8.0; // meters
  public static final double MIN_SHOT_DISTANCE = 0.5; // meters
  public static final double MAX_PIVOT_ANGLE = Math.toRadians(60); // radians
  public static final double MIN_PIVOT_ANGLE = Math.toRadians(10); // radians
}
