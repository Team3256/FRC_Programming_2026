package frc.robot.subsystems.sotm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.FieldConstants;

import org.littletonrobotics.junction.Logger;

public class ShotCalculator {
  private static ShotCalculator instance;


  public static ShotCalculator getInstance() {
    if (instance == null) instance = new ShotCalculator();
    return instance;
  }

  public record ShootingParameters(
      boolean isValid,
      Rotation2d turretAngle,
      double turretVelocity,
      double hoodAngle,
      double hoodVelocity,
      double flywheelSpeed) {}

  private ShootingParameters latestParameters = null;


  private static double phaseDelay;
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  static {
 
    phaseDelay = 0.03;
    timeOfFlightMap.put(5.68, 1.16);
    timeOfFlightMap.put(4.55, 1.12);
    timeOfFlightMap.put(3.15, 1.11);
    timeOfFlightMap.put(1.88, 1.09);
    timeOfFlightMap.put(1.38, 0.90);
  }

  /**
   * Calculates shooting parameters based on provided robot pose and velocity.
   *
   * @param robotPose Robot's current pose
   * @param robotVelocity Robot's field-relative velocity
   * @param robotToTurret Robot-to-turret transform
   * @return ShootingParameters for this shot
   */
  public ShootingParameters getParameters(
      Pose2d robotPose, ChassisSpeeds robotVelocity, Transform2d robotToTurret) {
    // Reset cache
    if (latestParameters != null) return latestParameters;

    // Phase delay
    Pose2d estimatedPose =
        robotPose.exp(
            new Twist2d(
                robotVelocity.vxMetersPerSecond * phaseDelay,
                robotVelocity.vyMetersPerSecond * phaseDelay,
                robotVelocity.omegaRadiansPerSecond * phaseDelay));

    // Turret position
    Pose2d turretPosition = estimatedPose.transformBy(robotToTurret);

    // Target
    Translation2d target =
        FieldConstants.Hub.topCenterPoint.toTranslation2d();

    // Distance from turret to target
    double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());

    // Field-relative turret velocity
    double robotAngle = estimatedPose.getRotation().getRadians();
    double turretVelocityX =
        robotVelocity.vxMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
                * (robotToTurret.getY() * Math.cos(robotAngle)
                    - robotToTurret.getX() * Math.sin(robotAngle));
    double turretVelocityY =
        robotVelocity.vyMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
                * (robotToTurret.getX() * Math.cos(robotAngle)
                    - robotToTurret.getY() * Math.sin(robotAngle));

    // Iteratively calculate lookahead pose
    Pose2d lookaheadPose = turretPosition;
    double lookaheadTurretToTargetDistance = turretToTargetDistance;
    double timeOfFlight = 0.0;
    for (int i = 0; i < 20; i++) {
      timeOfFlight = timeOfFlightMap.get(lookaheadTurretToTargetDistance);
      double offsetX = turretVelocityX * timeOfFlight;
      double offsetY = turretVelocityY * timeOfFlight;
      lookaheadPose =
          new Pose2d(
              turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              turretPosition.getRotation());
    }


    Logger.recordOutput("ShotCalculator/LookaheadPose", lookaheadPose);
    Logger.recordOutput("ShotCalculator/TurretToTargetDistance", lookaheadTurretToTargetDistance);

    return latestParameters;
  }

}


//get pose
//do calculations in perioid
// take everything except last one as suppliers