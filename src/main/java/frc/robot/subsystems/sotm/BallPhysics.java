package frc.robot.subsystems.sotm;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;

public final class BallPhysics {

    public record ShotSolution(
            double launchPitchRad,
            double launchSpeed,
            double flightTimeSeconds) {
    }

    private BallPhysics() {
    }

    public static ShotSolution solveBallisticWithIncomingAngle(
            Pose3d shooterPose,
            Pose3d targetPose,
            double incomingPitchRad) {

        Translation3d s = shooterPose.getTranslation();
        Translation3d t = targetPose.getTranslation();

        double dx = t.getX() - s.getX();
        double dy = t.getY() - s.getY();
        double dz = t.getZ() - s.getZ();

        double d = Math.hypot(dx, dy);
        if (d < 1e-9) {
            throw new IllegalArgumentException("horizontal distance too small");
        }

        double tanThetaT = Math.tan(incomingPitchRad);

        double rhs = dz - d * tanThetaT;
        if (rhs <= 0) {
            throw new IllegalArgumentException(
                    "no physical solution: dz - d*tan(thetaT) must be > 0");
        }

        double T = Math.sqrt(2.0 * rhs / ShotCalculatorConstants.kGravity);

        double vHoriz = d / T;
        double vZ0 = vHoriz * tanThetaT + ShotCalculatorConstants.kGravity * T;

        double launchSpeed = Math.hypot(vHoriz, vZ0);
        double launchPitch = Math.atan2(vZ0, vHoriz);

        return new ShotSolution(launchPitch, launchSpeed, T);
    }

    public static ShotSolution solveBallisticWithSpeed(
            Pose3d shooterPose,
            Pose3d targetPose,
            double launchSpeed) {

        Translation3d s = shooterPose.getTranslation();
        Translation3d t = targetPose.getTranslation();

        double dx = t.getX() - s.getX();
        double dy = t.getY() - s.getY();
        double dz = t.getZ() - s.getZ();

        double d = Math.hypot(dx, dy);
        if (d < 1e-9) {
            throw new IllegalArgumentException("Horizontal distance too small");
        }

        double v2 = launchSpeed * launchSpeed;
        double g = ShotCalculatorConstants.kGravity;

        double discriminant = v2 * v2 - g * (g * d * d + 2.0 * dz * v2);
        if (discriminant < 0) {
            return new ShotSolution(0, 0, 0);
        }

        double tanTheta = (v2 + Math.sqrt(discriminant)) / (g * d);

        double launchPitch = Math.atan(tanTheta);

        double vHoriz = launchSpeed * Math.cos(launchPitch);
        double time = d / vHoriz;

        return new ShotSolution(launchPitch, launchSpeed, time);
    }

    public static double minSpeedForAnyArc(
            Pose3d shooterPose,
            Pose3d targetPose) {

        Translation3d s = shooterPose.getTranslation();
        Translation3d t = targetPose.getTranslation();

        double dx = t.getX() - s.getX();
        double dy = t.getY() - s.getY();
        double dz = t.getZ() - s.getZ();

        double d = Math.hypot(dx, dy);

        return Math.sqrt(
                ShotCalculatorConstants.kGravity * (Math.hypot(d, dz) + dz));
    }

    public static double getTimeToShoot(
            Pose3d shooterPose,
            Pose3d targetPose,
            double launchSpeed,
            double launchPitchRad) {
        Translation3d s = shooterPose.getTranslation();
        Translation3d t = targetPose.getTranslation();

        double dx = t.getX() - s.getX();
        double dy = t.getY() - s.getY();

        double horizontalDist = Math.hypot(dx, dy);
        double vHoriz = launchSpeed * Math.cos(launchPitchRad);

        if (vHoriz <= 1e-6) {
            throw new IllegalArgumentException("horizontal velocity too small");
        }

        return horizontalDist / vHoriz;
    }
}
