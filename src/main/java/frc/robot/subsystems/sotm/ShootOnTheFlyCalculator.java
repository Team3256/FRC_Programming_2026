package frc.robot.subsystems.sotm;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ShootOnTheFlyCalculator {

    public record InterceptSolution(
            Pose3d effectiveTargetPose,
            double launchPitchRad,
            double launchSpeed,
            double flightTime,
            double requiredYaw) {
    }

    public static InterceptSolution solveShootOnTheFly(
            Pose3d shooterPose,
            Pose3d targetPose,
            ChassisSpeeds fieldRelRobotVelocity,
            ChassisAccelerations fieldRelRobotAcceleration,
            double targetSpeedRps,
            int maxIterations,
            double timeTolerance) {

        BallPhysics.ShotSolution sol = BallPhysics.solveBallisticWithSpeed(
                shooterPose,
                targetPose,
                targetSpeedRps);

        double t = sol.flightTimeSeconds();
        Pose3d effectiveTarget = targetPose;

        for (int i = 0; i < maxIterations; i++) {

            double dx = fieldRelRobotVelocity.vxMetersPerSecond * t;

            double dy = fieldRelRobotVelocity.vyMetersPerSecond * t;

            effectiveTarget = new Pose3d(
                    targetPose.getX() - dx,
                    targetPose.getY() - dy,
                    targetPose.getZ(),
                    targetPose.getRotation());

            BallPhysics.ShotSolution newSol = BallPhysics.solveBallisticWithSpeed(
                    shooterPose,
                    effectiveTarget,
                    targetSpeedRps);

            if (Math.abs(newSol.flightTimeSeconds() - t) < timeTolerance) {
                return new InterceptSolution(
                        effectiveTarget,
                        newSol.launchPitchRad(),
                        newSol.launchSpeed(),
                        newSol.flightTimeSeconds(),
                        0);
            }

            sol = newSol;
            t = newSol.flightTimeSeconds();
        }

        return new InterceptSolution(
                effectiveTarget,
                sol.launchPitchRad(),
                sol.launchSpeed(),
                sol.flightTimeSeconds(),
                0);
    }
}
