// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utils.DisableSubsystem;
import frc.robot.utils.LoggedTracer;
import frc.robot.utils.Util;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Turret extends DisableSubsystem {

  private final TurretIO turretIO;
  private final TurretIOInputsAutoLogged turretIOInputsAutoLogged = new TurretIOInputsAutoLogged();

  private double reqPosition = 0.0;

  private boolean hasBeenZeroed = false;

  public final Trigger reachedPosition = new Trigger(this::reachedPosition);

  public Turret(boolean enabled, TurretIO turretIO) {
    super(enabled);

    this.turretIO = turretIO;
    turretIO.resetPosition(0);
  }

  @Override
  public void periodic() {
    super.periodic();
    turretIO.updateInputs(turretIOInputsAutoLogged);
    Logger.processInputs("Turret", turretIOInputsAutoLogged);

    Logger.recordOutput(this.getClass().getSimpleName() + "/reqPosition", reqPosition);
    Logger.recordOutput(
        this.getClass().getSimpleName() + "/absTurretPos",
        solveTheta(
            TurretConstants.mainTurretGear,
            TurretConstants.cancoderGear1,
            TurretConstants.cancoderGear2,
            turretIOInputsAutoLogged.turretEncoder1AbsolutePosition,
            turretIOInputsAutoLogged.turretEncoder2AbsolutePosition));

    if (!hasBeenZeroed
        && solveTheta(
                TurretConstants.mainTurretGear,
                TurretConstants.cancoderGear1,
                TurretConstants.cancoderGear2,
                turretIOInputsAutoLogged.turretEncoder1AbsolutePosition,
                turretIOInputsAutoLogged.turretEncoder2AbsolutePosition)
            != 0) {
      turretIO.resetPosition(
          solveTheta(
                  TurretConstants.mainTurretGear,
                  TurretConstants.cancoderGear1,
                  TurretConstants.cancoderGear2,
                  turretIOInputsAutoLogged.turretEncoder1AbsolutePosition,
                  turretIOInputsAutoLogged.turretEncoder2AbsolutePosition)
              - TurretConstants.crtOffsetRot);
      hasBeenZeroed = true;
    }
    LoggedTracer.record("Turret");
  }

  public Command setVoltage(double voltage) {
    return this.run(() -> turretIO.setVoltage(voltage));
  }

  // Base
  public Command setPosition(double position) {
    return this.run(
        () -> {
          reqPosition = position;
          turretIO.setPosition(reqPosition);
        });
  }

  public Command pointToPose(Supplier<Pose2d> robotPose, Supplier<Pose2d> targetPose) {
    return this.run(
        () -> {
          Rotation2d rotation =
              targetPose.get().getTranslation().minus(robotPose.get().getTranslation()).getAngle();
          setPositionFieldRelative(rotation, robotPose.get().getRotation());
        });
  }

  private void setPositionFieldRelative(Rotation2d targetRot, Rotation2d robotRot) {
    reqPosition = targetRot.minus(robotRot).getRotations();
    turretIO.setPosition(-reqPosition);
  }

  public Command setPositionFieldRelative(
      Supplier<Rotation2d> position, Supplier<Rotation2d> robotRot) {
    return this.run(() -> setPositionFieldRelative(position.get(), robotRot.get()));
  }

  public static double solveTheta(int p, int q1, int q2, double a, double b) {
    double range = (double) Util.lcm(q1, q2) / p;
    double r1 = (double) q1 / p;
    double r2 = (double) q2 / p;

    double i = 0;
    double j = 0;
    while (i * r1 < range * 10 && j * r2 < range * 10) {
      double alpha = r1 * (i + a);
      double beta = r2 * (j + b);

      if (Math.abs(alpha - beta) < 1e-2) {
        return alpha;
      }

      if (alpha < beta) {
        i += 1;
      } else {
        j += 1;
      }
    }

    return 0;
  }

  public Command zero() {
    return this.runOnce(turretIO::zero);
  }

  public Command off() {
    return this.runOnce(turretIO::off).withName("off");
  }

  public boolean reachedPosition() {
    return Util.epsilonEquals(turretIOInputsAutoLogged.turretMotorPosition, reqPosition, 0.05);
  }
}
