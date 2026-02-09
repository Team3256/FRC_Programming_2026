// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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

  public final Trigger reachedPosition = new Trigger(this::reachedPosition);

  public Turret(boolean enabled, TurretIO turretIO) {
    super(enabled);

    this.turretIO = turretIO;
  }

  @Override
  public void periodic() {
    super.periodic();
    turretIO.updateInputs(turretIOInputsAutoLogged);
    Logger.processInputs("Turret", turretIOInputsAutoLogged);

    Logger.recordOutput(this.getClass().getSimpleName() + "/reqPosition", reqPosition);

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
          Transform2d diff =
              robotPose
                  .get()
                  .transformBy(TurretConstants.driveBaseToTurret)
                  .minus(targetPose.get());
          Rotation2d rotation =
              new Rotation2d(diff.getX(), diff.getY()).plus(TurretConstants.turretOffset);
          setPositionFieldRelative(rotation, robotPose.get().getRotation());
        });
  }

  private void setPositionFieldRelative(Rotation2d targetRot, Rotation2d robotRot) {
    reqPosition = targetRot.minus(robotRot).getRotations();
    turretIO.setPosition(reqPosition);
  }

  public Command setPositionFieldRelative(
      Supplier<Rotation2d> position, Supplier<Rotation2d> robotRot) {
    return this.run(() -> setPositionFieldRelative(position.get(), robotRot.get()));
  }

  public static Rotation2d getTurretPosition(Rotation2d cancoder1, Rotation2d cancoder2) {
    Rotation2d diff = cancoder2.minus(cancoder1);
    double drivingRotRaw = diff.getDegrees() / TurretConstants.differenceDegrees;
    double expectedG1 = (drivingRotRaw * TurretConstants.ratio1) % 1;
    double g1Diff = expectedG1 - (cancoder1.getRotations() % 1);
    if (g1Diff > 0.5) {
      g1Diff -= 1;
    } else if (g1Diff < -0.5) {
      g1Diff += 1;
    }
    double drivingRem = g1Diff / TurretConstants.ratio1;
    return Rotation2d.fromRotations(drivingRotRaw - drivingRem);
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
