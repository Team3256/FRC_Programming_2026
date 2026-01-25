// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.turret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.sotm.ShotCalculator;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.utils.DisableSubsystem;
import frc.robot.utils.LoggedTracer;
import frc.robot.utils.Util;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Turret extends DisableSubsystem {

  private final TurretIO turretIO;
  private final TurretIOInputsAutoLogged turretIOInputsAutoLogged = new TurretIOInputsAutoLogged();

  private double reqPosition = 0.0;

  private final PIDController turretRotationController =
      new PIDController(
          TurretConstants.trackingP, TurretConstants.trackingI, TurretConstants.trackingD);

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

  public Command setPosition(Rotation2d position) {
    return setPosition(() -> position);
  }

  public Command setPosition(Supplier<Rotation2d> position) {
    return this.run(
        () -> {
          reqPosition = position.get().getRotations();
          turretIO.setPosition(reqPosition);
        });
  }

  public Command setPositionFieldRelative(Rotation2d position, CommandSwerveDrivetrain swerve) {
    return this.run(
        () ->
            turretIO.setPosition(
                position.minus(swerve.getState().Pose.getRotation()).getRotations()));
  }

  public double getPosition() {
    return turretIOInputsAutoLogged.turretMotorPosition;
  }

  public Command setVoltage(double voltage) {
    return this.run(() -> turretIO.setVoltage(voltage));
  }

  public double getVoltage() {
    return turretIOInputsAutoLogged.turretMotorVoltage;
  }

  public Command trackTarget(ShotCalculator calc) {
    return run(
        () -> {
          turretRotationController.calculate(
              turretIOInputsAutoLogged.turretMotorPosition, calc.getCurrentEffectiveYaw());
        });
  }

  public Command zero() {
    return this.runOnce(turretIO::zero);
  }

  public Command off() {
    return this.runOnce(turretIO::off).withName("off");
  }

  public boolean reachedPosition() {
    return Util.epsilonEquals(turretIOInputsAutoLogged.turretMotorPosition, reqPosition, 0.01);
  }
}
