// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.sotm.ShotCalculator;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.utils.DisableSubsystem;
import frc.robot.utils.LoggedTracer;
import frc.robot.utils.Util;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Turret extends DisableSubsystem {

  private final TurretIO TurretIO;
  private final TurretIOInputsAutoLogged TurretIOInputsAutoLogged = new TurretIOInputsAutoLogged();

  public final Trigger reachedPosition = new Trigger(this::reachedPosition);

  //    private final SimpleMotorFeedforward m_shooterFeedforward =
  //            new SimpleMotorFeedforward(
  //                    TurretConstants.kSVolts,
  // TurretConstants.kVVoltSecondsPerRotation);

  private final PIDController pivotRotationController =
      new PIDController(
          TurretConstants.trackingP, TurretConstants.trackingI, TurretConstants.trackingD);

  private double reqPosition = 0.0;

  public Turret(boolean enabled, TurretIO TurretIO) {
    super(enabled);

    this.TurretIO = TurretIO;
  }

  @Override
  public void periodic() {
    super.periodic();
    TurretIO.updateInputs(TurretIOInputsAutoLogged);
    Logger.processInputs("Turret", TurretIOInputsAutoLogged);

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
          TurretIO.setPosition(reqPosition);
        });
  }

  public Command setPositionFieldRelative(Rotation2d position, CommandSwerveDrivetrain swerve) {
    return this.run(
        () ->
            TurretIO.setPosition(
                position.minus(swerve.getState().Pose.getRotation()).getRotations()));
  }

  public double getPosition() {
    return TurretIOInputsAutoLogged.turretMotorPosition;
  }

  public Command setVoltage(double voltage) {
    return this.run(() -> TurretIO.setVoltage(voltage));
  }

  public double getVoltage() {
    return TurretIOInputsAutoLogged.turretMotorVoltage;
  }

  public Command trackTarget(ShotCalculator calc, Translation3d target) {

    calc.setTarget(new Pose3d(target, Rotation3d.kZero));
    return run(
        () -> {
          pivotRotationController.calculate(
              TurretIOInputsAutoLogged.turretMotorPosition, calc.getCurrentEffectiveYaw());
        });
  }

  public Command zero() {
    return this.runOnce(TurretIO::zero);
  }

  public Command off() {
    return this.runOnce(TurretIO::off).withName("off");
  }

  public boolean reachedPosition() {
    return Util.epsilonEquals(TurretIOInputsAutoLogged.turretMotorPosition, reqPosition, 0.01);
  }
}
