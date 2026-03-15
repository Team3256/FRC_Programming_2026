// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utils.DisableSubsystem;
import frc.robot.utils.LoggedTracer;
import frc.robot.utils.Util;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends DisableSubsystem {

  private final ShooterIO shooterIO;
  private final ShooterIOInputsAutoLogged shooterIOAutoLogged = new ShooterIOInputsAutoLogged();

  private double reqVelocity = 0.0;

  public final Trigger reachedVelocity = new Trigger(this::reachedVelocity);

  public Shooter(boolean enabled, ShooterIO shooterIO) {
    super(enabled);
    this.shooterIO = shooterIO;
  }

  @Override
  public void periodic() {
    super.periodic();
    shooterIO.updateInputs(shooterIOAutoLogged);
    Logger.processInputs("Shooter", shooterIOAutoLogged);

    Logger.recordOutput(this.getClass().getSimpleName() + "/reqVelocity", reqVelocity);

    LoggedTracer.record("Shooter");
  }

  public Command setVoltage(double voltage) {
    return this.run(() -> shooterIO.setShooterVoltage(voltage)).finallyDo(shooterIO::off);
  }

  public Command setVelocity(double velocity) {
    return setVelocity(() -> velocity);
  }

  public Command setVelocity(DoubleSupplier velocity) {
    return this.run(
            () -> {
              reqVelocity = velocity.getAsDouble();
              shooterIO.setShooterVelocity(reqVelocity);
            })
        .finallyDo(shooterIO::off);
  }

  public Command shootHub(DoubleSupplier distance, DoubleSupplier velMultiplier) {
    return setVelocity(
        () -> (ShooterConstants.hubLUT.get(distance.getAsDouble()) * velMultiplier.getAsDouble()));
  }

  public Command feedCorner(DoubleSupplier distance) {
    return setVelocity(() -> ShooterConstants.feedLUT.get(distance.getAsDouble()));
  }

  public boolean reachedVelocity() {
    return Util.epsilonEquals(shooterIOAutoLogged.shooterMotorVelocity, reqVelocity, 5);
  }

  public Command off() {
    return this.runOnce(shooterIO::off);
  }
}
