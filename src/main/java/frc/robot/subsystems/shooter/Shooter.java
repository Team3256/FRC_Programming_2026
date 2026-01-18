// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.DisableSubsystem;
import frc.robot.utils.LoggedTracer;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends DisableSubsystem {

  private final ShooterIO shooterIO;
  private final ShooterIOInputsAutoLogged shooterIOAutoLogged = new ShooterIOInputsAutoLogged();

  public Shooter(boolean disabled, ShooterIO shooterIO) {
    super(disabled);
    this.shooterIO = shooterIO;
  }

  @Override
  public void periodic() {
    super.periodic();
    shooterIO.updateInputs(shooterIOAutoLogged);
    Logger.processInputs("Shooter", shooterIOAutoLogged);

    LoggedTracer.record("Shooter");
  }

  public Command setVoltage(double voltage) {
    return this.run(() -> shooterIO.setShooterVoltage(voltage)).finallyDo(shooterIO::off);
  }

  public Command setVelocity(double velocity) {
    return this.run(() -> shooterIO.setShooterVelocity(velocity)).finallyDo(shooterIO::off);
  }

  public Command setVelocity(DoubleSupplier velocity) {
    return this.run(() -> shooterIO.setShooterVelocity(velocity.getAsDouble()))
        .finallyDo(shooterIO::off);
  }

  public Command off() {
    return this.runOnce(shooterIO::off);
  }
}
