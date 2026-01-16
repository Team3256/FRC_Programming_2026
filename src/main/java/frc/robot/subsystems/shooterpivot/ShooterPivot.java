// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooterpivot;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utils.DisableSubsystem;
import frc.robot.utils.LoggedTracer;
import frc.robot.utils.Util;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class ShooterPivot extends DisableSubsystem {

  private final ShooterPivotIO shooterPivotIO;
  private final ShooterPivotIOInputsAutoLogged shooterPivotIOInputsAutoLogged =
      new ShooterPivotIOInputsAutoLogged();

  public final Trigger reachedPosition = new Trigger(this::reachedPosition);

  private double reqPosition = 0.0;

  public ShooterPivot(boolean enabled, ShooterPivotIO shooterPivotIO) {
    super(enabled);

    this.shooterPivotIO = shooterPivotIO;

    this.shooterPivotIO.resetPosition(Rotations.of(.4267));
  }

  @Override
  public void periodic() {
    super.periodic();
    shooterPivotIO.updateInputs(shooterPivotIOInputsAutoLogged);
    Logger.processInputs("ShooterPivot", shooterPivotIOInputsAutoLogged);

    Logger.recordOutput(this.getClass().getSimpleName() + "/reqPosition", reqPosition);

    LoggedTracer.record("ShooterPivot");
  }

  public Command setPosition(double position) {
    return setPosition(() -> position);
  }

  public Command setPosition(DoubleSupplier position) {
    return this.run(
        () -> {
          reqPosition = position.getAsDouble();
          shooterPivotIO.setPosition(reqPosition);
        });
  }

  public Command setVoltage(double voltage) {
    return this.run(() -> shooterPivotIO.setVoltage(voltage));
  }

  public Command zero() {
    return this.runOnce(shooterPivotIO::zero);
  }

  public Command off() {
    return this.runOnce(shooterPivotIO::off).withName("off");
  }

  public Command goToStow() {
    return this.setPosition(ShooterPivotConstants.stowPosition);
  }

  public Command goToShoot() {
    return this.setPosition(ShooterPivotConstants.baseShootingPosition);
  }

  public boolean reachedPosition() {
    return Util.epsilonEquals(shooterPivotIOInputsAutoLogged.pivotMotorPosition, reqPosition, 0.01);
  }
}