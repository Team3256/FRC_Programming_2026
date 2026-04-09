// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooterpivot;

import edu.wpi.first.math.filter.LinearFilter;
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

  private double lastHoodAngle = 0;

  private double hoodVel = 0;

  private final LinearFilter hoodAngleFilter = LinearFilter.movingAverage((int) (0.1 / .02));

  public ShooterPivot(boolean enabled, ShooterPivotIO shooterPivotIO) {
    super(enabled);

    this.shooterPivotIO = shooterPivotIO;

    shooterPivotIO.resetPosition(0.0);
  }

  @Override
  public void periodic() {
    super.periodic();
    shooterPivotIO.updateInputs(shooterPivotIOInputsAutoLogged);
    Logger.processInputs("ShooterPivot", shooterPivotIOInputsAutoLogged);

    Logger.recordOutput(this.getClass().getSimpleName() + "/reqPosition", reqPosition);

    hoodVel = hoodAngleFilter.calculate((reqPosition - lastHoodAngle) / 0.02);
    lastHoodAngle = reqPosition;
    Logger.recordOutput(this.getClass().getSimpleName() + "/turretVel", hoodVel);

    LoggedTracer.record("ShooterPivot");
  }

  public Command setPosition(double position) {
    return setPosition(() -> position);
  }

  public Command setPosition(DoubleSupplier position) {
    return this.run(
        () -> {
          reqPosition = position.getAsDouble();
          shooterPivotIO.setPosition(reqPosition, hoodVel);
        });
  }

  public Command setVoltage(double voltage) {
    return this.run(() -> shooterPivotIO.setVoltage(voltage));
  }

  public Command off() {
    return this.runOnce(shooterPivotIO::off).withName("off");
  }

  public Command shootHub(DoubleSupplier distance) {
    return setPosition(() -> ShooterPivotConstants.hubLUT.get(distance.getAsDouble()));
  }

  public Command feedCorner(DoubleSupplier distance) {
    return setPosition(() -> ShooterPivotConstants.feedLUT.get(distance.getAsDouble()));
  }

  public boolean reachedPosition() {
    return Util.epsilonEquals(
        shooterPivotIOInputsAutoLogged.shooterPivotMotorPosition, reqPosition, 0.01);
  }
}
