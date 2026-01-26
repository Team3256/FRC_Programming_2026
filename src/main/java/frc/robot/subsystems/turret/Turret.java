// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.sotm.ShotCalculator;
import frc.robot.utils.DisableSubsystem;
import frc.robot.utils.LoggedTracer;
import frc.robot.utils.Util;
import org.littletonrobotics.junction.Logger;

public class Turret extends DisableSubsystem {

  private final TurretIO turretIO;
  private final TurretIOInputsAutoLogged turretIOInputsAutoLogged = new TurretIOInputsAutoLogged();

  private double reqPosition = 0.0;

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

  public Command setPosition(double position) {
    return this.run(
        () -> {
          reqPosition = position;
          turretIO.setPosition(reqPosition);
        });
  }

  public Command setVoltage(double voltage) {
    return this.run(() -> turretIO.setVoltage(voltage));
  }

  public Command trackTarget(ShotCalculator calc, Pose3d target, double targetSpeed) {
    calc.setTarget(target, targetSpeed);
    return this.run(
        () -> {
          reqPosition = calc.getCurrentEffectiveYaw();
          this.setPosition(reqPosition);
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
