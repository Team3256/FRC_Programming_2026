// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utils.DisableSubsystem;
import frc.robot.utils.Util;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Climb extends DisableSubsystem {
  private final ClimbIO climbIO;
  private final ClimbIOInputsAutoLogged climbIOAutoLogged = new ClimbIOInputsAutoLogged();

  private final MutAngle requestedPosition = Rotations.of(0.0).mutableCopy();

  public final Trigger reachedPosition = new Trigger(this::isAtPosition);

  public Climb(boolean enabled, ClimbIO climbIO) {
    super(enabled);
    this.climbIO = climbIO;
  }

  @Override
  public void periodic() {
    super.periodic();
    climbIO.updateInputs(climbIOAutoLogged);
    Logger.processInputs(this.getClass().getSimpleName(), climbIOAutoLogged);
  }

  public Command setPosition(double position) {
    return this.run(
        () -> {
          requestedPosition.mut_replace(position, Rotations);
          climbIO.setPosition(requestedPosition);
        });
  }

  public Command setPosition(Angle position) {
    return this.run(
        () -> {
          requestedPosition.mut_replace(position);
          climbIO.setPosition(requestedPosition);
        });
  }

  @AutoLogOutput
  public boolean isAtPosition() {
    return Util.epsilonEquals(
        climbIOAutoLogged.climbMotorPosition, requestedPosition.in(Rotations), 0.05);
  }

  public Command setVoltage(double voltage) {
    return this.run(() -> climbIO.setVoltage(voltage)).withName("setVoltage_" + voltage);
  }

  public Command off() {
    return this.runOnce(climbIO::off);
  }

  public Command zero() {
    return this.runOnce(climbIO::zero);
  }
}
